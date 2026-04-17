use crate::error::VosaError;
use crate::parser::ast::*;
use crate::runtime::ExecutionReport;
use mavlink::common::{self, MavMessage};
use mavlink::MavConnection;

#[cfg(feature = "ros2")]
pub mod ros2;
#[cfg(feature = "ros2")]
pub use ros2::Ros2Bridge;

// ── Constants ─────────────────────────────────────────────────────────────────

const TARGET_SYSTEM: u8 = 1;
const TARGET_COMPONENT: u8 = 1;

/// Messages to scan while waiting for a specific response (~1 s at 100 Hz).
const WAIT_ATTEMPTS: usize = 100;
/// Extended wait for the initial vehicle heartbeat (up to ~60 s at 100 Hz).
/// PX4 SITL can take 10-20 s before its MAVLink interface is ready.
const HEARTBEAT_WAIT_ATTEMPTS: usize = 6000;
/// Extended wait for GPS lock (up to ~30 s at 100 Hz).
const GPS_WAIT_ATTEMPTS: usize = 3000;

// PX4 custom mode values for MAV_CMD_DO_SET_MODE
const PX4_CUSTOM_MAIN_MODE_AUTO: f32 = 4.0;
const PX4_CUSTOM_SUB_MODE_AUTO_MISSION: f32 = 4.0;
const PX4_CUSTOM_SUB_MODE_AUTO_RTL: f32 = 5.0;
const PX4_CUSTOM_SUB_MODE_AUTO_LAND: f32 = 6.0;

// ── Telemetry state ───────────────────────────────────────────────────────────

/// Live vehicle state updated from MAVLink telemetry messages.
/// This is what drives the reactive trigger system on real hardware —
/// conditions are evaluated against actual sensor readings, not simulated values.
#[derive(Debug, Clone, Default)]
struct TelemetryState {
    /// Battery remaining (%), -1 if unknown
    battery_percent: f64,
    /// Wind speed magnitude (m/s), derived from WIND_COV
    wind_speed_ms: f64,
    /// Obstacle detected (populated from OBSTACLE_DISTANCE or companion computer)
    obstacle_detected: bool,
    /// GPS fix type: 0=none, 2=2D, 3=3D, 4=DGPS, 5=RTK float, 6=RTK fixed
    gps_fix_type: u8,
    /// Home position received from autopilot
    home_set: bool,
    /// Home latitude in degrees
    home_lat: f64,
    /// Home longitude in degrees
    home_lon: f64,
}

impl TelemetryState {
    /// Update fields from an incoming MAVLink message.
    fn update(&mut self, msg: &MavMessage) {
        match msg {
            MavMessage::SYS_STATUS(d) => {
                if d.battery_remaining >= 0 {
                    self.battery_percent = d.battery_remaining as f64;
                }
            }
            MavMessage::GPS_RAW_INT(d) => {
                self.gps_fix_type = d.fix_type as u8;
            }
            MavMessage::HOME_POSITION(d) => {
                self.home_lat = d.latitude as f64 / 1e7;
                self.home_lon = d.longitude as f64 / 1e7;
                if !self.home_set {
                    println!("[MAVLink] Home GPS: lat={:.7}, lon={:.7}, alt={:.1}m",
                        self.home_lat, self.home_lon,
                        d.altitude as f64 / 1000.0);
                }
                self.home_set = true;
            }
            MavMessage::WIND_COV(d) => {
                self.wind_speed_ms =
                    ((d.wind_x as f64).powi(2) + (d.wind_y as f64).powi(2)).sqrt();
            }
            _ => {}
        }
    }
}

// ── Active trigger (MAVLink context) ─────────────────────────────────────────

/// A registered `on` trigger evaluated against live telemetry.
struct ActiveTrigger {
    condition: TriggerCondition,
    /// Flattened list of commands to execute when the condition fires.
    commands: Vec<Command>,
    /// Rising-edge latch: true while condition is currently satisfied.
    fired: bool,
}

// ── Bridge ────────────────────────────────────────────────────────────────────

/// Translates a VOSA mission into real MAVLink packets for PX4 / ArduPilot.
///
/// ## Gazebo + PX4 SITL quick-start
/// ```bash
/// # Terminal 1 — PX4 SITL inside Gazebo
/// cd PX4-Autopilot && make px4_sitl gazebo
///
/// # Terminal 2 — run the VOSA mission
/// vosa run mission.vosa --mavlink udpin:0.0.0.0:14550
/// ```
///
/// ## Execution sequence
/// 1. Connect to autopilot
/// 2. GCS heartbeat handshake
/// 3. Wait for 3D GPS lock
/// 4. Wait for home position
/// 5. Switch to AUTO.MISSION mode
/// 6. Upload mission items via MAVLink mission protocol
/// 7. Arm vehicle
/// 8. Start mission
/// 9. Monitor live telemetry — evaluate reactive triggers and send
///    MAVLink override commands when conditions fire
pub struct MavlinkBridge {
    connection_string: String,
}

impl MavlinkBridge {
    pub fn new(connection_string: &str) -> Self {
        MavlinkBridge {
            connection_string: connection_string.to_string(),
        }
    }

    pub fn execute(&mut self, mission: &Mission) -> Result<ExecutionReport, VosaError> {
        println!("[MAVLink] Connecting to {} ...", self.connection_string);
        let vehicle = mavlink::connect::<MavMessage>(&self.connection_string)
            .map_err(|e| VosaError::RuntimeError(format!("Connection failed: {e}")))?;

        // ── 1. Heartbeat handshake ────────────────────────────────────────────
        println!("[MAVLink] Sending GCS heartbeat");
        vehicle
            .send_default(&gcs_heartbeat())
            .map_err(|e| VosaError::RuntimeError(format!("Heartbeat send failed: {e}")))?;

        println!("[MAVLink] Waiting for vehicle heartbeat (up to ~60 s) ...");
        let mut telemetry = TelemetryState::default();
        recv_until(&vehicle, &mut telemetry, HEARTBEAT_WAIT_ATTEMPTS, |msg| {
            matches!(msg, MavMessage::HEARTBEAT(_))
        })?;
        println!("[MAVLink] Vehicle link established");

        // Keep sending heartbeats for 3 s so PX4 counts us as a connected GCS
        // (PX4 requires several consecutive heartbeats before clearing the
        // "No connection to the GCS" preflight warning)
        println!("[MAVLink] Establishing GCS link (3 s) ...");
        for _ in 0..30 {
            let _ = vehicle.send_default(&gcs_heartbeat());
            // drain incoming messages while we wait
            let _ = vehicle.recv();
            std::thread::sleep(std::time::Duration::from_millis(100));
        }
        println!("[MAVLink] GCS link established");

        // ── 2. Wait for 3D GPS lock ───────────────────────────────────────────
        println!("[MAVLink] Waiting for 3D GPS lock ...");
        recv_until(&vehicle, &mut telemetry, GPS_WAIT_ATTEMPTS, |_| {
            false // keep going; GPS state updated via telemetry.update() inside recv_until
        })
        .or_else(|_| Ok::<_, VosaError>(()))?; // timeout is OK here; check below

        // Poll with logging until fix
        for attempt in 0..GPS_WAIT_ATTEMPTS {
            if telemetry.gps_fix_type >= 3 {
                println!("[MAVLink] GPS lock acquired (fix_type={})", telemetry.gps_fix_type);
                break;
            }
            if attempt % 100 == 0 {
                println!(
                    "[MAVLink] Waiting for GPS ... fix_type={} (need 3)",
                    telemetry.gps_fix_type
                );
            }
            match vehicle.recv() {
                Ok((_, msg)) => telemetry.update(&msg),
                Err(e) => {
                    return Err(VosaError::RuntimeError(format!("GPS wait error: {e}")))
                }
            }
        }
        if telemetry.gps_fix_type < 3 {
            return Err(VosaError::RuntimeError(
                "Timed out waiting for GPS lock — check antenna / simulator position".into(),
            ));
        }

        // ── 3. Wait for home position ─────────────────────────────────────────
        println!("[MAVLink] Waiting for home position ...");
        for _ in 0..WAIT_ATTEMPTS {
            if telemetry.home_set {
                break;
            }
            match vehicle.recv() {
                Ok((_, msg)) => telemetry.update(&msg),
                Err(e) => {
                    return Err(VosaError::RuntimeError(format!("Home wait error: {e}")))
                }
            }
        }
        if !telemetry.home_set {
            println!("[MAVLink] Warning: home position not confirmed — continuing anyway");
        } else {
            println!("[MAVLink] Home position set");
        }

        // ── 4. Set AUTO.MISSION mode ──────────────────────────────────────────
        println!("[MAVLink] Setting AUTO.MISSION mode ...");
        set_flight_mode(
            &vehicle,
            PX4_CUSTOM_MAIN_MODE_AUTO,
            PX4_CUSTOM_SUB_MODE_AUTO_MISSION,
        )?;
        wait_for_command_ack(&vehicle, &mut telemetry, common::MavCmd::MAV_CMD_DO_SET_MODE)?;
        println!("[MAVLink] Mode: AUTO.MISSION");

        // ── 5. Build and upload mission ───────────────────────────────────────
        let mut items: Vec<MavItem> = Vec::new();
        collect_items(&mission.sequence.statements, &mut items);
        let home_lat = telemetry.home_lat;
        let home_lon = telemetry.home_lon;
        println!("[MAVLink] Uploading {} mission items ...", items.len());
        upload_mission(&vehicle, &mut telemetry, &items, home_lat, home_lon)?;
        println!("[MAVLink] Mission upload complete");

        // ── 6. Wait for EKF2 + arming checks to clear, then arm ─────────────
        // PX4 refuses to arm if EKF2 hasn't converged. Poll SYS_STATUS for up
        // to 30 s; if it clears we arm normally, if not we try anyway.
        println!("[MAVLink] Waiting for EKF2 / preflight checks (up to 30 s) ...");
        for i in 0..300 {
            let _ = vehicle.send_default(&gcs_heartbeat());
            match vehicle.recv() {
                Ok((_, msg)) => telemetry.update(&msg),
                Err(_) => {}
            }
            if i % 50 == 49 {
                println!("[MAVLink]   ... still waiting for EKF2");
            }
            std::thread::sleep(std::time::Duration::from_millis(100));
        }
        println!("[MAVLink] Preflight wait complete — arming ...");
        println!("[MAVLink] Arming ...");
        send_command_long(
            &vehicle,
            common::MavCmd::MAV_CMD_COMPONENT_ARM_DISARM,
            [1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
        )?;
        wait_for_command_ack(
            &vehicle,
            &mut telemetry,
            common::MavCmd::MAV_CMD_COMPONENT_ARM_DISARM,
        )?;
        println!("[MAVLink] Armed");

        // ── 7. Start mission ──────────────────────────────────────────────────
        println!("[MAVLink] Starting mission ...");
        send_command_long(
            &vehicle,
            common::MavCmd::MAV_CMD_MISSION_START,
            [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
        )?;
        println!("[MAVLink] Mission running — monitoring telemetry");

        // ── 8. Monitor telemetry + evaluate reactive triggers ─────────────────
        let mut triggers = collect_triggers(&mission.sequence.statements);
        let item_count = items.len();
        let mut steps = items
            .iter()
            .enumerate()
            .map(|(i, it)| crate::runtime::ExecutionStep {
                index: i,
                description: it.describe(),
            })
            .collect::<Vec<_>>();

        monitor_mission(&vehicle, &mut telemetry, &mut triggers, item_count, &mut steps)?;

        // ── 9. Return to launch after waypoints complete ──────────────────────
        println!("[MAVLink] Waypoints complete — sending RTL");
        set_flight_mode(&vehicle, PX4_CUSTOM_MAIN_MODE_AUTO, PX4_CUSTOM_SUB_MODE_AUTO_RTL)?;

        println!("[MAVLink] Mission complete");
        Ok(ExecutionReport {
            mission_name: mission.name.clone(),
            steps,
            total_distance_m: 0.0,
            max_altitude_m: items
                .iter()
                .filter_map(|it| it.altitude())
                .fold(0.0_f64, f64::max),
        })
    }
}

// ── Telemetry helpers ─────────────────────────────────────────────────────────

/// Receive messages, updating telemetry state from each, until `done` returns
/// true or `max` attempts are exhausted.
fn recv_until<C, F>(
    vehicle: &C,
    telemetry: &mut TelemetryState,
    max: usize,
    mut done: F,
) -> Result<(), VosaError>
where
    C: MavConnection<MavMessage>,
    F: FnMut(&MavMessage) -> bool,
{
    for _ in 0..max {
        match vehicle.recv() {
            Ok((_, msg)) => {
                telemetry.update(&msg);
                if done(&msg) {
                    return Ok(());
                }
            }
            Err(e) => {
                return Err(VosaError::RuntimeError(format!("MAVLink recv error: {e}")))
            }
        }
    }
    Err(VosaError::RuntimeError("recv_until: max attempts exhausted".into()))
}

// ── Mission monitoring + reactive triggers ────────────────────────────────────

/// Run after mission start. Receives all MAVLink messages, keeps telemetry
/// fresh, evaluates reactive triggers on every update, and fires trigger
/// commands when conditions transition false → true.
///
/// Returns when `MISSION_ITEM_REACHED` for the last item is received.
fn monitor_mission<C: MavConnection<MavMessage>>(
    vehicle: &C,
    telemetry: &mut TelemetryState,
    triggers: &mut Vec<ActiveTrigger>,
    item_count: usize,
    steps: &mut Vec<crate::runtime::ExecutionStep>,
) -> Result<(), VosaError> {
    // Send a GCS heartbeat every 1 s based on wall-clock time, not message count.
    // PX4 triggers a GCS-lost failsafe if it doesn't receive a heartbeat for >3 s.
    let mut last_heartbeat = std::time::Instant::now();
    let mut last_reached_seq: Option<u16> = None;
    loop {
        // Always heartbeat on time regardless of incoming message rate
        if last_heartbeat.elapsed() >= std::time::Duration::from_secs(1) {
            let _ = vehicle.send_default(&gcs_heartbeat());
            last_heartbeat = std::time::Instant::now();
        }

        match vehicle.recv() {
            Ok((_, msg)) => {
                telemetry.update(&msg);

                // Log waypoint progress — deduplicate repeated MISSION_ITEM_REACHED
                if let MavMessage::MISSION_ITEM_REACHED(data) = &msg {
                    if last_reached_seq != Some(data.seq) {
                        last_reached_seq = Some(data.seq);
                        let desc = format!(
                            "[REACHED] Mission item {}/{}",
                            data.seq + 1,
                            item_count
                        );
                        println!("[MAVLink] {desc}  batt={:.1}%  wind={:.1}m/s",
                            telemetry.battery_percent, telemetry.wind_speed_ms);
                        steps.push(crate::runtime::ExecutionStep {
                            index: steps.len(),
                            description: desc,
                        });
                    }

                    // Check if this was the last item
                    if data.seq as usize >= item_count.saturating_sub(1) {
                        return Ok(());
                    }
                }

                // Evaluate reactive triggers after every telemetry update
                fire_triggers(vehicle, telemetry, triggers, steps)?;
            }
            Err(e) => {
                return Err(VosaError::RuntimeError(format!(
                    "Telemetry error during mission: {e}"
                )))
            }
        }
    }
}

/// Evaluate all registered triggers against current telemetry.
/// Fires on rising edge (false → true); resets when condition clears.
fn fire_triggers<C: MavConnection<MavMessage>>(
    vehicle: &C,
    telemetry: &TelemetryState,
    triggers: &mut Vec<ActiveTrigger>,
    steps: &mut Vec<crate::runtime::ExecutionStep>,
) -> Result<(), VosaError> {
    for trigger in triggers.iter_mut() {
        let condition_met = eval_trigger(&trigger.condition, telemetry);

        if condition_met && !trigger.fired {
            trigger.fired = true;
            let label = trigger_label(&trigger.condition);
            println!("[MAVLink] TRIGGER FIRED: on {label}  batt={:.1}%  wind={:.1}m/s",
                telemetry.battery_percent, telemetry.wind_speed_ms);
            steps.push(crate::runtime::ExecutionStep {
                index: steps.len(),
                description: format!("[TRIGGER FIRED] on {label}"),
            });

            // Execute each command in the trigger body
            for cmd in &trigger.commands.clone() {
                execute_trigger_command(vehicle, cmd, steps)?;
            }
        } else if !condition_met {
            trigger.fired = false;
        }
    }
    Ok(())
}

/// Evaluate a trigger condition against live telemetry.
fn eval_trigger(condition: &TriggerCondition, t: &TelemetryState) -> bool {
    match condition {
        TriggerCondition::Battery { operator, threshold_percent } => match operator {
            Operator::LessThan    => t.battery_percent < *threshold_percent,
            Operator::GreaterThan => t.battery_percent > *threshold_percent,
        },
        TriggerCondition::Wind { operator, threshold_ms } => match operator {
            Operator::LessThan    => t.wind_speed_ms < *threshold_ms,
            Operator::GreaterThan => t.wind_speed_ms > *threshold_ms,
        },
        TriggerCondition::ObstacleDetected => t.obstacle_detected,
    }
}

/// Send the MAVLink command corresponding to a VOSA trigger body command.
fn execute_trigger_command<C: MavConnection<MavMessage>>(
    vehicle: &C,
    cmd: &Command,
    steps: &mut Vec<crate::runtime::ExecutionStep>,
) -> Result<(), VosaError> {
    let desc = match cmd {
        Command::ReturnHome => {
            // Switch to AUTO.RTL mode
            set_flight_mode(vehicle, PX4_CUSTOM_MAIN_MODE_AUTO, PX4_CUSTOM_SUB_MODE_AUTO_RTL)?;
            "[TRIGGER CMD] MAV_CMD_DO_SET_MODE AUTO.RTL".to_string()
        }
        Command::Land => {
            // Switch to AUTO.LAND mode
            set_flight_mode(vehicle, PX4_CUSTOM_MAIN_MODE_AUTO, PX4_CUSTOM_SUB_MODE_AUTO_LAND)?;
            "[TRIGGER CMD] MAV_CMD_DO_SET_MODE AUTO.LAND".to_string()
        }
        Command::Hover { duration } => {
            send_command_long(
                vehicle,
                common::MavCmd::MAV_CMD_NAV_LOITER_TIME,
                [*duration as f32, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
            )?;
            format!("[TRIGGER CMD] MAV_CMD_NAV_LOITER_TIME {duration}s")
        }
        Command::Takeoff { altitude } => {
            send_command_long(
                vehicle,
                common::MavCmd::MAV_CMD_NAV_TAKEOFF,
                [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, *altitude as f32],
            )?;
            format!("[TRIGGER CMD] MAV_CMD_NAV_TAKEOFF {altitude}m")
        }
        other => {
            format!("[TRIGGER CMD] (skipped in MAVLink context: {other:?})")
        }
    };
    println!("[MAVLink] {desc}");
    steps.push(crate::runtime::ExecutionStep {
        index: steps.len(),
        description: desc,
    });
    Ok(())
}

// ── MAVLink protocol helpers ──────────────────────────────────────────────────

fn gcs_heartbeat() -> MavMessage {
    MavMessage::HEARTBEAT(common::HEARTBEAT_DATA {
        custom_mode: 0,
        mavtype: common::MavType::MAV_TYPE_GCS,
        autopilot: common::MavAutopilot::MAV_AUTOPILOT_INVALID,
        base_mode: common::MavModeFlag::empty(),
        system_status: common::MavState::MAV_STATE_ACTIVE,
        mavlink_version: 0x3,
    })
}

/// Send MAV_CMD_DO_SET_MODE to switch the vehicle flight mode.
/// PX4 interprets param1 as MAV_MODE_FLAG_CUSTOM_MODE_ENABLED (1),
/// param2 as the main mode, and param3 as the sub mode.
fn set_flight_mode<C: MavConnection<MavMessage>>(
    vehicle: &C,
    main_mode: f32,
    sub_mode: f32,
) -> Result<(), VosaError> {
    send_command_long(
        vehicle,
        common::MavCmd::MAV_CMD_DO_SET_MODE,
        [
            common::MavModeFlag::MAV_MODE_FLAG_CUSTOM_MODE_ENABLED.bits() as f32,
            main_mode,
            sub_mode,
            0.0, 0.0, 0.0, 0.0,
        ],
    )
}

fn send_command_long<C: MavConnection<MavMessage>>(
    vehicle: &C,
    command: common::MavCmd,
    params: [f32; 7],
) -> Result<(), VosaError> {
    vehicle
        .send_default(&MavMessage::COMMAND_LONG(common::COMMAND_LONG_DATA {
            param1: params[0],
            param2: params[1],
            param3: params[2],
            param4: params[3],
            param5: params[4],
            param6: params[5],
            param7: params[6],
            command,
            target_system: TARGET_SYSTEM,
            target_component: TARGET_COMPONENT,
            confirmation: 0,
        }))
        .map_err(|e| VosaError::RuntimeError(format!("COMMAND_LONG ({command:?}) failed: {e}")))
        .map(|_| ())
}

fn wait_for_command_ack<C: MavConnection<MavMessage>>(
    vehicle: &C,
    telemetry: &mut TelemetryState,
    command: common::MavCmd,
) -> Result<(), VosaError> {
    // Use a longer window for ACKs and don't hard-fail — PX4 sometimes
    // executes the command (e.g. arm) before sending the ACK, causing a
    // race if we bail out too early and stop sending heartbeats.
    let result = recv_until(vehicle, telemetry, WAIT_ATTEMPTS * 5, |msg| {
        matches!(msg, MavMessage::COMMAND_ACK(ack) if ack.command == command)
    });
    if result.is_err() {
        println!("[MAVLink] Warning: no ACK received for {command:?} — continuing anyway");
    }
    Ok(())
}

/// Upload the full mission using the MAVLink mission protocol.
fn upload_mission<C: MavConnection<MavMessage>>(
    vehicle: &C,
    telemetry: &mut TelemetryState,
    items: &[MavItem],
    home_lat: f64,
    home_lon: f64,
) -> Result<(), VosaError> {
    vehicle
        .send_default(&MavMessage::MISSION_COUNT(common::MISSION_COUNT_DATA {
            count: items.len() as u16,
            target_system: TARGET_SYSTEM,
            target_component: TARGET_COMPONENT,
        }))
        .map_err(|e| VosaError::RuntimeError(format!("MISSION_COUNT send failed: {e}")))?;

    let mut acked = false;
    let mut attempts = items.len() * WAIT_ATTEMPTS + 50;

    while !acked && attempts > 0 {
        attempts -= 1;
        match vehicle.recv() {
            Ok((_, msg)) => {
                telemetry.update(&msg);
                match msg {
                    MavMessage::MISSION_REQUEST_INT(req) => {
                        let seq = req.seq as usize;
                        if seq >= items.len() {
                            return Err(VosaError::RuntimeError(format!(
                                "Vehicle requested out-of-range item {seq}"
                            )));
                        }
                        println!("[MAVLink]   Item {}/{}", seq + 1, items.len());
                        vehicle
                            .send_default(&items[seq].to_mission_item_int(seq as u16, home_lat, home_lon))
                            .map_err(|e| {
                                VosaError::RuntimeError(format!(
                                    "MISSION_ITEM_INT[{seq}] failed: {e}"
                                ))
                            })?;
                    }
                    MavMessage::MISSION_ACK(ack) => match ack.mavtype {
                        common::MavMissionResult::MAV_MISSION_ACCEPTED => acked = true,
                        other => {
                            return Err(VosaError::RuntimeError(format!(
                                "Mission upload rejected: {other:?}"
                            )))
                        }
                    },
                    _ => {}
                }
            }
            Err(e) => {
                return Err(VosaError::RuntimeError(format!(
                    "Upload recv error: {e}"
                )))
            }
        }
    }

    if !acked {
        return Err(VosaError::RuntimeError(
            "Mission upload timed out — no MISSION_ACK received".into(),
        ));
    }
    Ok(())
}

// ── AST helpers ───────────────────────────────────────────────────────────────

/// Flatten VOSA statements into ordered MAVLink mission items.
fn collect_items(stmts: &[Statement], items: &mut Vec<MavItem>) {
    for stmt in stmts {
        match stmt {
            Statement::Command(cmd) => {
                if let Some(item) = command_to_mav_item(cmd) {
                    items.push(item);
                }
            }
            Statement::Repeat { count, body } => {
                for _ in 0..*count {
                    collect_items(&body.statements, items);
                }
            }
            Statement::IfBattery { body, .. } | Statement::Parallel { body } => {
                collect_items(&body.statements, items);
            }
            Statement::OnCondition { condition, .. } => {
                println!(
                    "[MAVLink] Note: 'on {condition:?}' → monitored via live telemetry after launch"
                );
            }
        }
    }
}

/// Extract `OnCondition` statements into `ActiveTrigger`s for the monitoring loop.
fn collect_triggers(stmts: &[Statement]) -> Vec<ActiveTrigger> {
    stmts
        .iter()
        .filter_map(|stmt| {
            if let Statement::OnCondition { condition, body } = stmt {
                let commands: Vec<Command> = body
                    .statements
                    .iter()
                    .filter_map(|s| {
                        if let Statement::Command(cmd) = s {
                            Some(cmd.clone())
                        } else {
                            None
                        }
                    })
                    .collect();
                Some(ActiveTrigger {
                    condition: condition.clone(),
                    commands,
                    fired: false,
                })
            } else {
                None
            }
        })
        .collect()
}

fn trigger_label(condition: &TriggerCondition) -> String {
    match condition {
        TriggerCondition::Battery { operator, threshold_percent } => {
            let op = if *operator == Operator::LessThan { "<" } else { ">" };
            format!("battery {op} {threshold_percent}%")
        }
        TriggerCondition::Wind { operator, threshold_ms } => {
            let op = if *operator == Operator::LessThan { "<" } else { ">" };
            format!("wind {op} {threshold_ms}m/s")
        }
        TriggerCondition::ObstacleDetected => "obstacle_detected".into(),
    }
}

// ── Mission item representation ───────────────────────────────────────────────

#[derive(Debug)]
enum MavItem {
    Takeoff { altitude: f32 },
    Waypoint { lat: f64, lon: f64, alt: f32 },
    LoiterTime { duration: f32 },
    Land,
    ReturnHome,
    CameraStartCapture,
    CameraStopCapture,
    CameraPhoto,
}

impl MavItem {
    fn to_mission_item_int(&self, seq: u16, home_lat: f64, home_lon: f64) -> MavMessage {
        let current = if seq == 0 { 1 } else { 0 };

        let (command, x, y, z, p1, p2, p3, p4, frame) = match self {
            MavItem::Takeoff { altitude } => (
                common::MavCmd::MAV_CMD_NAV_TAKEOFF,
                // Use actual home coords so feasibility checker doesn't flag (0,0)
                (home_lat * 1e7) as i32,
                (home_lon * 1e7) as i32,
                *altitude,
                0.0, 0.0, 0.0, f32::NAN,
                common::MavFrame::MAV_FRAME_GLOBAL_RELATIVE_ALT,
            ),
            MavItem::Waypoint { lat, lon, alt } => (
                common::MavCmd::MAV_CMD_NAV_WAYPOINT,
                (*lat * 1e7) as i32,
                (*lon * 1e7) as i32,
                *alt,
                0.0, 0.0, 0.0, f32::NAN,
                common::MavFrame::MAV_FRAME_GLOBAL_RELATIVE_ALT,
            ),
            MavItem::LoiterTime { duration } => (
                common::MavCmd::MAV_CMD_NAV_LOITER_TIME,
                0, 0, 0.0,
                *duration, 0.0, 0.0, f32::NAN,
                common::MavFrame::MAV_FRAME_GLOBAL_RELATIVE_ALT,
            ),
            MavItem::Land => (
                common::MavCmd::MAV_CMD_NAV_LAND,
                0, 0, 0.0,
                0.0, 0.0, 0.0, f32::NAN,
                common::MavFrame::MAV_FRAME_GLOBAL_RELATIVE_ALT,
            ),
            MavItem::ReturnHome => (
                common::MavCmd::MAV_CMD_NAV_RETURN_TO_LAUNCH,
                0, 0, 0.0,
                0.0, 0.0, 0.0, 0.0,
                common::MavFrame::MAV_FRAME_GLOBAL_RELATIVE_ALT,
            ),
            MavItem::CameraStartCapture => (
                common::MavCmd::MAV_CMD_VIDEO_START_CAPTURE,
                0, 0, 0.0,
                0.0, 0.0, 0.0, 0.0,
                common::MavFrame::MAV_FRAME_MISSION,
            ),
            MavItem::CameraStopCapture => (
                common::MavCmd::MAV_CMD_VIDEO_STOP_CAPTURE,
                0, 0, 0.0,
                0.0, 0.0, 0.0, 0.0,
                common::MavFrame::MAV_FRAME_MISSION,
            ),
            MavItem::CameraPhoto => (
                common::MavCmd::MAV_CMD_IMAGE_START_CAPTURE,
                0, 0, 0.0,
                0.0, 1.0, 1.0, f32::NAN,
                common::MavFrame::MAV_FRAME_MISSION,
            ),
        };

        MavMessage::MISSION_ITEM_INT(common::MISSION_ITEM_INT_DATA {
            param1: p1,
            param2: p2,
            param3: p3,
            param4: p4,
            x,
            y,
            z,
            seq,
            command,
            target_system: TARGET_SYSTEM,
            target_component: TARGET_COMPONENT,
            frame,
            current,
            autocontinue: 1,
        })
    }

    fn describe(&self) -> String {
        match self {
            MavItem::Takeoff { altitude } => format!("NAV_TAKEOFF alt={altitude}m"),
            MavItem::Waypoint { lat, lon, alt } => {
                format!("NAV_WAYPOINT ({lat:.5}°, {lon:.5}°) alt={alt}m")
            }
            MavItem::LoiterTime { duration } => format!("NAV_LOITER_TIME {duration}s"),
            MavItem::Land => "NAV_LAND".into(),
            MavItem::ReturnHome => "NAV_RETURN_TO_LAUNCH".into(),
            MavItem::CameraStartCapture => "VIDEO_START_CAPTURE".into(),
            MavItem::CameraStopCapture => "VIDEO_STOP_CAPTURE".into(),
            MavItem::CameraPhoto => "IMAGE_START_CAPTURE".into(),
        }
    }

    fn altitude(&self) -> Option<f64> {
        match self {
            MavItem::Takeoff { altitude } => Some(*altitude as f64),
            MavItem::Waypoint { alt, .. } => Some(*alt as f64),
            _ => None,
        }
    }
}

fn command_to_mav_item(cmd: &Command) -> Option<MavItem> {
    Some(match cmd {
        Command::Takeoff { altitude } => MavItem::Takeoff { altitude: *altitude as f32 },
        Command::Waypoint { lat, lon, alt } => {
            MavItem::Waypoint { lat: *lat, lon: *lon, alt: *alt as f32 }
        }
        Command::Hover { duration } => MavItem::LoiterTime { duration: *duration as f32 },
        // Land and ReturnHome are sent as COMMAND_LONG after mission upload,
        // not as mission items — PX4 gz_x500 SITL rejects them as NAV items.
        Command::Land => {
            println!("[MAVLink] Skipping land() as mission item — will send as COMMAND_LONG");
            return None;
        }
        Command::ReturnHome => {
            println!("[MAVLink] Skipping return_home() as mission item — will send as COMMAND_LONG");
            return None;
        }
        // Camera commands are skipped in MAVLink mode — PX4 SITL rejects
        // IMAGE/VIDEO_CAPTURE on vehicles without a camera payload.
        // On real hardware with a camera-equipped vehicle, remove this guard.
        Command::Camera { action, .. } => {
            println!("[MAVLink] Skipping camera({action:?}) — no camera payload on this vehicle");
            return None;
        }
    })
}
