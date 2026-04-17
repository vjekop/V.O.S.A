use crate::error::VosaError;
use crate::parser::ast::*;
use crate::runtime::ExecutionReport;
use mavlink::common::{self, MavMessage};
use mavlink::MavConnection;

pub mod ros2;
pub use ros2::Ros2Bridge;

// ── Constants ─────────────────────────────────────────────────────────────────

/// MAVLink system ID of the target autopilot (PX4/ArduPilot default)
const TARGET_SYSTEM: u8 = 1;
/// MAVLink component ID of the autopilot
const TARGET_COMPONENT: u8 = 1;
/// Max messages to sift through while waiting for a specific response.
/// At ~100 Hz MAVLink throughput this is ~1 second of tolerance.
const MAX_RECV_ATTEMPTS: usize = 100;

// ── Bridge ────────────────────────────────────────────────────────────────────

/// Translates a VOSA mission into real MAVLink packets and uploads it to an
/// autopilot (PX4 SITL, ArduPilot SITL, or physical hardware).
///
/// ## Gazebo + PX4 SITL usage
/// ```bash
/// # In one terminal: start PX4 SITL with Gazebo
/// make px4_sitl gazebo
///
/// # In another: run your VOSA mission
/// vosa run mission.vosa --mavlink udpin:0.0.0.0:14550
/// ```
///
/// ## Protocol
/// 1. Open TCP/UDP connection to the autopilot
/// 2. Send a GCS `HEARTBEAT` to identify ourselves
/// 3. Wait for the vehicle's `HEARTBEAT` to confirm link
/// 4. Upload the mission via the MAVLink mission protocol:
///    `MISSION_COUNT` → (per-item) `MISSION_REQUEST_INT` ↔ `MISSION_ITEM_INT` → `MISSION_ACK`
/// 5. Arm the vehicle via `MAV_CMD_COMPONENT_ARM_DISARM`
/// 6. Start the mission via `MAV_CMD_MISSION_START`
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
            .map_err(|e| VosaError::RuntimeError(format!("MAVLink connection failed: {e}")))?;

        // ── Step 1: Announce ourselves as a GCS ──────────────────────────────
        println!("[MAVLink] Sending GCS heartbeat");
        vehicle
            .send_default(&gcs_heartbeat())
            .map_err(|e| VosaError::RuntimeError(format!("Heartbeat send failed: {e}")))?;

        // ── Step 2: Wait for vehicle heartbeat (confirms link is live) ───────
        println!("[MAVLink] Waiting for vehicle heartbeat ...");
        wait_for_heartbeat(&vehicle)?;
        println!("[MAVLink] Vehicle link established");

        // ── Step 3: Flatten the VOSA AST into ordered MAVLink mission items ──
        let mut items: Vec<MavItem> = Vec::new();
        collect_items(&mission.sequence.statements, &mut items);
        println!("[MAVLink] Mission has {} items", items.len());

        // ── Step 4: Upload mission via MAVLink mission protocol ───────────────
        upload_mission(&vehicle, &items)?;
        println!("[MAVLink] Mission upload complete");

        // ── Step 5: Arm ───────────────────────────────────────────────────────
        println!("[MAVLink] Arming vehicle ...");
        send_command_long(
            &vehicle,
            common::MavCmd::MAV_CMD_COMPONENT_ARM_DISARM,
            [1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
        )?;
        wait_for_command_ack(&vehicle, common::MavCmd::MAV_CMD_COMPONENT_ARM_DISARM)?;
        println!("[MAVLink] Vehicle armed");

        // ── Step 6: Start mission ─────────────────────────────────────────────
        println!("[MAVLink] Starting mission ...");
        send_command_long(
            &vehicle,
            common::MavCmd::MAV_CMD_MISSION_START,
            [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
        )?;
        println!("[MAVLink] Mission running on vehicle");

        // ── Build execution report from the items we uploaded ─────────────────
        let steps = items
            .iter()
            .enumerate()
            .map(|(i, item)| crate::runtime::ExecutionStep {
                index: i,
                description: item.describe(),
            })
            .collect();

        Ok(ExecutionReport {
            mission_name: mission.name.clone(),
            steps,
            total_distance_m: 0.0, // live telemetry required for real tracking
            max_altitude_m: items
                .iter()
                .filter_map(|it| it.altitude())
                .fold(0.0_f64, f64::max),
        })
    }
}

// ── MAVLink message helpers ───────────────────────────────────────────────────

/// GCS identification heartbeat — sent once at startup to announce ourselves.
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

/// Block until we receive a HEARTBEAT from the vehicle (confirms the link).
fn wait_for_heartbeat<C: MavConnection<MavMessage>>(vehicle: &C) -> Result<(), VosaError> {
    for _ in 0..MAX_RECV_ATTEMPTS {
        match vehicle.recv() {
            Ok((_, MavMessage::HEARTBEAT(_))) => return Ok(()),
            Ok(_) => continue,
            Err(e) => {
                return Err(VosaError::RuntimeError(format!(
                    "Error waiting for vehicle heartbeat: {e}"
                )))
            }
        }
    }
    Err(VosaError::RuntimeError(
        "Timed out waiting for vehicle heartbeat — is the autopilot running?".into(),
    ))
}

/// Upload the full mission using the MAVLink mission protocol.
///
/// Protocol flow:
/// 1. GCS sends `MISSION_COUNT`
/// 2. Vehicle responds with `MISSION_REQUEST_INT` for each item (may be out of order)
/// 3. GCS responds to each request with the corresponding `MISSION_ITEM_INT`
/// 4. Vehicle sends `MISSION_ACK` when all items are received
fn upload_mission<C: MavConnection<MavMessage>>(
    vehicle: &C,
    items: &[MavItem],
) -> Result<(), VosaError> {
    // Tell the vehicle how many items are coming
    vehicle
        .send_default(&MavMessage::MISSION_COUNT(common::MISSION_COUNT_DATA {
            count: items.len() as u16,
            target_system: TARGET_SYSTEM,
            target_component: TARGET_COMPONENT,
        }))
        .map_err(|e| VosaError::RuntimeError(format!("MISSION_COUNT send failed: {e}")))?;

    // Respond to each MISSION_REQUEST_INT from the vehicle
    let mut acked = false;
    let mut attempts = items.len() * MAX_RECV_ATTEMPTS + 50;

    while !acked && attempts > 0 {
        attempts -= 1;
        match vehicle.recv() {
            Ok((_, MavMessage::MISSION_REQUEST_INT(req))) => {
                let seq = req.seq as usize;
                if seq >= items.len() {
                    return Err(VosaError::RuntimeError(format!(
                        "Vehicle requested out-of-range mission item {seq} (have {})",
                        items.len()
                    )));
                }
                println!("[MAVLink]   Uploading item {}/{}", seq + 1, items.len());
                vehicle
                    .send_default(&items[seq].to_mission_item_int(seq as u16))
                    .map_err(|e| {
                        VosaError::RuntimeError(format!("MISSION_ITEM_INT[{seq}] send failed: {e}"))
                    })?;
            }
            Ok((_, MavMessage::MISSION_ACK(ack))) => {
                match ack.mavtype {
                    common::MavMissionResult::MAV_MISSION_ACCEPTED => {
                        acked = true;
                    }
                    other => {
                        return Err(VosaError::RuntimeError(format!(
                            "Vehicle rejected mission upload: {other:?}"
                        )));
                    }
                }
            }
            Ok(_) => {} // ignore unrelated messages (telemetry, etc.)
            Err(e) => {
                return Err(VosaError::RuntimeError(format!(
                    "MAVLink receive error during upload: {e}"
                )))
            }
        }
    }

    if !acked {
        return Err(VosaError::RuntimeError(
            "Mission upload timed out — vehicle did not ACK".into(),
        ));
    }

    Ok(())
}

/// Send a `COMMAND_LONG` to the autopilot.
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

/// Wait for a `COMMAND_ACK` confirming the given command was accepted.
fn wait_for_command_ack<C: MavConnection<MavMessage>>(
    vehicle: &C,
    command: common::MavCmd,
) -> Result<(), VosaError> {
    for _ in 0..MAX_RECV_ATTEMPTS {
        match vehicle.recv() {
            Ok((_, MavMessage::COMMAND_ACK(ack))) if ack.command == command => {
                return match ack.result {
                    common::MavResult::MAV_RESULT_ACCEPTED => Ok(()),
                    other => Err(VosaError::RuntimeError(format!(
                        "Command {command:?} rejected by autopilot: {other:?}"
                    ))),
                };
            }
            Ok(_) => continue,
            Err(e) => {
                return Err(VosaError::RuntimeError(format!(
                    "Error waiting for COMMAND_ACK: {e}"
                )))
            }
        }
    }
    Err(VosaError::RuntimeError(format!(
        "Timed out waiting for COMMAND_ACK for {command:?}"
    )))
}

// ── Mission item representation ───────────────────────────────────────────────

/// An intermediate representation of a single MAVLink mission item.
/// Built by flattening the VOSA AST before upload.
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
    /// Convert to a `MISSION_ITEM_INT` MAVLink message for upload.
    fn to_mission_item_int(&self, seq: u16) -> MavMessage {
        // Item 0 is marked `current = 1` so the autopilot starts there.
        let current = if seq == 0 { 1 } else { 0 };

        let (command, x, y, z, p1, p2, p3, p4, frame) = match self {
            MavItem::Takeoff { altitude } => (
                common::MavCmd::MAV_CMD_NAV_TAKEOFF,
                0, 0, *altitude,
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
            MavItem::Takeoff { altitude } => format!("MAV_CMD_NAV_TAKEOFF alt={altitude}m"),
            MavItem::Waypoint { lat, lon, alt } => {
                format!("MAV_CMD_NAV_WAYPOINT ({lat:.5}°, {lon:.5}°) alt={alt}m")
            }
            MavItem::LoiterTime { duration } => {
                format!("MAV_CMD_NAV_LOITER_TIME duration={duration}s")
            }
            MavItem::Land => "MAV_CMD_NAV_LAND".into(),
            MavItem::ReturnHome => "MAV_CMD_NAV_RETURN_TO_LAUNCH".into(),
            MavItem::CameraStartCapture => "MAV_CMD_VIDEO_START_CAPTURE".into(),
            MavItem::CameraStopCapture => "MAV_CMD_VIDEO_STOP_CAPTURE".into(),
            MavItem::CameraPhoto => "MAV_CMD_IMAGE_START_CAPTURE".into(),
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

/// Flatten a VOSA statement list into an ordered list of `MavItem`s.
///
/// - `Repeat` blocks are unrolled into repeated items (missions are static plans).
/// - `IfBattery` bodies are included unconditionally — the flight controller's
///   own failsafe system handles battery-triggered behaviour.
/// - `OnCondition` triggers are skipped here; on real hardware they are wired to
///   the autopilot's event/failsafe system via parameter configuration.
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
                // Reactive triggers are not mission items — they map to autopilot
                // failsafe parameters or companion computer logic. Log and skip.
                println!(
                    "[MAVLink] Note: 'on {condition:?}' trigger — configure matching \
                     failsafe on the autopilot (e.g. PX4 EKF2_BATT_THR / COM_LOW_BAT_ACT)"
                );
            }
        }
    }
}

fn command_to_mav_item(cmd: &Command) -> Option<MavItem> {
    Some(match cmd {
        Command::Takeoff { altitude } => MavItem::Takeoff { altitude: *altitude as f32 },
        Command::Waypoint { lat, lon, alt } => MavItem::Waypoint {
            lat: *lat,
            lon: *lon,
            alt: *alt as f32,
        },
        Command::Hover { duration } => MavItem::LoiterTime { duration: *duration as f32 },
        Command::Land => MavItem::Land,
        Command::ReturnHome => MavItem::ReturnHome,
        Command::Camera { action, .. } => match action {
            CameraAction::Record => MavItem::CameraStartCapture,
            CameraAction::Stop => MavItem::CameraStopCapture,
            CameraAction::Photo => MavItem::CameraPhoto,
        },
    })
}
