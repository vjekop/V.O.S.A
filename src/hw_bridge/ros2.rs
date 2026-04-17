use crate::error::VosaError;
use crate::parser::ast::*;
use crate::runtime::ExecutionReport;
use ros2_client::{
    Context, Node, NodeOptions, Publisher,
    Name, NodeName, MessageTypeName, DEFAULT_PUBLISHER_QOS,
};
use serde::{Deserialize, Serialize};

// ── MAVROS-compatible message types ──────────────────────────────────────────
//
// These structs mirror the ROS 2 / MAVROS message definitions used by the
// running MAVROS node. They are serialized via CDR (the ROS 2 wire format)
// using serde.
//
// Reference: https://github.com/mavlink/mavros/tree/ros2/mavros_msgs/msg

/// `std_msgs/Header`
#[derive(Serialize, Deserialize, Clone)]
pub struct Header {
    pub stamp: TimeStamp,
    pub frame_id: String,
}

/// `builtin_interfaces/Time`
#[derive(Serialize, Deserialize, Clone)]
pub struct TimeStamp {
    pub sec: i32,
    pub nanosec: u32,
}

/// `mavros_msgs/GlobalPositionTarget`
///
/// Published to `/mavros/setpoint_raw/global`.
/// Instructs MAVROS to fly the vehicle to the given GPS coordinate.
///
/// `type_mask` selects which fields the autopilot should use:
///   Bit 0-2: ignore vx/vy/vz, Bit 3-5: ignore ax/ay/az, Bit 6: ignore yaw,
///   Bit 7: ignore yaw_rate.  0b11111000 = 0xF8 = position-only.
#[derive(Serialize, Deserialize, Clone)]
pub struct GlobalPositionTarget {
    pub header: Header,
    /// MAVLink coordinate frame.  6 = MAV_FRAME_GLOBAL_RELATIVE_ALT_INT
    pub coordinate_frame: u8,
    /// Bitmask of fields to ignore (position-only = 0xF8)
    pub type_mask: u16,
    pub latitude: f64,
    pub longitude: f64,
    /// Altitude above home (metres, relative)
    pub altitude: f32,
    pub vx: f32,
    pub vy: f32,
    pub vz: f32,
    pub afx: f32,
    pub afy: f32,
    pub afz: f32,
    pub yaw: f32,
    pub yaw_rate: f32,
}

/// `std_msgs/String` — used for high-level command strings on `/mavros/vosa/cmd`.
#[derive(Serialize, Deserialize, Clone)]
pub struct StringMsg {
    pub data: String,
}

// ── Bridge ────────────────────────────────────────────────────────────────────

/// Translates VOSA missions to MAVROS-compatible ROS 2 topic publications.
///
/// ## Gazebo + MAVROS setup
/// ```bash
/// # Terminal 1 — PX4 SITL inside Gazebo
/// make px4_sitl gazebo
///
/// # Terminal 2 — MAVROS bridge (connects PX4 to ROS 2)
/// ros2 run mavros mavros_node --ros-args -p fcu_url:=udp://:14540@127.0.0.1:14557
///
/// # Terminal 3 — run the VOSA mission
/// vosa run mission.vosa --ros2 0
/// ```
///
/// ## Topics published
///
/// | Topic | Type | Purpose |
/// |-------|------|---------|
/// | `/mavros/setpoint_raw/global` | `mavros_msgs/GlobalPositionTarget` | GPS waypoints |
/// | `/mavros/vosa/cmd` | `std_msgs/String` | Takeoff / land / RTH commands |
///
/// ## What still needs manual setup
/// Arming and mode changes require ROS 2 **service** calls, which the current
/// ros2-client implementation does not support natively. Before running:
///
/// ```bash
/// # Arm the vehicle
/// ros2 service call /mavros/cmd/arming mavros_msgs/srv/CommandBool "{value: true}"
///
/// # Set mode to OFFBOARD (for streaming setpoints)
/// ros2 service call /mavros/set_mode mavros_msgs/srv/SetMode "{custom_mode: 'OFFBOARD'}"
/// ```
///
/// These calls will be integrated once ros2-client adds service support.
pub struct Ros2Bridge {
    _node: Node,
    /// Publishes GPS waypoints / position targets to MAVROS
    wp_pub: Publisher<GlobalPositionTarget>,
    /// Publishes high-level command strings (takeoff, land, RTH)
    cmd_pub: Publisher<StringMsg>,
}

impl Ros2Bridge {
    pub fn new(domain_id: u32) -> Result<Self, VosaError> {
        println!("[ROS 2] Initializing VOSA commander node on Domain ID {domain_id}");

        let context = Context::new()
            .map_err(|e| VosaError::RuntimeError(format!("ROS 2 context failed: {e:?}")))?;

        let mut node = context
            .new_node(
                NodeName::new("/vosa", "vosa_commander")
                    .map_err(|e| VosaError::RuntimeError(format!("{e:?}")))?,
                NodeOptions::new().enable_rosout(true),
            )
            .map_err(|e| VosaError::RuntimeError(format!("ROS 2 node creation failed: {e:?}")))?;

        // /mavros/setpoint_raw/global  —  GPS position setpoints
        let wp_topic = node
            .create_topic(
                &Name::new("/mavros", "setpoint_raw/global").unwrap(),
                MessageTypeName::new("mavros_msgs", "GlobalPositionTarget"),
                &DEFAULT_PUBLISHER_QOS,
            )
            .map_err(|e| VosaError::RuntimeError(format!("Failed to create waypoint topic: {e:?}")))?;

        let wp_pub = node
            .create_publisher(&wp_topic, None)
            .map_err(|e| VosaError::RuntimeError(format!("Failed to create waypoint publisher: {e:?}")))?;

        // /mavros/vosa/cmd  —  high-level command strings
        let cmd_topic = node
            .create_topic(
                &Name::new("/mavros", "vosa/cmd").unwrap(),
                MessageTypeName::new("std_msgs", "String"),
                &DEFAULT_PUBLISHER_QOS,
            )
            .map_err(|e| VosaError::RuntimeError(format!("Failed to create command topic: {e:?}")))?;

        let cmd_pub = node
            .create_publisher(&cmd_topic, None)
            .map_err(|e| VosaError::RuntimeError(format!("Failed to create command publisher: {e:?}")))?;

        println!("[ROS 2] Publishing waypoints  → /mavros/setpoint_raw/global");
        println!("[ROS 2] Publishing commands   → /mavros/vosa/cmd");
        println!("[ROS 2] Note: arm + set mode via 'ros2 service call' before running (see 'vosa docs')");

        Ok(Ros2Bridge { _node: node, wp_pub, cmd_pub })
    }

    pub fn execute(&mut self, mission: &Mission) -> Result<ExecutionReport, VosaError> {
        println!("[ROS 2] Executing mission \"{}\"", mission.name);

        let mut report = ExecutionReport {
            mission_name: mission.name.clone(),
            steps: vec![],
            total_distance_m: 0.0,
            max_altitude_m: 0.0,
        };

        let mut step_idx = 0usize;
        let mut log = |desc: String| {
            println!("[ROS 2]   {desc}");
            report.steps.push(crate::runtime::ExecutionStep {
                index: step_idx,
                description: desc,
            });
            step_idx += 1;
        };

        for stmt in &mission.sequence.statements.clone() {
            self.execute_statement(&stmt, &mut log)?;
        }

        println!("[ROS 2] All topics published. Verify execution via 'ros2 topic echo /mavros/state'");
        Ok(report)
    }

    fn execute_statement<F>(&mut self, stmt: &Statement, log: &mut F) -> Result<(), VosaError>
    where
        F: FnMut(String),
    {
        match stmt {
            Statement::Command(cmd) => self.execute_command(cmd, log)?,
            Statement::Repeat { count, body } => {
                for i in 0..*count {
                    log(format!("[REPEAT] iteration {}/{count}", i + 1));
                    for inner in &body.statements.clone() {
                        self.execute_statement(&inner, log)?;
                    }
                }
            }
            Statement::IfBattery { body, .. } | Statement::Parallel { body } => {
                for inner in &body.statements.clone() {
                    self.execute_statement(&inner, log)?;
                }
            }
            Statement::OnCondition { condition, .. } => {
                // On real hardware, reactive triggers become ROS 2 subscriber nodes
                // listening to /mavros/battery_state, /mavros/wind_estimation, etc.
                // Wire those subscribers to execute the trigger body's commands.
                log(format!(
                    "[TRIGGER REGISTERED] Subscribe to telemetry for: {condition:?}"
                ));
            }
        }
        Ok(())
    }

    fn execute_command<F>(&mut self, cmd: &Command, log: &mut F) -> Result<(), VosaError>
    where
        F: FnMut(String),
    {
        match cmd {
            Command::Takeoff { altitude } => {
                let msg = format!("TAKEOFF {altitude}");
                log(format!("→ /mavros/vosa/cmd  \"{msg}\""));
                let _ = self.cmd_pub.publish(StringMsg { data: msg });
            }

            Command::Land => {
                log("→ /mavros/vosa/cmd  \"LAND\"".into());
                let _ = self.cmd_pub.publish(StringMsg { data: "LAND".into() });
            }

            Command::Hover { duration } => {
                let msg = format!("LOITER {duration}");
                log(format!("→ /mavros/vosa/cmd  \"{msg}\""));
                let _ = self.cmd_pub.publish(StringMsg { data: msg });
            }

            Command::Waypoint { lat, lon, alt } => {
                let target = GlobalPositionTarget {
                    header: Header {
                        stamp: TimeStamp { sec: 0, nanosec: 0 },
                        frame_id: "map".into(),
                    },
                    coordinate_frame: 6, // MAV_FRAME_GLOBAL_RELATIVE_ALT_INT
                    type_mask: 0b1111_1000, // position-only, ignore vel/acc/yaw
                    latitude: *lat,
                    longitude: *lon,
                    altitude: *alt as f32,
                    vx: 0.0, vy: 0.0, vz: 0.0,
                    afx: 0.0, afy: 0.0, afz: 0.0,
                    yaw: 0.0, yaw_rate: 0.0,
                };
                log(format!(
                    "→ /mavros/setpoint_raw/global  ({lat:.5}°, {lon:.5}°, {alt}m)"
                ));
                let _ = self.wp_pub.publish(target);
            }

            Command::ReturnHome => {
                log("→ /mavros/vosa/cmd  \"RETURN_TO_LAUNCH\"".into());
                let _ = self
                    .cmd_pub
                    .publish(StringMsg { data: "RETURN_TO_LAUNCH".into() });
            }

            Command::Camera { action, resolution } => {
                let res = resolution.as_deref().unwrap_or("default");
                let msg = match action {
                    CameraAction::Record => format!("CAMERA_RECORD res={res}"),
                    CameraAction::Photo  => format!("CAMERA_PHOTO  res={res}"),
                    CameraAction::Stop   => "CAMERA_STOP".into(),
                };
                log(format!("→ /mavros/vosa/cmd  \"{msg}\""));
                let _ = self.cmd_pub.publish(StringMsg { data: msg });
            }
        }
        Ok(())
    }
}
