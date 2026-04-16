use crate::error::VosaError;
use crate::parser::ast::*;
use crate::runtime::ExecutionReport;

/// A hardware bridge that translates VOSA semantics to MAVLink packets.
pub mod ros2;

pub use ros2::Ros2Bridge;
pub struct MavlinkBridge {
    connection_string: String,
    // Provide a mocked ExecutionReport generator for now
    // until full bidirectional MAVLink ACKs are baked
}

impl MavlinkBridge {
    pub fn new(connection_string: &str) -> Self {
        MavlinkBridge {
            connection_string: connection_string.to_string(),
        }
    }

    pub fn execute(&mut self, mission: &Mission) -> Result<ExecutionReport, VosaError> {
        println!("[MAVLink API] Attempting to connect to: {}", self.connection_string);
        // let mut vehicle = mavlink::connect(&self.connection_string)
        //    .map_err(|e| VosaError::RuntimeError(format!("Failed to connect to MAVLink: {e}")))?;

        println!("[MAVLink API] Connection mocked. Translating mission AST to MAVLink frames...");

        // Create a dummy report to satisfy the API
        let mut report = ExecutionReport {
            mission_name: mission.name.clone(),
            steps: vec![],
            total_distance_m: 0.0,
            max_altitude_m: 0.0,
        };

        let mut step_idx = 0;
        let mut log_step = |desc: String| {
            report.steps.push(crate::runtime::ExecutionStep {
                index: step_idx,
                description: desc,
            });
            step_idx += 1;
        };

        for stmt in &mission.sequence.statements {
            self.execute_statement(stmt, &mut log_step)?;
        }

        println!("[MAVLink API] Execution completed successfully.");
        Ok(report)
    }

    fn execute_statement<F>(&self, stmt: &Statement, log: &mut F) -> Result<(), VosaError> 
    where
        F: FnMut(String)
    {
        match stmt {
            Statement::Command(cmd) => self.execute_command(cmd, log)?,
            Statement::Repeat { count, body } => {
                for _ in 0..*count {
                    for inner_stmt in &body.statements {
                        self.execute_statement(inner_stmt, log)?;
                    }
                }
            }
            Statement::IfBattery { body, .. } => {
                // Real battery would be pulled via MAV_SYS_STATUS packet
                // Assuming condition passes for the bridge mock
                for inner_stmt in &body.statements {
                    self.execute_statement(inner_stmt, log)?;
                }
            }
            Statement::Parallel { body } => {
                log("[MAV_CMD] [PARALLEL BLOCK START] Dispatching async commands...".into());
                for inner_stmt in &body.statements {
                    self.execute_statement(inner_stmt, log)?;
                }
                log("[MAV_CMD] [PARALLEL BLOCK END]".into());
            }
        }
        Ok(())
    }

    fn execute_command<F>(&self, cmd: &Command, log: &mut F) -> Result<(), VosaError> 
    where
        F: FnMut(String)
    {
        match cmd {
            Command::Takeoff { altitude } => {
                log(format!("-> MAV_CMD_NAV_TAKEOFF (param7 z = {altitude}m)"));
            }
            Command::Land => {
                log("-> MAV_CMD_NAV_LAND".into());
            }
            Command::Hover { duration } => {
                log(format!("-> MAV_CMD_NAV_DELAY (param1 = {duration}s)"));
            }
            Command::Waypoint { lat, lon, alt } => {
                log(format!("-> MAV_CMD_NAV_WAYPOINT (param5 lat={lat}, param6 lon={lon}, param7 z={alt}m)"));
            }
            Command::ReturnHome => {
                log("-> MAV_CMD_NAV_RETURN_TO_LAUNCH".into());
            }
            Command::Camera { action, .. } => {
                let mav_action = match action {
                    CameraAction::Record => "MAV_CMD_VIDEO_START_CAPTURE",
                    CameraAction::Photo => "MAV_CMD_IMAGE_START_CAPTURE",
                    CameraAction::Stop => "MAV_CMD_VIDEO_STOP_CAPTURE",
                };
                log(format!("-> {}", mav_action));
            }
        }
        Ok(())
    }
}
