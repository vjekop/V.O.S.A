use crate::error::VosaError;
use crate::parser::ast::*;
use crate::runtime::ExecutionReport;
use ros2_client::{rustdds::QosPolicyBuilder, Context, Node, NodeOptions, Publisher, Name, NodeName, MessageTypeName, DEFAULT_PUBLISHER_QOS};
use serde::{Deserialize, Serialize};

// Standard ROS 2 Message Definitions
#[derive(Serialize, Deserialize, Clone)]
pub struct StringMsg {
    pub data: String,
}

#[derive(Serialize, Deserialize, Clone)]
pub struct Point {
    pub x: f64,
    pub y: f64,
    pub z: f64,
}

#[derive(Serialize, Deserialize, Clone)]
pub struct Quaternion {
    pub x: f64,
    pub y: f64,
    pub z: f64,
    pub w: f64,
}

#[derive(Serialize, Deserialize, Clone)]
pub struct Pose {
    pub position: Point,
    pub orientation: Quaternion,
}

#[derive(Serialize, Deserialize, Clone)]
pub struct Time {
    pub sec: i32,
    pub nanosec: u32,
}

#[derive(Serialize, Deserialize, Clone)]
pub struct Header {
    pub stamp: Time,
    pub frame_id: String,
}

#[derive(Serialize, Deserialize, Clone)]
pub struct PoseStamped {
    pub header: Header,
    pub pose: Pose,
}

pub struct Ros2Bridge {
    _node: Node,
    cmd_pub: Publisher<StringMsg>,
    wp_pub: Publisher<PoseStamped>,
}

impl Ros2Bridge {
    pub fn new(domain_id: u32) -> Result<Self, VosaError> {
        println!("[ROS 2] Initializing Node on Domain ID {}", domain_id);

        let context = Context::new()
            .map_err(|e| VosaError::RuntimeError(format!("Failed to create ROS 2 context: {:?}", e)))?;
            
        let mut node = context.new_node(
            NodeName::new("/vosa", "vosa_commander").map_err(|e| VosaError::RuntimeError(format!("{:?}", e)))?,
            NodeOptions::new().enable_rosout(true),
        ).map_err(|e| VosaError::RuntimeError(format!("Failed to create ROS 2 Node: {:?}", e)))?;

        let cmd_topic = node.create_topic(
            &Name::new("/vosa", "command").unwrap(),
            MessageTypeName::new("std_msgs", "String"),
            &DEFAULT_PUBLISHER_QOS
        ).map_err(|e| VosaError::RuntimeError(format!("Failed to create topic: {:?}", e)))?;
        
        let cmd_pub = node.create_publisher(&cmd_topic, None)
            .map_err(|e| VosaError::RuntimeError(format!("Failed to create publisher: {:?}", e)))?;

        let wp_topic = node.create_topic(
            &Name::new("/vosa", "waypoint").unwrap(),
            MessageTypeName::new("geometry_msgs", "PoseStamped"),
            &DEFAULT_PUBLISHER_QOS
        ).map_err(|e| VosaError::RuntimeError(format!("Failed to create topic: {:?}", e)))?;
        
        let wp_pub = node.create_publisher(&wp_topic, None)
            .map_err(|e| VosaError::RuntimeError(format!("Failed to create publisher: {:?}", e)))?;

        Ok(Ros2Bridge {
            _node: node,
            cmd_pub,
            wp_pub,
        })
    }

    pub fn execute(&mut self, mission: &Mission) -> Result<ExecutionReport, VosaError> {
        println!("[ROS 2 API] Translating mission AST to ROS 2 Topics...");

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

        println!("[ROS 2 API] Execution completed successfully.");
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
                for inner_stmt in &body.statements {
                    self.execute_statement(inner_stmt, log)?;
                }
            }
            Statement::Parallel { body } => {
                log("[ROS 2] [PARALLEL BLOCK START] Dispatching async topics...".into());
                for inner_stmt in &body.statements {
                    self.execute_statement(inner_stmt, log)?;
                }
                log("[ROS 2] [PARALLEL BLOCK END]".into());
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
                log(format!("Publishing /vosa/command -> TAKEOFF {altitude}m"));
                let _ = self.cmd_pub.publish(StringMsg { data: format!("TAKEOFF {}", altitude) });
            }
            Command::Land => {
                log("Publishing /vosa/command -> LAND".into());
                let _ = self.cmd_pub.publish(StringMsg { data: "LAND".to_string() });
            }
            Command::Hover { duration } => {
                log(format!("Publishing /vosa/command -> HOVER {duration}s"));
                let _ = self.cmd_pub.publish(StringMsg { data: format!("HOVER {}", duration) });
            }
            Command::Waypoint { lat, lon, alt } => {
                log(format!("Publishing /vosa/waypoint -> PoseStamped({lat}, {lon}, {alt}m)"));
                let msg = PoseStamped {
                    header: Header {
                        stamp: Time { sec: 0, nanosec: 0 },
                        frame_id: "map".to_string(),
                    },
                    pose: Pose {
                        position: Point { x: *lat, y: *lon, z: *alt },
                        orientation: Quaternion { x: 0.0, y: 0.0, z: 0.0, w: 1.0 },
                    }
                };
                let _ = self.wp_pub.publish(msg);
            }
            Command::ReturnHome => {
                log("Publishing /vosa/command -> RETURN_HOME".into());
                let _ = self.cmd_pub.publish(StringMsg { data: "RETURN_HOME".to_string() });
            }
            Command::Camera { action, .. } => {
                let action_str = match action {
                    CameraAction::Record => "CAMERA_RECORD",
                    CameraAction::Photo => "CAMERA_PHOTO",
                    CameraAction::Stop => "CAMERA_STOP",
                };
                log(format!("Publishing /vosa/command -> {action_str}"));
                let _ = self.cmd_pub.publish(StringMsg { data: action_str.to_string() });
            }
        }
        Ok(())
    }
}
