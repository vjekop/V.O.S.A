use crate::error::VosaError;
use crate::parser::ast::*;

/// The result of executing a VOSA mission in the simulator.
#[derive(Debug)]
pub struct ExecutionReport {
    pub mission_name: String,
    pub steps: Vec<ExecutionStep>,
    pub total_distance_m: f64,
    pub max_altitude_m: f64,
}

/// A single executed step in the simulation log.
#[derive(Debug)]
pub struct ExecutionStep {
    pub index: usize,
    pub description: String,
}

/// The VOSA runtime simulates drone execution and produces an `ExecutionReport`.
/// (In a real deployment this layer would emit MAVLink / ROS2 messages.)
pub struct Runtime {
    steps: Vec<ExecutionStep>,
    current_lat: f64,
    current_lon: f64,
    current_alt: f64,
    total_distance: f64,
    max_alt: f64,
}

impl Runtime {
    pub fn new() -> Self {
        Runtime {
            steps: Vec::new(),
            current_lat: 0.0,
            current_lon: 0.0,
            current_alt: 0.0,
            total_distance: 0.0,
            max_alt: 0.0,
        }
    }

    fn log(&mut self, desc: impl Into<String>) {
        let idx = self.steps.len();
        self.steps.push(ExecutionStep {
            index: idx,
            description: desc.into(),
        });
    }

    pub fn execute(&mut self, mission: &Mission) -> Result<ExecutionReport, VosaError> {
        self.log(format!("[MISSION] Starting \"{}\"", mission.name));

        if let Some(v) = &mission.vehicle {
            self.log(format!("[VEHICLE] {:?}", v));
        }

        for cmd in &mission.sequence.commands {
            self.execute_command(cmd)?;
        }

        self.log("[MISSION] Complete.");

        Ok(ExecutionReport {
            mission_name: mission.name.clone(),
            steps: std::mem::take(&mut self.steps),
            total_distance_m: self.total_distance,
            max_altitude_m: self.max_alt,
        })
    }

    fn execute_command(&mut self, cmd: &Command) -> Result<(), VosaError> {
        match cmd {
            Command::Takeoff { altitude } => {
                self.current_alt = *altitude;
                self.max_alt = self.max_alt.max(*altitude);
                self.log(format!("[TAKEOFF] Ascending to {altitude}m"));
            }
            Command::Land => {
                self.log(format!(
                    "[LAND] Descending from {:.1}m and disarming",
                    self.current_alt
                ));
                self.current_alt = 0.0;
            }
            Command::Hover { duration } => {
                self.log(format!("[HOVER] Holding position for {duration}s"));
            }
            Command::Waypoint { lat, lon, alt } => {
                let dist = haversine(self.current_lat, self.current_lon, *lat, *lon);
                self.total_distance += dist;
                self.current_lat = *lat;
                self.current_lon = *lon;
                self.current_alt = *alt;
                self.max_alt = self.max_alt.max(*alt);
                self.log(format!(
                    "[WAYPOINT] → ({lat:.4}°, {lon:.4}°) at {alt}m  [{dist:.0}m traveled]"
                ));
            }
            Command::ReturnHome => {
                let dist = haversine(self.current_lat, self.current_lon, 0.0, 0.0);
                self.total_distance += dist;
                self.log(format!(
                    "[RTH] Returning to home  [{dist:.0}m]"
                ));
                self.current_lat = 0.0;
                self.current_lon = 0.0;
            }
            Command::Camera { action, resolution } => {
                let res = resolution.as_deref().unwrap_or("default");
                self.log(format!("[CAMERA] {:?} @ {res}", action));
            }
        }
        Ok(())
    }
}

impl Default for Runtime {
    fn default() -> Self { Self::new() }
}

/// Approximate distance between two GPS coordinates (metres) using haversine.
fn haversine(lat1: f64, lon1: f64, lat2: f64, lon2: f64) -> f64 {
    const R: f64 = 6_371_000.0; // Earth radius in metres
    let dlat = (lat2 - lat1).to_radians();
    let dlon = (lon2 - lon1).to_radians();
    let a = (dlat / 2.0).sin().powi(2)
        + lat1.to_radians().cos() * lat2.to_radians().cos() * (dlon / 2.0).sin().powi(2);
    let c = 2.0 * a.sqrt().atan2((1.0 - a).sqrt());
    R * c
}
