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

/// A single logged step in the execution simulation.
#[derive(Debug)]
pub struct ExecutionStep {
    pub index: usize,
    pub description: String,
}

/// The VOSA runtime simulates mission execution and produces an `ExecutionReport`.
///
/// In a real deployment this layer would emit MAVLink packets or ROS 2 messages.
/// In simulation mode it logs every action and tracks position / distance.
///
/// ## Dynamic safety checks performed here
/// - **Geofence boundary**: each `Waypoint` is checked against the declared
///   geofence circle. If the waypoint is outside the radius, execution is aborted
///   with a `SafetyViolation` error before that command is executed.
///
/// Static checks (altitude limits, speed limits) are handled by `SafetySandbox`
/// before `Runtime::execute` is ever called.
pub struct Runtime {
    steps: Vec<ExecutionStep>,
    /// Current simulated position
    current_lat: f64,
    current_lon: f64,
    current_alt: f64,
    /// Cumulative distance flown (metres)
    total_distance: f64,
    /// Highest altitude reached (metres)
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

    /// Execute the full mission and return an `ExecutionReport`.
    pub fn execute(&mut self, mission: &Mission) -> Result<ExecutionReport, VosaError> {
        self.log(format!("[MISSION] Starting \"{}\"", mission.name));

        if let Some(v) = &mission.vehicle {
            self.log(format!("[VEHICLE] {:?}", v));
        }

        // Log active safety constraints for transparency
        if let Some(s) = &mission.safety {
            if let Some(max) = s.max_altitude {
                self.log(format!("[SAFETY] max_altitude: {max}m"));
            }
            if let Some(spd) = s.max_speed {
                self.log(format!("[SAFETY] max_speed: {spd}m/s"));
            }
            if let Some(Geofence::Circle { center, radius }) = &s.geofence {
                let center_desc = match center {
                    GeoCenter::Home => "home (0.0, 0.0)".to_string(),
                    GeoCenter::Coord { lat, lon } => format!("({lat:.4}°, {lon:.4}°)"),
                };
                self.log(format!("[SAFETY] geofence: {radius}m radius from {center_desc}"));
            }
        }

        for cmd in &mission.sequence.commands {
            self.execute_command(cmd, mission.safety.as_ref())?;
        }

        self.log("[MISSION] Complete.");

        Ok(ExecutionReport {
            mission_name: mission.name.clone(),
            steps: std::mem::take(&mut self.steps),
            total_distance_m: self.total_distance,
            max_altitude_m: self.max_alt,
        })
    }

    /// Execute a single command, checking dynamic safety constraints as needed.
    fn execute_command(
        &mut self,
        cmd: &Command,
        safety: Option<&SafetyBlock>,
    ) -> Result<(), VosaError> {
        match cmd {
            Command::Takeoff { altitude } => {
                self.current_alt = *altitude;
                self.max_alt = self.max_alt.max(*altitude);
                self.log(format!("[TAKEOFF] Ascending to {altitude}m"));
            }

            Command::Land => {
                self.log(format!(
                    "[LAND] Descending from {:.1}m — disarming",
                    self.current_alt
                ));
                self.current_alt = 0.0;
            }

            Command::Hover { duration } => {
                self.log(format!("[HOVER] Holding position for {duration}s"));
            }

            Command::Waypoint { lat, lon, alt } => {
                // ── Geofence boundary check ───────────────────────────────────
                // Before moving to this waypoint, verify it lies within the
                // declared geofence. Violation aborts the mission immediately.
                if let Some(safety) = safety {
                    if let Some(Geofence::Circle { center, radius }) = &safety.geofence {
                        let (fence_lat, fence_lon) = match center {
                            GeoCenter::Home => (0.0_f64, 0.0_f64),
                            GeoCenter::Coord { lat, lon } => (*lat, *lon),
                        };
                        let dist = haversine(fence_lat, fence_lon, *lat, *lon);
                        if dist > *radius {
                            return Err(VosaError::SafetyViolation(format!(
                                "waypoint ({lat:.4}°, {lon:.4}°) is {dist:.0}m from geofence center — exceeds radius of {radius}m"
                            )));
                        }
                    }
                }

                // ── Move to waypoint ──────────────────────────────────────────
                let dist = haversine(self.current_lat, self.current_lon, *lat, *lon);
                self.total_distance += dist;
                self.current_lat = *lat;
                self.current_lon = *lon;
                self.current_alt = *alt;
                self.max_alt = self.max_alt.max(*alt);
                self.log(format!(
                    "[WAYPOINT] → ({lat:.4}°, {lon:.4}°) alt {alt}m  [{dist:.0}m leg]"
                ));
            }

            Command::ReturnHome => {
                let dist = haversine(self.current_lat, self.current_lon, 0.0, 0.0);
                self.total_distance += dist;
                self.log(format!("[RTH] Returning to home  [{dist:.0}m]"));
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
    fn default() -> Self {
        Self::new()
    }
}

/// Haversine great-circle distance between two GPS coordinates (metres).
/// Uses Earth mean radius R = 6,371,000 m.
pub fn haversine(lat1: f64, lon1: f64, lat2: f64, lon2: f64) -> f64 {
    const R: f64 = 6_371_000.0;
    let dlat = (lat2 - lat1).to_radians();
    let dlon = (lon2 - lon1).to_radians();
    let a = (dlat / 2.0).sin().powi(2)
        + lat1.to_radians().cos() * lat2.to_radians().cos() * (dlon / 2.0).sin().powi(2);
    let c = 2.0 * a.sqrt().atan2((1.0 - a).sqrt());
    R * c
}

// ── Unit tests ────────────────────────────────────────────────────────────────

#[cfg(test)]
mod tests {
    use super::*;
    use crate::parser::ast::*;

    fn simple_mission(safety: Option<SafetyBlock>, commands: Vec<Command>) -> Mission {
        Mission {
            name: "rt_test".into(),
            vehicle: None,
            safety,
            flight: None,
            sequence: Sequence { commands },
        }
    }

    #[test]
    fn takeoff_and_land_reports_correct_altitude() {
        let m = simple_mission(
            None,
            vec![
                Command::Takeoff { altitude: 25.0 },
                Command::Land,
            ],
        );
        let report = Runtime::new().execute(&m).unwrap();
        assert!((report.max_altitude_m - 25.0).abs() < 0.01);
        assert!((report.total_distance_m).abs() < 0.01); // no lateral movement
    }

    #[test]
    fn waypoint_within_geofence_passes() {
        // Fence: 1000m radius from (0.0, 0.0)
        // Waypoint: ~111m north — well within fence
        let m = simple_mission(
            Some(SafetyBlock {
                geofence: Some(Geofence::Circle {
                    center: GeoCenter::Coord { lat: 0.0, lon: 0.0 },
                    radius: 1000.0,
                }),
                ..Default::default()
            }),
            vec![
                Command::Takeoff { altitude: 10.0 },
                Command::Waypoint { lat: 0.001, lon: 0.0, alt: 10.0 },
                Command::Land,
            ],
        );
        assert!(Runtime::new().execute(&m).is_ok());
    }

    #[test]
    fn waypoint_outside_geofence_is_safety_violation() {
        // Fence: 100m radius from (0.0, 0.0)
        // Waypoint: ~111m north — just outside fence
        let m = simple_mission(
            Some(SafetyBlock {
                geofence: Some(Geofence::Circle {
                    center: GeoCenter::Coord { lat: 0.0, lon: 0.0 },
                    radius: 100.0,
                }),
                ..Default::default()
            }),
            vec![
                Command::Takeoff { altitude: 10.0 },
                Command::Waypoint { lat: 0.001, lon: 0.0, alt: 10.0 },
            ],
        );
        let err = Runtime::new().execute(&m).unwrap_err();
        assert!(matches!(err, VosaError::SafetyViolation(_)));
        assert!(err.to_string().contains("geofence"));
    }

    #[test]
    fn haversine_same_point_is_zero() {
        assert!(haversine(0.0, 0.0, 0.0, 0.0).abs() < 0.001);
    }

    #[test]
    fn haversine_one_degree_latitude_is_approx_111km() {
        let d = haversine(0.0, 0.0, 1.0, 0.0);
        assert!((d - 111_195.0).abs() < 200.0, "got {d}");
    }
}
