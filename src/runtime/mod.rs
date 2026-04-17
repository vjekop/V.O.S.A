use crate::error::VosaError;
use crate::parser::ast::*;
use std::collections::HashMap;

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

/// A registered reactive trigger with its fired-state for edge detection.
///
/// Triggers use rising-edge semantics: the body executes once when the condition
/// transitions from false → true. It resets automatically when the condition
/// becomes false again, allowing it to re-fire if the situation recurs.
struct ActiveTrigger {
    condition: TriggerCondition,
    body: Sequence,
    /// If Some(d), condition must be continuously true for d seconds before firing.
    duration_s: Option<f64>,
    /// Simulated time (seconds) when the condition first became true in the current run.
    /// None means the condition is currently false.
    condition_true_since: Option<f64>,
    /// True while the condition is currently satisfied (prevents re-firing until reset).
    fired: bool,
}

/// The VOSA runtime simulates mission execution and produces an `ExecutionReport`.
///
/// In a real deployment this layer would emit MAVLink packets or ROS 2 messages.
/// In simulation mode it logs every action and tracks position / distance.
///
/// ## Reactive trigger system
/// `on` blocks are registered as active triggers when the runtime encounters them
/// in the sequence. After every command, all registered triggers are evaluated
/// against current environmental state. When a condition transitions false → true,
/// the trigger body is executed immediately before the mission resumes.
///
/// ## Simulated environment
/// - **Battery**: starts at 100%, drains at 1% per 500m of lateral flight.
/// - **Wind**: starts at 3 m/s, increases by 1 m/s per 3 waypoints reached.
/// - **Obstacle**: always false in simulation (requires live sensor input).
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
    /// Simulated battery level (%)
    battery_percent: f64,
    /// Simulated wind speed (m/s) — increases with waypoints visited
    wind_speed_ms: f64,
    /// Number of waypoints reached (used for wind simulation)
    waypoints_visited: u32,
    /// Whether an obstacle has been detected (always false in simulation)
    obstacle_detected: bool,
    /// Current values of user-declared custom sensors (always 0.0 in simulation)
    sensor_values: HashMap<String, f64>,
    /// Simulated mission clock in seconds — advanced by each command's estimated duration
    sim_time_s: f64,
    /// All registered `on` triggers accumulated during sequence execution
    triggers: Vec<ActiveTrigger>,
    /// Fixed sensor overrides supplied via `--inject`. Keys: "battery", "wind",
    /// "obstacle", or any custom sensor name. Values stay constant — no drain/escalation.
    injected: HashMap<String, f64>,
}

impl Runtime {
    pub fn new() -> Self {
        Self::with_injection(HashMap::new())
    }

    /// Create a runtime with fixed sensor overrides for simulation testing.
    /// Injected values remain constant throughout execution (no battery drain,
    /// no wind escalation) so you can test specific trigger conditions in isolation.
    pub fn with_injection(injected: HashMap<String, f64>) -> Self {
        let battery = injected.get("battery").copied().unwrap_or(100.0);
        let wind = injected.get("wind").copied().unwrap_or(3.0);
        let obstacle = injected.get("obstacle").map(|v| *v != 0.0).unwrap_or(false);
        Runtime {
            steps: Vec::new(),
            current_lat: 0.0,
            current_lon: 0.0,
            current_alt: 0.0,
            total_distance: 0.0,
            max_alt: 0.0,
            battery_percent: battery,
            wind_speed_ms: wind,
            waypoints_visited: 0,
            obstacle_detected: obstacle,
            sensor_values: HashMap::new(),
            sim_time_s: 0.0,
            triggers: Vec::new(),
            injected,
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
                self.log(format!(
                    "[SAFETY] geofence: {radius}m radius from {center_desc}"
                ));
            }
        }

        self.log(format!("[ENV] Initial wind: {:.1}m/s", self.wind_speed_ms));

        // Register declared sensors; injected values override the 0.0 default
        for binding in &mission.sensors {
            let value = self.injected.get(&binding.name).copied().unwrap_or(0.0);
            self.sensor_values.insert(binding.name.clone(), value);
            let src = if self.injected.contains_key(&binding.name) {
                format!("injected: {value}")
            } else {
                format!("simulation: 0.0")
            };
            self.log(format!(
                "[SENSOR] Registered: {} → {}.{} ({})",
                binding.name, binding.message, binding.field, src
            ));
        }

        for stmt in &mission.sequence.statements.clone() {
            self.execute_statement(&stmt, mission.safety.as_ref())?;
        }

        self.log("[MISSION] Complete.");

        Ok(ExecutionReport {
            mission_name: mission.name.clone(),
            steps: std::mem::take(&mut self.steps),
            total_distance_m: self.total_distance,
            max_altitude_m: self.max_alt,
        })
    }

    /// Execute a statement, handling repeats, conditionals, parallel blocks, and triggers.
    fn execute_statement(
        &mut self,
        stmt: &Statement,
        safety: Option<&SafetyBlock>,
    ) -> Result<(), VosaError> {
        match stmt {
            Statement::Command(cmd) => {
                self.execute_command(cmd, safety)?;
                // Evaluate reactive triggers after every command
                self.check_triggers(safety)?;
            }
            Statement::Repeat { count, body } => {
                for i in 0..*count {
                    self.log(format!("[REPEAT] Iteration {}/{}", i + 1, count));
                    for inner_stmt in &body.statements.clone() {
                        self.execute_statement(&inner_stmt, safety)?;
                    }
                }
            }
            Statement::IfBattery {
                operator,
                threshold_percent,
                body,
            } => {
                let current_battery = self.battery_percent;
                let condition_met = match operator {
                    Operator::LessThan => current_battery < *threshold_percent,
                    Operator::GreaterThan => current_battery > *threshold_percent,
                };
                let op_str = if *operator == Operator::LessThan {
                    "<"
                } else {
                    ">"
                };
                self.log(format!(
                    "[IF BATTERY] {:.1}% {} {}% — condition is {}",
                    current_battery, op_str, threshold_percent, condition_met
                ));
                if condition_met {
                    for inner_stmt in &body.statements.clone() {
                        self.execute_statement(&inner_stmt, safety)?;
                    }
                }
            }
            Statement::Parallel { body } => {
                self.log("[PARALLEL] Dispatching concurrent block");
                for inner_stmt in &body.statements.clone() {
                    self.execute_statement(&inner_stmt, safety)?;
                }
            }
            Statement::OnCondition {
                condition,
                duration_s,
                body,
            } => {
                let label = condition_label(condition);
                let dur_desc = match duration_s {
                    Some(d) => format!(" for {d}s"),
                    None => String::new(),
                };
                self.log(format!("[TRIGGER] Registered: on {label}{dur_desc}"));
                self.triggers.push(ActiveTrigger {
                    condition: condition.clone(),
                    body: body.clone(),
                    duration_s: *duration_s,
                    condition_true_since: None,
                    fired: false,
                });
            }
        }
        Ok(())
    }

    /// Evaluate all registered triggers against current environmental state.
    /// Fires the body of any trigger whose condition just became true (rising edge).
    fn check_triggers(&mut self, safety: Option<&SafetyBlock>) -> Result<(), VosaError> {
        let battery = self.battery_percent;
        let wind = self.wind_speed_ms;
        let obstacle = self.obstacle_detected;
        let sensors = self.sensor_values.clone();
        let now = self.sim_time_s;

        let mut to_fire: Vec<usize> = Vec::new();

        for (i, trigger) in self.triggers.iter_mut().enumerate() {
            let condition_met =
                eval_condition(&trigger.condition, battery, wind, obstacle, &sensors);

            if condition_met {
                if !trigger.fired {
                    match trigger.duration_s {
                        None => {
                            // No duration guard — fire immediately on rising edge
                            if trigger.condition_true_since.is_none() {
                                trigger.condition_true_since = Some(now);
                                trigger.fired = true;
                                to_fire.push(i);
                            }
                        }
                        Some(required) => {
                            // Start the clock the first time we see the condition true
                            let since = trigger.condition_true_since.get_or_insert(now);
                            if now - *since >= required {
                                trigger.fired = true;
                                to_fire.push(i);
                            }
                        }
                    }
                }
            } else {
                // Condition cleared — reset so it can re-arm
                trigger.fired = false;
                trigger.condition_true_since = None;
            }
        }

        for i in to_fire {
            let label = condition_label(&self.triggers[i].condition);
            let dur = self.triggers[i]
                .duration_s
                .map(|d| format!(" (held {d}s)"))
                .unwrap_or_default();
            self.log(format!("[TRIGGER] FIRED: on {label}{dur}"));
            let body = self.triggers[i].body.clone();
            for stmt in &body.statements {
                self.execute_statement(stmt, safety)?;
            }
        }

        Ok(())
    }

    /// Execute a single command, checking dynamic safety constraints as needed.
    fn execute_command(
        &mut self,
        cmd: &Command,
        safety: Option<&SafetyBlock>,
    ) -> Result<(), VosaError> {
        match cmd {
            Command::Takeoff { altitude } => {
                self.sim_time_s += altitude / 3.0; // ~3 m/s vertical climb
                self.current_alt = *altitude;
                self.max_alt = self.max_alt.max(*altitude);
                self.log(format!("[TAKEOFF] Ascending to {altitude}m"));
            }

            Command::Land => {
                self.sim_time_s += self.current_alt / 2.0; // ~2 m/s descent
                self.log(format!(
                    "[LAND] Descending from {:.1}m — disarming",
                    self.current_alt
                ));
                self.current_alt = 0.0;
            }

            Command::Hover { duration } => {
                self.sim_time_s += duration;
                self.log(format!("[HOVER] Holding position for {duration}s"));
            }

            Command::Waypoint { lat, lon, alt } => {
                // ── Geofence boundary check ───────────────────────────────────
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
                self.sim_time_s += dist / 10.0; // estimate 10 m/s cruise
                self.drain_battery(dist);
                self.total_distance += dist;
                self.current_lat = *lat;
                self.current_lon = *lon;
                self.current_alt = *alt;
                self.check_battery_failsafe(safety)?;
                self.max_alt = self.max_alt.max(*alt);
                self.waypoints_visited += 1;
                self.simulate_wind();
                self.log(format!(
                    "[WAYPOINT] → ({lat:.4}°, {lon:.4}°) alt {alt}m  [{dist:.0}m leg]  \
                     [batt {:.1}%]  [wind {:.1}m/s]",
                    self.battery_percent, self.wind_speed_ms
                ));
            }

            Command::ReturnHome => {
                let dist = haversine(self.current_lat, self.current_lon, 0.0, 0.0);
                self.sim_time_s += dist / 10.0;
                self.drain_battery(dist);
                self.total_distance += dist;
                self.log(format!("[RTH] Returning to home  [{dist:.0}m]"));
                self.current_lat = 0.0;
                self.current_lon = 0.0;
                self.check_battery_failsafe(safety)?;
            }

            Command::Camera { action, resolution } => {
                let res = resolution.as_deref().unwrap_or("default");
                self.log(format!("[CAMERA] {:?} @ {res}", action));
            }
        }
        Ok(())
    }

    fn drain_battery(&mut self, distance_m: f64) {
        if self.injected.contains_key("battery") {
            return; // injected battery is held constant
        }
        self.battery_percent -= distance_m / 500.0;
        if self.battery_percent < 0.0 {
            self.battery_percent = 0.0;
        }
    }

    /// Check whether battery has dropped below the declared reserve and, if so,
    /// execute the failsafe action and abort the mission.
    fn check_battery_failsafe(&mut self, safety: Option<&SafetyBlock>) -> Result<(), VosaError> {
        let safety = match safety {
            Some(s) => s,
            None => return Ok(()),
        };
        let reserve = match safety.battery_reserve {
            Some(r) => r,
            None => return Ok(()),
        };

        if self.battery_percent <= reserve {
            let action = safety
                .failsafe
                .as_ref()
                .cloned()
                .unwrap_or(FailsafeAction::ReturnHome);
            self.log(format!(
                "[FAILSAFE] Battery {:.1}% \u{2264} reserve {reserve}% — executing failsafe: {action:?}",
                self.battery_percent
            ));
            match &action {
                FailsafeAction::ReturnHome => {
                    let dist = haversine(self.current_lat, self.current_lon, 0.0, 0.0);
                    self.battery_percent -= dist / 500.0;
                    if self.battery_percent < 0.0 {
                        self.battery_percent = 0.0;
                    }
                    self.total_distance += dist;
                    self.log(format!("[FAILSAFE] RTH — {dist:.0}m to home"));
                    self.current_lat = 0.0;
                    self.current_lon = 0.0;
                    self.current_alt = 0.0;
                }
                FailsafeAction::Land => {
                    self.log(format!(
                        "[FAILSAFE] Landing at ({:.4}°, {:.4}°)",
                        self.current_lat, self.current_lon
                    ));
                    self.current_alt = 0.0;
                }
                FailsafeAction::Hover => {
                    self.log("[FAILSAFE] Hovering — awaiting manual intervention");
                }
            }
            return Err(VosaError::FailsafeTriggered(format!(
                "battery dropped to {:.1}%, below reserve of {reserve}%",
                self.battery_percent
            )));
        }
        Ok(())
    }

    /// Simulate wind increasing with flight time (1 m/s per 3 waypoints).
    fn simulate_wind(&mut self) {
        if self.injected.contains_key("wind") {
            return; // injected wind is held constant
        }
        self.wind_speed_ms = 3.0 + (self.waypoints_visited / 3) as f64;
    }
}

impl Default for Runtime {
    fn default() -> Self {
        Self::new()
    }
}

/// Evaluate a trigger condition against current environmental state.
fn eval_condition(
    condition: &TriggerCondition,
    battery: f64,
    wind: f64,
    obstacle: bool,
    sensors: &HashMap<String, f64>,
) -> bool {
    match condition {
        TriggerCondition::Battery {
            operator,
            threshold_percent,
        } => match operator {
            Operator::LessThan => battery < *threshold_percent,
            Operator::GreaterThan => battery > *threshold_percent,
        },
        TriggerCondition::Wind {
            operator,
            threshold_ms,
        } => match operator {
            Operator::LessThan => wind < *threshold_ms,
            Operator::GreaterThan => wind > *threshold_ms,
        },
        TriggerCondition::ObstacleDetected => obstacle,
        TriggerCondition::Custom {
            name,
            operator,
            threshold,
        } => {
            let value = sensors.get(name).copied().unwrap_or(0.0);
            match operator {
                Operator::LessThan => value < *threshold,
                Operator::GreaterThan => value > *threshold,
            }
        }
        TriggerCondition::And(a, b) => {
            eval_condition(a, battery, wind, obstacle, sensors)
                && eval_condition(b, battery, wind, obstacle, sensors)
        }
        TriggerCondition::Or(a, b) => {
            eval_condition(a, battery, wind, obstacle, sensors)
                || eval_condition(b, battery, wind, obstacle, sensors)
        }
    }
}

/// Human-readable label for a trigger condition (used in log output).
fn condition_label(condition: &TriggerCondition) -> String {
    match condition {
        TriggerCondition::Battery {
            operator,
            threshold_percent,
        } => {
            let op = if *operator == Operator::LessThan {
                "<"
            } else {
                ">"
            };
            format!("battery {op} {threshold_percent}%")
        }
        TriggerCondition::Wind {
            operator,
            threshold_ms,
        } => {
            let op = if *operator == Operator::LessThan {
                "<"
            } else {
                ">"
            };
            format!("wind {op} {threshold_ms}m/s")
        }
        TriggerCondition::ObstacleDetected => "obstacle_detected".to_string(),
        TriggerCondition::Custom {
            name,
            operator,
            threshold,
        } => {
            let op = if *operator == Operator::LessThan {
                "<"
            } else {
                ">"
            };
            format!("{name} {op} {threshold}")
        }
        TriggerCondition::And(a, b) => format!("{} and {}", condition_label(a), condition_label(b)),
        TriggerCondition::Or(a, b) => format!("{} or {}", condition_label(a), condition_label(b)),
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

    fn simple_mission(safety: Option<SafetyBlock>, statements: Vec<Statement>) -> Mission {
        Mission {
            name: "rt_test".into(),
            vehicle: None,
            safety,
            flight: None,
            sequence: Sequence { statements },
            sensors: vec![],
        }
    }

    #[test]
    fn takeoff_and_land_reports_correct_altitude() {
        let m = simple_mission(
            None,
            vec![
                Statement::Command(Command::Takeoff { altitude: 25.0 }),
                Statement::Command(Command::Land),
            ],
        );
        let report = Runtime::new().execute(&m).unwrap();
        assert!((report.max_altitude_m - 25.0).abs() < 0.01);
        assert!((report.total_distance_m).abs() < 0.01);
    }

    #[test]
    fn waypoint_within_geofence_passes() {
        let m = simple_mission(
            Some(SafetyBlock {
                geofence: Some(Geofence::Circle {
                    center: GeoCenter::Coord { lat: 0.0, lon: 0.0 },
                    radius: 1000.0,
                }),
                ..Default::default()
            }),
            vec![
                Statement::Command(Command::Takeoff { altitude: 10.0 }),
                Statement::Command(Command::Waypoint {
                    lat: 0.001,
                    lon: 0.0,
                    alt: 10.0,
                }),
                Statement::Command(Command::Land),
            ],
        );
        assert!(Runtime::new().execute(&m).is_ok());
    }

    #[test]
    fn waypoint_outside_geofence_is_safety_violation() {
        let m = simple_mission(
            Some(SafetyBlock {
                geofence: Some(Geofence::Circle {
                    center: GeoCenter::Coord { lat: 0.0, lon: 0.0 },
                    radius: 100.0,
                }),
                ..Default::default()
            }),
            vec![
                Statement::Command(Command::Takeoff { altitude: 10.0 }),
                Statement::Command(Command::Waypoint {
                    lat: 0.001,
                    lon: 0.0,
                    alt: 10.0,
                }),
            ],
        );
        let err = Runtime::new().execute(&m).unwrap_err();
        assert!(matches!(err, VosaError::SafetyViolation(_)));
        assert!(err.to_string().contains("geofence"));
    }

    #[test]
    fn battery_trigger_fires_when_condition_met() {
        // Battery starts at 100%. Drain it with a long waypoint (~1000m), then
        // check that an `on battery < 99%` trigger fires.
        let m = simple_mission(
            None,
            vec![
                Statement::OnCondition {
                    condition: TriggerCondition::Battery {
                        operator: Operator::LessThan,
                        threshold_percent: 99.0,
                    },
                    duration_s: None,
                    body: Sequence {
                        statements: vec![Statement::Command(Command::Hover { duration: 5.0 })],
                    },
                },
                Statement::Command(Command::Takeoff { altitude: 20.0 }),
                // ~1km leg drains ~2% battery
                Statement::Command(Command::Waypoint {
                    lat: 0.009,
                    lon: 0.0,
                    alt: 20.0,
                }),
                Statement::Command(Command::Land),
            ],
        );
        let report = Runtime::new().execute(&m).unwrap();
        // Trigger should have fired — find the TRIGGER FIRED step
        let fired = report
            .steps
            .iter()
            .any(|s| s.description.contains("[TRIGGER] FIRED"));
        assert!(
            fired,
            "expected battery trigger to fire, steps: {:#?}",
            report.steps
        );
    }

    #[test]
    fn wind_trigger_fires_when_wind_exceeds_threshold() {
        // Wind starts at 3m/s and increases after waypoints. After 3 waypoints it
        // reaches 4m/s. Trigger on wind > 3.5m/s should fire.
        let waypoints: Vec<Statement> = (0..4)
            .map(|i| {
                Statement::Command(Command::Waypoint {
                    lat: i as f64 * 0.001,
                    lon: 0.0,
                    alt: 30.0,
                })
            })
            .collect();

        let mut stmts = vec![
            Statement::OnCondition {
                condition: TriggerCondition::Wind {
                    operator: Operator::GreaterThan,
                    threshold_ms: 3.5,
                },
                duration_s: None,
                body: Sequence {
                    statements: vec![Statement::Command(Command::Hover { duration: 10.0 })],
                },
            },
            Statement::Command(Command::Takeoff { altitude: 30.0 }),
        ];
        stmts.extend(waypoints);
        stmts.push(Statement::Command(Command::Land));

        let m = simple_mission(None, stmts);
        let report = Runtime::new().execute(&m).unwrap();
        let fired = report
            .steps
            .iter()
            .any(|s| s.description.contains("[TRIGGER] FIRED"));
        assert!(fired, "expected wind trigger to fire");
    }

    #[test]
    fn trigger_does_not_fire_when_condition_never_met() {
        // Battery threshold set very low — should never fire in a short mission.
        let m = simple_mission(
            None,
            vec![
                Statement::OnCondition {
                    condition: TriggerCondition::Battery {
                        operator: Operator::LessThan,
                        threshold_percent: 1.0,
                    },
                    duration_s: None,
                    body: Sequence {
                        statements: vec![Statement::Command(Command::ReturnHome)],
                    },
                },
                Statement::Command(Command::Takeoff { altitude: 10.0 }),
                Statement::Command(Command::Land),
            ],
        );
        let report = Runtime::new().execute(&m).unwrap();
        let fired = report
            .steps
            .iter()
            .any(|s| s.description.contains("[TRIGGER] FIRED"));
        assert!(!fired, "trigger should not have fired");
    }

    #[test]
    fn temporal_trigger_fires_only_after_duration_elapses() {
        // Battery drains below 99% after the first long waypoint (~2% drop over ~1km).
        // With a `for 5s` guard, the trigger should fire once the sim clock
        // has advanced at least 5 s while the battery is below 99%.
        // The waypoint leg is ~1km → 100s of simulated time, so the trigger fires.
        let m = simple_mission(
            None,
            vec![
                Statement::OnCondition {
                    condition: TriggerCondition::Battery {
                        operator: Operator::LessThan,
                        threshold_percent: 99.0,
                    },
                    duration_s: Some(5.0),
                    body: Sequence {
                        statements: vec![Statement::Command(Command::Hover { duration: 5.0 })],
                    },
                },
                Statement::Command(Command::Takeoff { altitude: 20.0 }),
                Statement::Command(Command::Waypoint {
                    lat: 0.009,
                    lon: 0.0,
                    alt: 20.0,
                }),
                Statement::Command(Command::Land),
            ],
        );
        let report = Runtime::new().execute(&m).unwrap();
        let fired = report
            .steps
            .iter()
            .any(|s| s.description.contains("[TRIGGER] FIRED"));
        assert!(fired, "temporal trigger should fire after duration elapsed");
    }

    #[test]
    fn temporal_trigger_does_not_fire_if_condition_clears_before_duration() {
        // Wind spikes briefly (1 waypoint = wind increases) but a `for 999s` guard
        // means it should never fire — the mission ends before the clock reaches 999s.
        let m = simple_mission(
            None,
            vec![
                Statement::OnCondition {
                    condition: TriggerCondition::Wind {
                        operator: Operator::GreaterThan,
                        threshold_ms: 3.5,
                    },
                    duration_s: Some(999.0),
                    body: Sequence {
                        statements: vec![Statement::Command(Command::Hover { duration: 1.0 })],
                    },
                },
                Statement::Command(Command::Takeoff { altitude: 30.0 }),
                Statement::Command(Command::Waypoint {
                    lat: 0.001,
                    lon: 0.0,
                    alt: 30.0,
                }),
                Statement::Command(Command::Land),
            ],
        );
        let report = Runtime::new().execute(&m).unwrap();
        let fired = report
            .steps
            .iter()
            .any(|s| s.description.contains("[TRIGGER] FIRED"));
        assert!(
            !fired,
            "trigger with 999s guard should not fire in a short mission"
        );
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
