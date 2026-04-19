use crate::error::VosaError;
use crate::parser::ast::*;
use std::collections::HashSet;

/// The safety sandbox validates a parsed mission against its own declared safety
/// constraints **before any execution begins** — i.e. before the motors spin.
///
/// All checks here are *static*: they operate on the AST alone, with no runtime
/// state. Dynamic checks (e.g. live obstacle avoidance) happen in the runtime
/// layer, but anything that can be caught statically is caught here.
///
/// Current static checks:
/// - `Takeoff` altitude vs `max_altitude` / `min_altitude`
/// - `Waypoint` altitude vs `max_altitude` / `min_altitude`
/// - `Hover` duration must be > 0
/// - `flight.cruise_speed` vs `safety.max_speed`
/// - All waypoints must fall inside the declared geofence
/// - Estimated flight distance vs battery reserve (will it make it home?)
/// - Conflicting triggers (two triggers whose conditions can overlap and whose
///   bodies issue contradictory flight-mode commands)
pub struct SafetySandbox;

impl SafetySandbox {
    pub fn new() -> Self {
        SafetySandbox
    }

    /// Validate a single command against a safety block — used by the serve API
    /// to check dynamically generated waypoints from an autonomous explorer.
    pub fn validate_command(
        &self,
        cmd: &Command,
        safety: &Option<SafetyBlock>,
    ) -> Result<(), VosaError> {
        let Some(safety) = safety else { return Ok(()) };
        self.check_command(cmd, safety)?;
        // Geofence check for relative waypoints
        if let Command::WaypointRelative { north, east, .. } = cmd {
            if let Some(crate::parser::ast::Geofence::Circle { center, radius }) = &safety.geofence
            {
                let (clat, clon) = match center {
                    crate::parser::ast::GeoCenter::Home => (0.0_f64, 0.0_f64),
                    crate::parser::ast::GeoCenter::Coord { lat, lon } => (*lat, *lon),
                };
                let dist = (north * north + east * east).sqrt();
                let fence_dist = (clat * clat + clon * clon).sqrt() + dist;
                if fence_dist > *radius {
                    return Err(VosaError::SafetyViolation(format!(
                        "waypoint(north: {north}m, east: {east}m) is {dist:.0}m from home but geofence radius is only {radius:.0}m"
                    )));
                }
            }
        }
        Ok(())
    }

    /// Run every static safety check. Returns `Ok(())` only if the mission is clean.
    pub fn validate(&self, mission: &Mission) -> Result<(), VosaError> {
        let safety = match &mission.safety {
            Some(s) => s,
            None => return Ok(()),
        };

        // ── Speed check ───────────────────────────────────────────────────────
        if let Some(max_spd) = safety.max_speed {
            if let Some(cruise) = mission.flight.as_ref().and_then(|f| f.cruise_speed) {
                if cruise > max_spd {
                    return Err(VosaError::SafetyViolation(format!(
                        "flight.cruise_speed of {cruise}m/s exceeds safety.max_speed of {max_spd}m/s"
                    )));
                }
            }
        }

        // ── Per-command checks (altitude, hover duration) ─────────────────────
        for stmt in &mission.sequence.statements {
            self.check_statement(stmt, safety)?;
        }

        // ── Geofence vs waypoints ─────────────────────────────────────────────
        self.check_geofence(mission, safety)?;

        // ── Battery reserve vs estimated flight distance ──────────────────────
        self.check_battery_range(mission, safety)?;

        // ── Conflicting triggers ──────────────────────────────────────────────
        self.check_trigger_conflicts(mission)?;

        // ── Sensor binding validation ─────────────────────────────────────────
        self.check_sensor_references(mission)?;

        Ok(())
    }

    // ── Statement / command traversal ────────────────────────────────────────

    fn check_statement(&self, stmt: &Statement, safety: &SafetyBlock) -> Result<(), VosaError> {
        match stmt {
            Statement::Command(cmd) => self.check_command(cmd, safety)?,
            Statement::OnCondition {
                duration_s, body, ..
            } => {
                if let Some(d) = duration_s {
                    if *d <= 0.0 {
                        return Err(VosaError::SafetyViolation(
                            "trigger 'for' duration must be greater than 0 seconds".into(),
                        ));
                    }
                }
                for inner in &body.statements {
                    self.check_statement(inner, safety)?;
                }
            }
            Statement::Repeat { body, .. }
            | Statement::IfBattery { body, .. }
            | Statement::Parallel { body } => {
                for inner in &body.statements {
                    self.check_statement(inner, safety)?;
                }
            }
        }
        Ok(())
    }

    fn check_command(&self, cmd: &Command, safety: &SafetyBlock) -> Result<(), VosaError> {
        match cmd {
            Command::Takeoff { altitude } => {
                self.check_altitude(*altitude, safety, "takeoff altitude")?;
            }
            Command::Waypoint { lat, lon, alt } => {
                self.check_altitude(
                    *alt,
                    safety,
                    &format!("waypoint({lat:.4}, {lon:.4}) altitude"),
                )?;
            }
            Command::WaypointRelative { north, east, alt } => {
                self.check_altitude(
                    *alt,
                    safety,
                    &format!("waypoint(north: {north}m, east: {east}m) altitude"),
                )?;
            }
            Command::Hover { duration } if *duration <= 0.0 => {
                return Err(VosaError::SafetyViolation(
                    "hover duration must be greater than 0 seconds".into(),
                ));
            }
            _ => {}
        }
        Ok(())
    }

    fn check_altitude(&self, alt: f64, safety: &SafetyBlock, label: &str) -> Result<(), VosaError> {
        if let Some(max) = safety.max_altitude {
            if alt > max {
                return Err(VosaError::SafetyViolation(format!(
                    "{label} of {alt}m exceeds max_altitude of {max}m"
                )));
            }
        }
        if let Some(min) = safety.min_altitude {
            if alt < min {
                return Err(VosaError::SafetyViolation(format!(
                    "{label} of {alt}m is below min_altitude of {min}m"
                )));
            }
        }
        Ok(())
    }

    // ── Geofence check ────────────────────────────────────────────────────────
    //
    // For circle geofences, verify every waypoint (and takeoff) is within the
    // radius. We use the haversine formula for accurate great-circle distance.
    // Takeoff / home is at the geofence centre so it always passes.

    fn check_geofence(&self, mission: &Mission, safety: &SafetyBlock) -> Result<(), VosaError> {
        let geofence = match &safety.geofence {
            Some(g) => g,
            None => return Ok(()),
        };

        let Geofence::Circle { center, radius } = geofence;

        // We can only check fixed-center geofences statically.
        // Home-relative centres are validated at runtime when home is known.
        let (fence_lat, fence_lon) = match center {
            GeoCenter::Coord { lat, lon } => (*lat, *lon),
            GeoCenter::Home => return Ok(()), // runtime check
        };

        for stmt in &mission.sequence.statements {
            self.check_geofence_stmt(stmt, fence_lat, fence_lon, *radius)?;
        }
        Ok(())
    }

    fn check_geofence_stmt(
        &self,
        stmt: &Statement,
        clat: f64,
        clon: f64,
        radius: f64,
    ) -> Result<(), VosaError> {
        match stmt {
            Statement::Command(cmd) => {
                match cmd {
                    Command::Waypoint { lat, lon, .. } => {
                        let dist = haversine_m(clat, clon, *lat, *lon);
                        if dist > radius {
                            return Err(VosaError::SafetyViolation(format!(
                                "waypoint({lat:.5}, {lon:.5}) is {dist:.0}m from geofence center \
                                 but geofence radius is only {radius:.0}m"
                            )));
                        }
                    }
                    Command::WaypointRelative { north, east, .. } => {
                        // Euclidean distance from home — valid for radii up to ~50 km
                        let dist = (north * north + east * east).sqrt();
                        if dist > radius {
                            return Err(VosaError::SafetyViolation(format!(
                                "waypoint(north: {north}m, east: {east}m) is {dist:.0}m from home \
                                 but geofence radius is only {radius:.0}m"
                            )));
                        }
                    }
                    _ => {}
                }
            }
            Statement::Repeat { body, .. }
            | Statement::IfBattery { body, .. }
            | Statement::Parallel { body }
            | Statement::OnCondition { body, .. } => {
                for inner in &body.statements {
                    self.check_geofence_stmt(inner, clat, clon, radius)?;
                }
            }
        }
        Ok(())
    }

    // ── Battery range check ───────────────────────────────────────────────────
    //
    // Estimate total mission distance from waypoints, apply a conservative
    // energy model, and warn if the battery reserve may not cover the return leg.
    //
    // Model:
    //   - A quadcopter burns roughly 1% battery per 100 m of horizontal flight
    //     at cruise altitude (very conservative — real vehicles vary widely).
    //   - We add a 20% margin on top of the declared battery_reserve to account
    //     for wind and sensor inaccuracy.
    //   - If (estimated_consumption + reserve + 20% margin) > 100%, flag it.
    //
    // This is intentionally conservative. False positives are safer than
    // false negatives when lives or equipment are at stake.

    fn check_battery_range(
        &self,
        mission: &Mission,
        safety: &SafetyBlock,
    ) -> Result<(), VosaError> {
        let reserve = match safety.battery_reserve {
            Some(r) => r,
            None => return Ok(()),
        };

        // Collect waypoint sequence in order
        let mut waypoints: Vec<(f64, f64)> = Vec::new();
        collect_waypoints(&mission.sequence.statements, &mut waypoints);

        if waypoints.len() < 2 {
            return Ok(());
        }

        // Estimate total distance
        let mut total_m = 0.0_f64;
        for w in waypoints.windows(2) {
            total_m += haversine_m(w[0].0, w[0].1, w[1].0, w[1].1);
        }

        // Return leg: last waypoint back to first (home approximation)
        let first = waypoints[0];
        let last = waypoints[waypoints.len() - 1];
        let return_m = haversine_m(last.0, last.1, first.0, first.1);
        total_m += return_m;

        // 1% per 100 m = 0.01% per metre
        let estimated_pct = total_m * 0.01 / 100.0;
        let margin_pct = 20.0; // safety margin on top of reserve
        let required_pct = estimated_pct + reserve + margin_pct;

        if required_pct > 100.0 {
            return Err(VosaError::SafetyViolation(format!(
                "estimated mission requires ~{estimated_pct:.1}% battery but only \
                 {:.1}% is available after reserve ({reserve}%) and safety margin ({margin_pct}%); \
                 shorten the mission or increase battery capacity",
                100.0 - reserve - margin_pct
            )));
        }

        Ok(())
    }

    // ── Conflicting trigger check ─────────────────────────────────────────────
    //
    // Two triggers conflict when:
    //   1. Their conditions can *both be true simultaneously*, AND
    //   2. Their bodies issue *contradictory* flight-mode commands
    //      (e.g. one says return_home, another says land in place).
    //
    // We check this by collecting all OnCondition triggers and comparing pairs.

    fn check_trigger_conflicts(&self, mission: &Mission) -> Result<(), VosaError> {
        let triggers: Vec<(&TriggerCondition, &Sequence)> = mission
            .sequence
            .statements
            .iter()
            .filter_map(|s| match s {
                Statement::OnCondition {
                    condition, body, ..
                } => Some((condition, body)),
                _ => None,
            })
            .collect();

        for i in 0..triggers.len() {
            for j in (i + 1)..triggers.len() {
                let (cond_a, body_a) = triggers[i];
                let (cond_b, body_b) = triggers[j];

                if conditions_can_overlap(cond_a, cond_b) {
                    let intent_a = trigger_intent(body_a);
                    let intent_b = trigger_intent(body_b);
                    if intents_conflict(&intent_a, &intent_b) {
                        return Err(VosaError::SafetyViolation(format!(
                            "conflicting triggers: '{}' and '{}' can fire simultaneously \
                             but issue contradictory commands ({intent_a:?} vs {intent_b:?}); \
                             use 'and'/'or' to combine them into a single trigger",
                            condition_display(cond_a),
                            condition_display(cond_b),
                        )));
                    }
                }
            }
        }
        Ok(())
    }

    // ── Sensor binding validation ─────────────────────────────────────────────

    fn check_sensor_references(&self, mission: &Mission) -> Result<(), VosaError> {
        // Validate each binding points to a supported MAVLink field
        for binding in &mission.sensors {
            if !is_known_sensor(&binding.message, &binding.field) {
                return Err(VosaError::SafetyViolation(format!(
                    "sensor '{}': '{}.{}' is not a supported MAVLink field — \
                     check the VOSA sensor reference for supported sources",
                    binding.name, binding.message, binding.field
                )));
            }
        }

        // Validate all Custom trigger conditions reference a declared sensor
        let declared: HashSet<&str> = mission.sensors.iter().map(|s| s.name.as_str()).collect();
        for stmt in &mission.sequence.statements {
            self.check_sensor_refs_stmt(stmt, &declared)?;
        }
        Ok(())
    }

    fn check_sensor_refs_stmt(
        &self,
        stmt: &Statement,
        declared: &HashSet<&str>,
    ) -> Result<(), VosaError> {
        match stmt {
            Statement::OnCondition {
                condition, body, ..
            } => {
                self.check_sensor_refs_condition(condition, declared)?;
                for inner in &body.statements {
                    self.check_sensor_refs_stmt(inner, declared)?;
                }
            }
            Statement::Repeat { body, .. }
            | Statement::IfBattery { body, .. }
            | Statement::Parallel { body } => {
                for inner in &body.statements {
                    self.check_sensor_refs_stmt(inner, declared)?;
                }
            }
            Statement::Command(_) => {}
        }
        Ok(())
    }

    fn check_sensor_refs_condition(
        &self,
        condition: &TriggerCondition,
        declared: &HashSet<&str>,
    ) -> Result<(), VosaError> {
        match condition {
            TriggerCondition::Custom { name, .. } if !declared.contains(name.as_str()) => {
                return Err(VosaError::SafetyViolation(format!(
                    "trigger references undeclared sensor '{name}' — \
                     add 'sensor {name} from MESSAGE.field' to the mission body"
                )));
            }
            TriggerCondition::And(a, b) | TriggerCondition::Or(a, b) => {
                self.check_sensor_refs_condition(a, declared)?;
                self.check_sensor_refs_condition(b, declared)?;
            }
            _ => {}
        }
        Ok(())
    }
}

/// Returns true if `MESSAGE.field` is a supported MAVLink sensor source.
fn is_known_sensor(message: &str, field: &str) -> bool {
    matches!(
        (message, field),
        ("ATTITUDE", "roll")
            | ("ATTITUDE", "pitch")
            | ("ATTITUDE", "yaw")
            | ("ATTITUDE", "rollspeed")
            | ("ATTITUDE", "pitchspeed")
            | ("ATTITUDE", "yawspeed")
            | ("VFR_HUD", "airspeed")
            | ("VFR_HUD", "groundspeed")
            | ("VFR_HUD", "alt")
            | ("VFR_HUD", "climb")
            | ("WIND_COV", "wind_x")
            | ("WIND_COV", "wind_y")
            | ("GPS_RAW_INT", "eph")
            | ("GPS_RAW_INT", "epv")
            | ("GPS_RAW_INT", "satellites_visible")
            | ("SYS_STATUS", "battery_remaining")
            | ("DISTANCE_SENSOR", "current_distance")
    )
}

impl Default for SafetySandbox {
    fn default() -> Self {
        Self::new()
    }
}

// ── Geometry helpers ──────────────────────────────────────────────────────────

/// Haversine great-circle distance in metres between two GPS coordinates.
fn haversine_m(lat1: f64, lon1: f64, lat2: f64, lon2: f64) -> f64 {
    const R: f64 = 6_371_000.0; // Earth radius in metres
    let phi1 = lat1.to_radians();
    let phi2 = lat2.to_radians();
    let dphi = (lat2 - lat1).to_radians();
    let dlam = (lon2 - lon1).to_radians();
    let a = (dphi / 2.0).sin().powi(2) + phi1.cos() * phi2.cos() * (dlam / 2.0).sin().powi(2);
    2.0 * R * a.sqrt().asin()
}

// ── Battery range helpers ─────────────────────────────────────────────────────

fn collect_waypoints(stmts: &[Statement], out: &mut Vec<(f64, f64)>) {
    for stmt in stmts {
        match stmt {
            Statement::Command(Command::Waypoint { lat, lon, .. }) => {
                out.push((*lat, *lon));
            }
            Statement::Repeat { body, .. }
            | Statement::IfBattery { body, .. }
            | Statement::Parallel { body }
            | Statement::OnCondition { body, .. } => {
                collect_waypoints(&body.statements, out);
            }
            Statement::Command(_) => {}
        }
    }
}

// ── Trigger conflict helpers ──────────────────────────────────────────────────

/// High-level intent of a trigger body — what mode does it ultimately command?
#[derive(Debug, Clone, PartialEq)]
enum TriggerIntent {
    ReturnHome,
    Land,
    Hover,
    Other,
}

fn trigger_intent(body: &Sequence) -> TriggerIntent {
    for stmt in &body.statements {
        if let Statement::Command(cmd) = stmt {
            match cmd {
                Command::ReturnHome => return TriggerIntent::ReturnHome,
                Command::Land => return TriggerIntent::Land,
                Command::Hover { .. } => return TriggerIntent::Hover,
                _ => {}
            }
        }
    }
    TriggerIntent::Other
}

fn intents_conflict(a: &TriggerIntent, b: &TriggerIntent) -> bool {
    use TriggerIntent::*;
    // ReturnHome vs Land is a conflict: one flies home, the other lands in place.
    // Hover vs either abort command is a conflict too.
    matches!(
        (&a, &b),
        (ReturnHome, Land)
            | (Land, ReturnHome)
            | (Hover, ReturnHome)
            | (ReturnHome, Hover)
            | (Hover, Land)
            | (Land, Hover)
    )
}

/// Returns true if two conditions can possibly be true at the same time.
///
/// Tiered thresholds (e.g. `battery < 40%` warning + `battery < 20%` abort) are
/// intentional and fire independently via rising-edge semantics — they are NOT
/// treated as overlapping. Only identical thresholds with the same operator can
/// fire simultaneously and are flagged for conflict checking.
fn conditions_can_overlap(a: &TriggerCondition, b: &TriggerCondition) -> bool {
    match (a, b) {
        (
            TriggerCondition::Battery {
                operator: op_a,
                threshold_percent: t_a,
            },
            TriggerCondition::Battery {
                operator: op_b,
                threshold_percent: t_b,
            },
        ) => {
            // Only truly ambiguous when both thresholds are identical — different
            // thresholds are tiered (each fires once at its own crossing point).
            match (op_a, op_b) {
                (Operator::LessThan, Operator::LessThan)
                | (Operator::GreaterThan, Operator::GreaterThan) => {
                    (t_a - t_b).abs() < f64::EPSILON
                }
                _ => false,
            }
        }
        // obstacle_detected and custom sensors are independent of battery/wind —
        // only flag overlap when two triggers share the same sensor type.
        (TriggerCondition::ObstacleDetected, TriggerCondition::ObstacleDetected) => true,
        (TriggerCondition::ObstacleDetected, _) | (_, TriggerCondition::ObstacleDetected) => false,
        (TriggerCondition::Custom { name: a, .. }, TriggerCondition::Custom { name: b, .. }) => {
            a == b
        }
        (TriggerCondition::Custom { .. }, _) | (_, TriggerCondition::Custom { .. }) => false,
        // Two wind thresholds — same tiered logic as battery
        (
            TriggerCondition::Wind {
                operator: op_a,
                threshold_ms: t_a,
            },
            TriggerCondition::Wind {
                operator: op_b,
                threshold_ms: t_b,
            },
        ) => match (op_a, op_b) {
            (Operator::LessThan, Operator::LessThan)
            | (Operator::GreaterThan, Operator::GreaterThan) => (t_a - t_b).abs() < f64::EPSILON,
            _ => false,
        },
        // Battery and wind monitor independent physical quantities — different responses
        // to each are intentional (e.g. abort on low battery, hover on high wind).
        // Priority is handled at runtime by whichever trigger fires first.
        (TriggerCondition::Battery { .. }, TriggerCondition::Wind { .. })
        | (TriggerCondition::Wind { .. }, TriggerCondition::Battery { .. }) => false,
        // Compound conditions — conservatively assume they can overlap
        _ => true,
    }
}

fn condition_display(c: &TriggerCondition) -> String {
    match c {
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
        TriggerCondition::ObstacleDetected => "obstacle_detected".into(),
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
        TriggerCondition::And(a, b) => {
            format!("{} and {}", condition_display(a), condition_display(b))
        }
        TriggerCondition::Or(a, b) => {
            format!("{} or {}", condition_display(a), condition_display(b))
        }
    }
}

// ── Unit tests ────────────────────────────────────────────────────────────────

#[cfg(test)]
mod tests {
    use super::*;
    use crate::parser::ast::*;

    fn mission_with_safety(safety: SafetyBlock, statements: Vec<Statement>) -> Mission {
        Mission {
            name: "test".into(),
            vehicle: None,
            safety: Some(safety),
            flight: None,
            sequence: Sequence { statements },
            sensors: vec![],
        }
    }

    fn mission_with_flight_and_safety(
        safety: SafetyBlock,
        flight: FlightConfig,
        statements: Vec<Statement>,
    ) -> Mission {
        Mission {
            name: "test".into(),
            vehicle: None,
            safety: Some(safety),
            flight: Some(flight),
            sequence: Sequence { statements },
            sensors: vec![],
        }
    }

    // ── Existing tests ────────────────────────────────────────────────────────

    #[test]
    fn takeoff_within_max_altitude_passes() {
        let m = mission_with_safety(
            SafetyBlock {
                max_altitude: Some(100.0),
                ..Default::default()
            },
            vec![Statement::Command(Command::Takeoff { altitude: 50.0 })],
        );
        assert!(SafetySandbox::new().validate(&m).is_ok());
    }

    #[test]
    fn takeoff_exceeds_max_altitude_is_violation() {
        let m = mission_with_safety(
            SafetyBlock {
                max_altitude: Some(30.0),
                ..Default::default()
            },
            vec![Statement::Command(Command::Takeoff { altitude: 50.0 })],
        );
        let err = SafetySandbox::new().validate(&m).unwrap_err();
        assert!(matches!(err, VosaError::SafetyViolation(_)));
        assert!(err.to_string().contains("max_altitude"));
    }

    #[test]
    fn waypoint_below_min_altitude_is_violation() {
        let m = mission_with_safety(
            SafetyBlock {
                min_altitude: Some(10.0),
                ..Default::default()
            },
            vec![Statement::Command(Command::Waypoint {
                lat: 0.0,
                lon: 0.0,
                alt: 5.0,
            })],
        );
        let err = SafetySandbox::new().validate(&m).unwrap_err();
        assert!(matches!(err, VosaError::SafetyViolation(_)));
        assert!(err.to_string().contains("min_altitude"));
    }

    #[test]
    fn hover_zero_duration_is_violation() {
        let m = mission_with_safety(
            SafetyBlock::default(),
            vec![Statement::Command(Command::Hover { duration: 0.0 })],
        );
        assert!(matches!(
            SafetySandbox::new().validate(&m).unwrap_err(),
            VosaError::SafetyViolation(_)
        ));
    }

    #[test]
    fn cruise_speed_exceeds_max_speed_is_violation() {
        let m = mission_with_flight_and_safety(
            SafetyBlock {
                max_speed: Some(10.0),
                ..Default::default()
            },
            FlightConfig {
                cruise_speed: Some(20.0),
                ..Default::default()
            },
            vec![Statement::Command(Command::Takeoff { altitude: 10.0 })],
        );
        let err = SafetySandbox::new().validate(&m).unwrap_err();
        assert!(matches!(err, VosaError::SafetyViolation(_)));
        assert!(err.to_string().contains("cruise_speed"));
    }

    #[test]
    fn no_safety_block_always_passes() {
        let m = Mission {
            name: "bare".into(),
            vehicle: None,
            safety: None,
            flight: None,
            sequence: Sequence {
                statements: vec![Statement::Command(Command::Takeoff { altitude: 9999.0 })],
            },
            sensors: vec![],
        };
        assert!(SafetySandbox::new().validate(&m).is_ok());
    }

    #[test]
    fn nested_repeat_violating_safety_fails() {
        let m = mission_with_safety(
            SafetyBlock {
                max_altitude: Some(10.0),
                ..Default::default()
            },
            vec![Statement::Repeat {
                count: 5,
                body: Sequence {
                    statements: vec![Statement::Command(Command::Takeoff { altitude: 20.0 })],
                },
            }],
        );
        assert!(matches!(
            SafetySandbox::new().validate(&m).unwrap_err(),
            VosaError::SafetyViolation(_)
        ));
    }

    // ── Geofence tests ────────────────────────────────────────────────────────

    #[test]
    fn waypoint_inside_geofence_passes() {
        let m = mission_with_safety(
            SafetyBlock {
                geofence: Some(Geofence::Circle {
                    center: GeoCenter::Coord {
                        lat: 47.398,
                        lon: 8.546,
                    },
                    radius: 500.0,
                }),
                ..Default::default()
            },
            vec![Statement::Command(Command::Waypoint {
                lat: 47.399,
                lon: 8.546,
                alt: 30.0,
            })],
        );
        assert!(SafetySandbox::new().validate(&m).is_ok());
    }

    #[test]
    fn waypoint_outside_geofence_is_violation() {
        let m = mission_with_safety(
            SafetyBlock {
                geofence: Some(Geofence::Circle {
                    center: GeoCenter::Coord {
                        lat: 47.398,
                        lon: 8.546,
                    },
                    radius: 100.0,
                }),
                ..Default::default()
            },
            // 5 km away — clearly outside
            vec![Statement::Command(Command::Waypoint {
                lat: 47.440,
                lon: 8.546,
                alt: 30.0,
            })],
        );
        let err = SafetySandbox::new().validate(&m).unwrap_err();
        assert!(matches!(err, VosaError::SafetyViolation(_)));
        assert!(err.to_string().contains("geofence"));
    }

    #[test]
    fn home_relative_geofence_skips_static_check() {
        // Can't validate home-relative geofence statically — should pass
        let m = mission_with_safety(
            SafetyBlock {
                geofence: Some(Geofence::Circle {
                    center: GeoCenter::Home,
                    radius: 100.0,
                }),
                ..Default::default()
            },
            vec![Statement::Command(Command::Waypoint {
                lat: 99.0,
                lon: 99.0,
                alt: 30.0,
            })],
        );
        assert!(SafetySandbox::new().validate(&m).is_ok());
    }

    // ── Battery range tests ───────────────────────────────────────────────────

    #[test]
    fn short_mission_within_battery_passes() {
        // 5 waypoints ~50 m apart — trivially within range
        let m = mission_with_safety(
            SafetyBlock {
                battery_reserve: Some(10.0),
                ..Default::default()
            },
            vec![
                Statement::Command(Command::Waypoint {
                    lat: 47.3980,
                    lon: 8.5462,
                    alt: 30.0,
                }),
                Statement::Command(Command::Waypoint {
                    lat: 47.3984,
                    lon: 8.5462,
                    alt: 30.0,
                }),
                Statement::Command(Command::Waypoint {
                    lat: 47.3988,
                    lon: 8.5462,
                    alt: 30.0,
                }),
            ],
        );
        assert!(SafetySandbox::new().validate(&m).is_ok());
    }

    #[test]
    fn extremely_long_mission_exceeds_battery() {
        // Waypoints 1000 km apart — impossible to complete
        let m = mission_with_safety(
            SafetyBlock {
                battery_reserve: Some(10.0),
                ..Default::default()
            },
            vec![
                Statement::Command(Command::Waypoint {
                    lat: 0.0,
                    lon: 0.0,
                    alt: 30.0,
                }),
                Statement::Command(Command::Waypoint {
                    lat: 10.0,
                    lon: 0.0,
                    alt: 30.0,
                }), // ~1111 km
                Statement::Command(Command::Waypoint {
                    lat: 10.0,
                    lon: 10.0,
                    alt: 30.0,
                }), // ~1111 km
            ],
        );
        let err = SafetySandbox::new().validate(&m).unwrap_err();
        assert!(matches!(err, VosaError::SafetyViolation(_)));
        assert!(err.to_string().contains("battery"));
    }

    // ── Trigger conflict tests ────────────────────────────────────────────────

    #[test]
    fn non_conflicting_triggers_pass() {
        // Two battery triggers with the same intent (both RTL) — no conflict
        let m = mission_with_safety(
            SafetyBlock::default(),
            vec![
                Statement::OnCondition {
                    condition: TriggerCondition::Battery {
                        operator: Operator::LessThan,
                        threshold_percent: 20.0,
                    },
                    duration_s: None,
                    body: Sequence {
                        statements: vec![Statement::Command(Command::ReturnHome)],
                    },
                },
                Statement::OnCondition {
                    condition: TriggerCondition::Battery {
                        operator: Operator::LessThan,
                        threshold_percent: 40.0,
                    },
                    duration_s: None,
                    body: Sequence {
                        statements: vec![Statement::Command(Command::ReturnHome)],
                    },
                },
            ],
        );
        assert!(SafetySandbox::new().validate(&m).is_ok());
    }

    #[test]
    fn conflicting_triggers_rtl_vs_land_is_violation() {
        // Two obstacle_detected triggers with contradictory actions — genuine conflict
        // (same sensor, same threshold, different commands).
        let m = mission_with_safety(
            SafetyBlock::default(),
            vec![
                Statement::OnCondition {
                    condition: TriggerCondition::ObstacleDetected,
                    duration_s: None,
                    body: Sequence {
                        statements: vec![Statement::Command(Command::ReturnHome)],
                    },
                },
                Statement::OnCondition {
                    condition: TriggerCondition::ObstacleDetected,
                    duration_s: None,
                    body: Sequence {
                        statements: vec![Statement::Command(Command::Land)],
                    },
                },
            ],
        );
        let err = SafetySandbox::new().validate(&m).unwrap_err();
        assert!(matches!(err, VosaError::SafetyViolation(_)));
        assert!(err.to_string().contains("conflicting triggers"));
    }
}
