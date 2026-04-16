use crate::error::VosaError;
use crate::parser::ast::*;

/// The safety sandbox validates a parsed mission against its own declared safety
/// constraints **before any execution begins** — i.e. before the motors spin.
///
/// All checks here are *static*: they operate on the AST alone, with no runtime
/// state. Dynamic checks (e.g. per-waypoint geofence boundary) happen in the
/// runtime layer, but anything that can be caught statically is caught here.
///
/// Current static checks:
/// - `Takeoff` altitude vs `max_altitude` / `min_altitude`
/// - `Waypoint` altitude vs `max_altitude` / `min_altitude`
/// - `Hover` duration must be > 0
/// - `flight.cruise_speed` vs `safety.max_speed`
pub struct SafetySandbox;

impl SafetySandbox {
    pub fn new() -> Self {
        SafetySandbox
    }

    /// Run every static safety check. Returns `Ok(())` only if the mission is clean.
    pub fn validate(&self, mission: &Mission) -> Result<(), VosaError> {
        let safety = match &mission.safety {
            Some(s) => s,
            // No safety block declared — nothing to validate against.
            None => return Ok(()),
        };

        // ── Speed check ───────────────────────────────────────────────────────
        // If the mission declares both a cruise_speed and a max_speed, the
        // cruise_speed must not exceed the limit. This catches mis-configured
        // missions at parse time rather than mid-flight.
        if let Some(max_spd) = safety.max_speed {
            if let Some(cruise) = mission.flight.as_ref().and_then(|f| f.cruise_speed) {
                if cruise > max_spd {
                    return Err(VosaError::SafetyViolation(format!(
                        "flight.cruise_speed of {cruise}m/s exceeds safety.max_speed of {max_spd}m/s"
                    )));
                }
            }
        }

        // ── Per-command checks ────────────────────────────────────────────────
        for cmd in &mission.sequence.commands {
            self.check_command(cmd, safety)?;
        }

        Ok(())
    }

    /// Validate a single command against the safety block.
    fn check_command(&self, cmd: &Command, safety: &SafetyBlock) -> Result<(), VosaError> {
        match cmd {
            Command::Takeoff { altitude } => {
                self.check_altitude(*altitude, safety, "takeoff altitude")?;
            }
            Command::Waypoint { lat, lon, alt } => {
                self.check_altitude(
                    *alt,
                    safety,
                    &format!("waypoint({lat:.4}°, {lon:.4}°) altitude"),
                )?;
            }
            Command::Hover { duration } => {
                if *duration <= 0.0 {
                    return Err(VosaError::SafetyViolation(
                        "hover duration must be greater than 0 seconds".into(),
                    ));
                }
            }
            // land / return_home / camera have no altitude or speed to check statically.
            _ => {}
        }
        Ok(())
    }

    /// Check an altitude value against the safety block's min/max limits.
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
}

impl Default for SafetySandbox {
    fn default() -> Self {
        Self::new()
    }
}

// ── Unit tests ────────────────────────────────────────────────────────────────

#[cfg(test)]
mod tests {
    use super::*;
    use crate::parser::ast::*;

    fn mission_with_safety(safety: SafetyBlock, commands: Vec<Command>) -> Mission {
        Mission {
            name: "test".into(),
            vehicle: None,
            safety: Some(safety),
            flight: None,
            sequence: Sequence { commands },
        }
    }

    fn mission_with_flight_and_safety(
        safety: SafetyBlock,
        flight: FlightConfig,
        commands: Vec<Command>,
    ) -> Mission {
        Mission {
            name: "test".into(),
            vehicle: None,
            safety: Some(safety),
            flight: Some(flight),
            sequence: Sequence { commands },
        }
    }

    #[test]
    fn takeoff_within_max_altitude_passes() {
        let mission = mission_with_safety(
            SafetyBlock { max_altitude: Some(100.0), ..Default::default() },
            vec![Command::Takeoff { altitude: 50.0 }],
        );
        assert!(SafetySandbox::new().validate(&mission).is_ok());
    }

    #[test]
    fn takeoff_exceeds_max_altitude_is_violation() {
        let mission = mission_with_safety(
            SafetyBlock { max_altitude: Some(30.0), ..Default::default() },
            vec![Command::Takeoff { altitude: 50.0 }],
        );
        let err = SafetySandbox::new().validate(&mission).unwrap_err();
        assert!(matches!(err, VosaError::SafetyViolation(_)));
        assert!(err.to_string().contains("max_altitude"));
    }

    #[test]
    fn waypoint_below_min_altitude_is_violation() {
        let mission = mission_with_safety(
            SafetyBlock { min_altitude: Some(10.0), ..Default::default() },
            vec![Command::Waypoint { lat: 0.0, lon: 0.0, alt: 5.0 }],
        );
        let err = SafetySandbox::new().validate(&mission).unwrap_err();
        assert!(matches!(err, VosaError::SafetyViolation(_)));
        assert!(err.to_string().contains("min_altitude"));
    }

    #[test]
    fn hover_zero_duration_is_violation() {
        let mission = mission_with_safety(
            SafetyBlock::default(),
            vec![Command::Hover { duration: 0.0 }],
        );
        let err = SafetySandbox::new().validate(&mission).unwrap_err();
        assert!(matches!(err, VosaError::SafetyViolation(_)));
    }

    #[test]
    fn cruise_speed_exceeds_max_speed_is_violation() {
        let mission = mission_with_flight_and_safety(
            SafetyBlock { max_speed: Some(10.0), ..Default::default() },
            FlightConfig { cruise_speed: Some(20.0), ..Default::default() },
            vec![Command::Takeoff { altitude: 10.0 }],
        );
        let err = SafetySandbox::new().validate(&mission).unwrap_err();
        assert!(matches!(err, VosaError::SafetyViolation(_)));
        assert!(err.to_string().contains("cruise_speed"));
    }

    #[test]
    fn cruise_speed_within_max_speed_passes() {
        let mission = mission_with_flight_and_safety(
            SafetyBlock { max_speed: Some(15.0), ..Default::default() },
            FlightConfig { cruise_speed: Some(8.0), ..Default::default() },
            vec![Command::Takeoff { altitude: 10.0 }],
        );
        assert!(SafetySandbox::new().validate(&mission).is_ok());
    }

    #[test]
    fn no_safety_block_always_passes() {
        let mission = Mission {
            name: "bare".into(),
            vehicle: None,
            safety: None,
            flight: None,
            sequence: Sequence {
                commands: vec![Command::Takeoff { altitude: 9999.0 }],
            },
        };
        assert!(SafetySandbox::new().validate(&mission).is_ok());
    }
}
