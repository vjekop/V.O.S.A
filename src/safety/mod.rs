use crate::error::VosaError;
use crate::parser::ast::*;

/// The safety sandbox validates a parsed mission against its own safety
/// constraints before any execution is attempted.  All violations are
/// surfaced as `VosaError::SafetyViolation` **before** the drone moves.
pub struct SafetySandbox;

impl SafetySandbox {
    pub fn new() -> Self {
        SafetySandbox
    }

    /// Run every safety check.  Returns `Ok(())` only if the mission is clean.
    pub fn validate(&self, mission: &Mission) -> Result<(), VosaError> {
        let safety = match &mission.safety {
            Some(s) => s,
            None    => return Ok(()), // no constraints declared → nothing to check
        };

        for cmd in &mission.sequence.commands {
            self.check_command(cmd, safety)?;
        }
        Ok(())
    }

    fn check_command(&self, cmd: &Command, safety: &SafetyBlock) -> Result<(), VosaError> {
        match cmd {
            Command::Takeoff { altitude } => {
                self.check_altitude(*altitude, safety, "takeoff altitude")?;
            }
            Command::Waypoint { alt, lat, lon } => {
                self.check_altitude(*alt, safety, &format!("waypoint({lat}, {lon}) altitude"))?;
            }
            Command::Hover { duration } => {
                if *duration <= 0.0 {
                    return Err(VosaError::SafetyViolation(
                        "hover duration must be greater than 0 seconds".into(),
                    ));
                }
            }
            // land / return_home / camera have no altitude/speed to check here
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
}

impl Default for SafetySandbox {
    fn default() -> Self { Self::new() }
}
