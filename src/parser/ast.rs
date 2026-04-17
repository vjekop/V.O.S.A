/// The Abstract Syntax Tree for a V.O.S.A. program.
/// Every parsed `.vosa` file produces a single `Mission`.

// ── Top-level mission ────────────────────────────────────────────────────────

#[derive(Debug, Clone)]
pub struct Mission {
    /// The mission name string, e.g. `"perimeter_scan"`
    pub name: String,
    /// Optional vehicle declaration
    pub vehicle: Option<VehicleKind>,
    /// Safety block (constraints enforced before execution)
    pub safety: Option<SafetyBlock>,
    /// Flight configuration defaults
    pub flight: Option<FlightConfig>,
    /// The ordered list of commands to execute
    pub sequence: Sequence,
    /// User-declared sensor bindings — map a name to a MAVLink message field
    pub sensors: Vec<SensorBinding>,
}

// ── Sensor binding ────────────────────────────────────────────────────────────

/// Maps a user-chosen name to a specific MAVLink telemetry field.
///
/// Syntax: `sensor <name> from <MESSAGE>.<field>`
///
/// Example:
/// ```vosa
/// sensor roll_angle from ATTITUDE.roll
/// sensor gps_hdop   from GPS_RAW_INT.eph
/// ```
#[derive(Debug, Clone)]
pub struct SensorBinding {
    /// The name used to reference this sensor in trigger conditions
    pub name: String,
    /// The MAVLink message type (e.g. `ATTITUDE`, `VFR_HUD`)
    pub message: String,
    /// The field within that message (e.g. `roll`, `groundspeed`)
    pub field: String,
}

// ── Vehicle ──────────────────────────────────────────────────────────────────

#[derive(Debug, Clone, PartialEq)]
pub enum VehicleKind {
    Quadcopter,
    FixedWing,
    Hexacopter,
    Custom(String),
}

// ── Safety block ─────────────────────────────────────────────────────────────

#[derive(Debug, Clone, Default)]
pub struct SafetyBlock {
    /// Maximum allowed altitude in metres
    pub max_altitude: Option<f64>,
    /// Minimum allowed altitude in metres
    pub min_altitude: Option<f64>,
    /// Maximum allowed speed in m/s
    pub max_speed: Option<f64>,
    /// Geofence constraint
    pub geofence: Option<Geofence>,
    /// Minimum battery % before failsafe triggers
    pub battery_reserve: Option<f64>,
    /// What to do when a safety threshold is breached
    pub failsafe: Option<FailsafeAction>,
}

#[derive(Debug, Clone)]
pub enum Geofence {
    Circle {
        center: GeoCenter,
        /// Radius in metres
        radius: f64,
    },
}

#[derive(Debug, Clone)]
pub enum GeoCenter {
    /// Use the home / takeoff position as centre
    Home,
    /// A fixed coordinate
    Coord { lat: f64, lon: f64 },
}

#[derive(Debug, Clone, PartialEq)]
pub enum FailsafeAction {
    ReturnHome,
    Land,
    Hover,
}

// ── Flight config ─────────────────────────────────────────────────────────────

#[derive(Debug, Clone, Default)]
pub struct FlightConfig {
    /// Default cruise altitude in metres
    pub cruise_altitude: Option<f64>,
    /// Default cruise speed in m/s
    pub cruise_speed: Option<f64>,
}

// ── Sequence & statements ───────────────────────────────────────────────────────

#[derive(Debug, Clone)]
pub struct Sequence {
    pub statements: Vec<Statement>,
}

#[derive(Debug, Clone)]
pub enum Statement {
    /// A regular flight command
    Command(Command),
    /// Loop the inner sequence `count` times
    Repeat { count: usize, body: Sequence },
    /// Execute inner sequence if battery constraint falls true
    IfBattery {
        operator: Operator,
        threshold_percent: f64,
        body: Sequence,
    },
    /// Execute inner sequence concurrently
    Parallel { body: Sequence },
    /// Register a reactive trigger: body executes whenever `condition` becomes true
    OnCondition {
        condition: TriggerCondition,
        body: Sequence,
    },
}

#[derive(Debug, Clone, PartialEq)]
pub enum Operator {
    LessThan,
    GreaterThan,
}

/// A condition evaluated continuously during mission execution.
///
/// Triggers fire on the rising edge — i.e. when the condition transitions
/// from false to true. They reset automatically when the condition becomes
/// false again, so they can re-fire if the situation recurs.
///
/// Conditions can be combined:
///   `on battery < 30% and wind > 8m/s { ... }`
///   `on battery < 20% or obstacle_detected { ... }`
#[derive(Debug, Clone)]
pub enum TriggerCondition {
    /// Fires when battery % crosses the threshold in the specified direction
    Battery {
        operator: Operator,
        threshold_percent: f64,
    },
    /// Fires when wind speed (m/s) crosses the threshold
    Wind {
        operator: Operator,
        threshold_ms: f64,
    },
    /// Fires when an obstacle is detected by onboard sensors
    ObstacleDetected,
    /// Both conditions must be true simultaneously
    And(Box<TriggerCondition>, Box<TriggerCondition>),
    /// Either condition must be true
    Or(Box<TriggerCondition>, Box<TriggerCondition>),
    /// Fires when a user-declared sensor crosses a threshold.
    /// The sensor must be declared via `sensor <name> from MESSAGE.field`.
    Custom {
        name: String,
        operator: Operator,
        threshold: f64,
    },
}

#[derive(Debug, Clone)]
pub enum Command {
    /// Ascend to `altitude` metres
    Takeoff { altitude: f64 },
    /// Descend and disarm
    Land,
    /// Hold current position for `duration` seconds
    Hover { duration: f64 },
    /// Fly to GPS coordinate at a given altitude
    Waypoint { lat: f64, lon: f64, alt: f64 },
    /// Fly back to the home / launch position
    ReturnHome,
    /// Control the onboard camera
    Camera {
        action: CameraAction,
        resolution: Option<String>,
    },
}

#[derive(Debug, Clone, PartialEq)]
pub enum CameraAction {
    Record,
    Photo,
    Stop,
}
