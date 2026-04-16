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

// ── Sequence & commands ───────────────────────────────────────────────────────

#[derive(Debug, Clone)]
pub struct Sequence {
    pub commands: Vec<Command>,
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
