/// A physical unit of measurement used in VOSA.
#[derive(Debug, Clone, PartialEq)]
pub enum Unit {
    /// Meters (altitude, distance)
    Meters,
    /// Meters per second (speed/velocity)
    MetersPerSecond,
    /// Seconds (duration)
    Seconds,
    /// Percent (battery levels)
    Percent,
    /// Degrees (angles)
    Degrees,
}

/// A single token produced by the VOSA lexer.
#[derive(Debug, Clone, PartialEq)]
pub struct Token {
    pub kind: TokenKind,
    pub line: usize,
    pub col: usize,
}

impl Token {
    pub fn new(kind: TokenKind, line: usize, col: usize) -> Self {
        Token { kind, line, col }
    }
}

/// All token variants in the VOSA language.
#[derive(Debug, Clone, PartialEq)]
pub enum TokenKind {
    // ── Literals ─────────────────────────────────────────────────────────────
    /// A quoted string literal, e.g. `"survey_alpha"`
    StringLit(String),
    /// A bare number, e.g. `38.897`
    Number(f64),
    /// A number with a physical unit, e.g. `50m`, `10m/s`, `5s`, `20%`
    Quantity(f64, Unit),
    /// Boolean true / false
    Bool(bool),

    // ── Block keywords ────────────────────────────────────────────────────────
    Mission,
    Safety,
    Flight,
    Sequence,
    Vehicle,
    Repeat,
    If,
    Parallel,

    // ── Named values / labels ─────────────────────────────────────────────────
    /// `home` — the launch/home position
    Home,
    /// `circle(...)` geofence shape
    Circle,
    /// `radius` parameter name inside circle
    Radius,
    /// `center` parameter name inside circle
    Center,

    // ── Safety failsafe actions ───────────────────────────────────────────────
    ReturnHome,
    Land,
    Hover,

    // ── Command keywords ──────────────────────────────────────────────────────
    Takeoff,
    Waypoint,
    Camera,

    // ── Camera actions ────────────────────────────────────────────────────────
    Record,
    Photo,
    Stop,

    // ── Vehicle types ─────────────────────────────────────────────────────────
    Quadcopter,
    FixedWing,
    Hexacopter,

    // ── Safety config keys ────────────────────────────────────────────────────
    MaxAltitude,
    MinAltitude,
    MaxSpeed,
    Geofence,
    BatteryReserve,
    Failsafe,

    // ── Flight config keys ────────────────────────────────────────────────────
    CruiseAltitude,
    CruiseSpeed,

    // ── Command parameter keys ────────────────────────────────────────────────
    Lat,
    Lon,
    Alt,
    Altitude,
    Duration,
    Action,
    Resolution,
    /// `north` — relative northward offset in metres
    North,
    /// `east` — relative eastward offset in metres
    East,

    // ── State variables / trigger conditions ─────────────────────────────────
    /// `battery` — current battery level (%)
    Battery,
    /// `wind` — current wind speed (m/s)
    Wind,
    /// `obstacle_detected` — boolean sensor signal
    ObstacleDetected,

    // ── Reactive trigger keyword ──────────────────────────────────────────────
    /// `on` — introduces a reactive trigger block
    On,
    /// `and` — logical AND between two trigger conditions
    And,
    /// `or` — logical OR between two trigger conditions
    Or,

    // ── Sensor binding keywords ───────────────────────────────────────────────
    /// `sensor` — declares a named telemetry binding
    Sensor,
    /// `from` — separates the sensor name from its MAVLink source
    From,
    /// `for` — requires a condition to hold for a duration before firing
    For,

    // ── Generic identifier (for anything not a keyword) ───────────────────────
    Ident(String),

    // ── Punctuation ───────────────────────────────────────────────────────────
    LBrace,
    RBrace,
    LParen,
    RParen,
    Colon,
    Comma,
    Dot,
    LessThan,
    GreaterThan,

    // ── Sentinel ──────────────────────────────────────────────────────────────
    Eof,
}
