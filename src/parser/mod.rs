pub mod ast;
pub use ast::*;

use crate::error::VosaError;
use crate::lexer::token::{Token, TokenKind, Unit};

/// Recursive-descent parser for the VOSA language.
pub struct Parser {
    tokens: Vec<Token>,
    pos: usize,
}

impl Parser {
    pub fn new(tokens: Vec<Token>) -> Self {
        Parser { tokens, pos: 0 }
    }

    // ── Helpers ───────────────────────────────────────────────────────────────

    fn peek(&self) -> &TokenKind {
        &self.tokens[self.pos].kind
    }

    fn current_line(&self) -> usize {
        self.tokens[self.pos].line
    }

    fn advance(&mut self) -> &Token {
        let t = &self.tokens[self.pos];
        if t.kind != TokenKind::Eof {
            self.pos += 1;
        }
        t
    }

    fn parse_err(&self, msg: impl Into<String>) -> VosaError {
        VosaError::ParseError {
            line: self.current_line(),
            msg: msg.into(),
        }
    }

    fn expect(&mut self, expected: &TokenKind) -> Result<(), VosaError> {
        if self.peek() == expected {
            self.advance();
            Ok(())
        } else {
            Err(self.parse_err(format!("expected {:?}, got {:?}", expected, self.peek())))
        }
    }

    /// Consume the next token and return it as a clone.
    fn consume(&mut self) -> TokenKind {
        self.advance().kind.clone()
    }

    // ── Quantity helpers ──────────────────────────────────────────────────────

    /// Accept a bare Number or a Quantity (any unit), returning the value.
    fn expect_number(&mut self) -> Result<f64, VosaError> {
        match self.consume() {
            TokenKind::Number(v) => Ok(v),
            TokenKind::Quantity(v, _) => Ok(v),
            other => Err(self.parse_err(format!("expected a number, got {:?}", other))),
        }
    }

    // ── Top-level ─────────────────────────────────────────────────────────────

    /// Parse a complete VOSA source file → `Mission`.
    pub fn parse(mut self) -> Result<Mission, VosaError> {
        self.expect(&TokenKind::Mission)?;
        let name = match self.consume() {
            TokenKind::StringLit(s) => s,
            other => {
                return Err(self.parse_err(format!("expected mission name string, got {:?}", other)))
            }
        };

        self.expect(&TokenKind::LBrace)?;

        let mut vehicle: Option<VehicleKind> = None;
        let mut safety: Option<SafetyBlock> = None;
        let mut flight: Option<FlightConfig> = None;
        let mut sequence: Option<Sequence> = None;
        let mut sensors: Vec<SensorBinding> = Vec::new();

        loop {
            match self.peek().clone() {
                TokenKind::RBrace => {
                    self.advance();
                    break;
                }
                TokenKind::Eof => return Err(self.parse_err("unexpected EOF inside mission block")),
                TokenKind::Vehicle => {
                    self.advance();
                    vehicle = Some(self.parse_vehicle()?);
                }
                TokenKind::Safety => {
                    self.advance();
                    safety = Some(self.parse_safety()?);
                }
                TokenKind::Flight => {
                    self.advance();
                    flight = Some(self.parse_flight()?);
                }
                TokenKind::Sequence => {
                    self.advance();
                    sequence = Some(self.parse_sequence()?);
                }
                TokenKind::Sensor => {
                    self.advance();
                    sensors.push(self.parse_sensor_binding()?);
                }
                other => {
                    return Err(
                        self.parse_err(format!("unexpected token in mission body: {:?}", other))
                    )
                }
            }
        }

        self.expect(&TokenKind::Eof)?;

        Ok(Mission {
            name,
            vehicle,
            safety,
            flight,
            sequence: sequence
                .ok_or_else(|| self.parse_err("mission must have a sequence block"))?,
            sensors,
        })
    }

    // ── vehicle ───────────────────────────────────────────────────────────────

    fn parse_vehicle(&mut self) -> Result<VehicleKind, VosaError> {
        self.expect(&TokenKind::Colon)?;
        Ok(match self.consume() {
            TokenKind::Quadcopter => VehicleKind::Quadcopter,
            TokenKind::FixedWing => VehicleKind::FixedWing,
            TokenKind::Hexacopter => VehicleKind::Hexacopter,
            TokenKind::Ident(s) => VehicleKind::Custom(s),
            other => return Err(self.parse_err(format!("expected vehicle kind, got {:?}", other))),
        })
    }

    // ── safety { } ───────────────────────────────────────────────────────────

    fn parse_safety(&mut self) -> Result<SafetyBlock, VosaError> {
        self.expect(&TokenKind::LBrace)?;
        let mut block = SafetyBlock::default();

        loop {
            match self.peek().clone() {
                TokenKind::RBrace => {
                    self.advance();
                    break;
                }
                TokenKind::Eof => return Err(self.parse_err("unexpected EOF in safety block")),
                TokenKind::MaxAltitude => {
                    self.advance();
                    self.expect(&TokenKind::Colon)?;
                    block.max_altitude = Some(self.expect_number()?);
                }
                TokenKind::MinAltitude => {
                    self.advance();
                    self.expect(&TokenKind::Colon)?;
                    block.min_altitude = Some(self.expect_number()?);
                }
                TokenKind::MaxSpeed => {
                    self.advance();
                    self.expect(&TokenKind::Colon)?;
                    block.max_speed = Some(self.expect_number()?);
                }
                TokenKind::BatteryReserve => {
                    self.advance();
                    self.expect(&TokenKind::Colon)?;
                    block.battery_reserve = Some(self.expect_number()?);
                }
                TokenKind::Failsafe => {
                    self.advance();
                    self.expect(&TokenKind::Colon)?;
                    block.failsafe = Some(self.parse_failsafe_action()?);
                }
                TokenKind::Geofence => {
                    self.advance();
                    self.expect(&TokenKind::Colon)?;
                    block.geofence = Some(self.parse_geofence()?);
                }
                other => {
                    return Err(
                        self.parse_err(format!("unexpected key in safety block: {:?}", other))
                    )
                }
            }
        }
        Ok(block)
    }

    fn parse_failsafe_action(&mut self) -> Result<FailsafeAction, VosaError> {
        match self.consume() {
            TokenKind::ReturnHome => Ok(FailsafeAction::ReturnHome),
            TokenKind::Land => Ok(FailsafeAction::Land),
            TokenKind::Hover => Ok(FailsafeAction::Hover),
            other => Err(self.parse_err(format!("expected failsafe action, got {:?}", other))),
        }
    }

    /// Parse a geofence shape.
    ///
    /// Supported forms:
    /// - `circle(center: home, radius: 500m)` — geofence centered on launch point
    /// - `circle(lat: 38.897, lon: -77.036, radius: 500m)` — geofence centered on a GPS coordinate
    ///
    /// The coordinate form is required for geofence enforcement in the simulator,
    /// since "home" is only known at runtime on real hardware.
    fn parse_geofence(&mut self) -> Result<Geofence, VosaError> {
        self.expect(&TokenKind::Circle)?;
        self.expect(&TokenKind::LParen)?;

        let mut center: Option<GeoCenter> = None;
        let mut radius: Option<f64> = None;
        let mut lat: Option<f64> = None;
        let mut lon: Option<f64> = None;

        loop {
            match self.peek().clone() {
                TokenKind::RParen => {
                    self.advance();
                    break;
                }
                TokenKind::Eof => return Err(self.parse_err("unexpected EOF in geofence")),
                TokenKind::Center => {
                    self.advance();
                    self.expect(&TokenKind::Colon)?;
                    center = Some(match self.consume() {
                        TokenKind::Home => GeoCenter::Home,
                        other => {
                            return Err(self
                                .parse_err(format!("expected geofence center, got {:?}", other)))
                        }
                    });
                    if self.peek() == &TokenKind::Comma {
                        self.advance();
                    }
                }
                TokenKind::Lat => {
                    self.advance();
                    self.expect(&TokenKind::Colon)?;
                    lat = Some(self.expect_number()?);
                    if self.peek() == &TokenKind::Comma {
                        self.advance();
                    }
                }
                TokenKind::Lon => {
                    self.advance();
                    self.expect(&TokenKind::Colon)?;
                    lon = Some(self.expect_number()?);
                    if self.peek() == &TokenKind::Comma {
                        self.advance();
                    }
                }
                TokenKind::Radius => {
                    self.advance();
                    self.expect(&TokenKind::Colon)?;
                    radius = Some(self.expect_number()?);
                    if self.peek() == &TokenKind::Comma {
                        self.advance();
                    }
                }
                other => {
                    return Err(self.parse_err(format!("unexpected token in geofence: {:?}", other)))
                }
            }
        }

        // Resolve the center: explicit `center: home` wins; otherwise use lat/lon coords.
        let resolved_center = center
            .or_else(|| match (lat, lon) {
                (Some(la), Some(lo)) => Some(GeoCenter::Coord { lat: la, lon: lo }),
                _ => None,
            })
            .ok_or_else(|| {
                self.parse_err(
                    "geofence circle requires either 'center: home' or 'lat:' and 'lon:' params",
                )
            })?;

        Ok(Geofence::Circle {
            center: resolved_center,
            radius: radius.ok_or_else(|| self.parse_err("geofence circle requires 'radius'"))?,
        })
    }

    // ── flight { } ───────────────────────────────────────────────────────────

    fn parse_flight(&mut self) -> Result<FlightConfig, VosaError> {
        self.expect(&TokenKind::LBrace)?;
        let mut cfg = FlightConfig::default();

        loop {
            match self.peek().clone() {
                TokenKind::RBrace => {
                    self.advance();
                    break;
                }
                TokenKind::Eof => return Err(self.parse_err("unexpected EOF in flight block")),
                TokenKind::CruiseAltitude => {
                    self.advance();
                    self.expect(&TokenKind::Colon)?;
                    cfg.cruise_altitude = Some(self.expect_number()?);
                }
                TokenKind::CruiseSpeed => {
                    self.advance();
                    self.expect(&TokenKind::Colon)?;
                    cfg.cruise_speed = Some(self.expect_number()?);
                }
                other => {
                    return Err(
                        self.parse_err(format!("unexpected key in flight block: {:?}", other))
                    )
                }
            }
        }
        Ok(cfg)
    }

    // ── sequence { } ─────────────────────────────────────────────────────────

    fn parse_sequence(&mut self) -> Result<Sequence, VosaError> {
        self.expect(&TokenKind::LBrace)?;
        let mut statements = Vec::new();

        loop {
            match self.peek().clone() {
                TokenKind::RBrace => {
                    self.advance();
                    break;
                }
                TokenKind::Eof => return Err(self.parse_err("unexpected EOF in sequence block")),
                _ => statements.push(self.parse_statement()?),
            }
        }
        Ok(Sequence { statements })
    }

    fn parse_statement(&mut self) -> Result<Statement, VosaError> {
        match self.peek().clone() {
            TokenKind::Repeat => self.parse_repeat(),
            TokenKind::If => self.parse_if(),
            TokenKind::Parallel => self.parse_parallel(),
            TokenKind::On => self.parse_on(),
            _ => Ok(Statement::Command(self.parse_command()?)),
        }
    }

    fn parse_repeat(&mut self) -> Result<Statement, VosaError> {
        self.expect(&TokenKind::Repeat)?;

        let count = match self.consume() {
            TokenKind::Number(n) => n as usize,
            other => {
                return Err(self.parse_err(format!("expected repeat count number, got {:?}", other)))
            }
        };

        let body = self.parse_sequence()?;
        Ok(Statement::Repeat { count, body })
    }

    fn parse_parallel(&mut self) -> Result<Statement, VosaError> {
        self.expect(&TokenKind::Parallel)?;
        let body = self.parse_sequence()?;
        Ok(Statement::Parallel { body })
    }

    fn parse_if(&mut self) -> Result<Statement, VosaError> {
        self.expect(&TokenKind::If)?;

        let target = self.consume();
        if target != TokenKind::Battery {
            return Err(self.parse_err(format!(
                "'if' currently only supports 'battery', got {:?}",
                target
            )));
        }

        let operator = match self.consume() {
            TokenKind::LessThan => Operator::LessThan,
            TokenKind::GreaterThan => Operator::GreaterThan,
            other => return Err(self.parse_err(format!("expected '<' or '>', got {:?}", other))),
        };

        let threshold_percent = match self.consume() {
            TokenKind::Quantity(v, Unit::Percent) => v,
            TokenKind::Number(v) => v,
            other => {
                return Err(self.parse_err(format!("expected battery threshold, got {:?}", other)))
            }
        };

        let body = self.parse_sequence()?;

        Ok(Statement::IfBattery {
            operator,
            threshold_percent,
            body,
        })
    }

    /// Parse a reactive `on <condition> { ... }` trigger block.
    ///
    /// Supported conditions:
    /// - `on battery < 20% { ... }`
    /// - `on battery > 50% { ... }`
    /// - `on wind > 15m/s { ... }`
    /// - `on wind < 5m/s { ... }`
    /// - `on obstacle_detected { ... }`
    fn parse_on(&mut self) -> Result<Statement, VosaError> {
        self.expect(&TokenKind::On)?;

        // Parse first (left-hand) condition, then look for `and` / `or`
        let mut condition = self.parse_single_condition()?;

        loop {
            match self.peek() {
                TokenKind::And => {
                    self.advance();
                    let right = self.parse_single_condition()?;
                    condition = TriggerCondition::And(Box::new(condition), Box::new(right));
                }
                TokenKind::Or => {
                    self.advance();
                    let right = self.parse_single_condition()?;
                    condition = TriggerCondition::Or(Box::new(condition), Box::new(right));
                }
                _ => break,
            }
        }

        // Optional `for <duration>` — condition must hold this long before firing
        let duration_s = if self.peek() == &TokenKind::For {
            self.advance();
            Some(self.expect_number()?)
        } else {
            None
        };

        let body = self.parse_sequence()?;
        Ok(Statement::OnCondition {
            condition,
            duration_s,
            body,
        })
    }

    /// Parse a `sensor <name> from <MESSAGE>.<field>` declaration.
    fn parse_sensor_binding(&mut self) -> Result<SensorBinding, VosaError> {
        let name = match self.consume() {
            TokenKind::Ident(s) => s,
            other => {
                return Err(self.parse_err(format!(
                    "expected sensor name after 'sensor', got {:?}",
                    other
                )))
            }
        };
        self.expect(&TokenKind::From)?;
        let message = match self.consume() {
            TokenKind::Ident(s) => s,
            other => {
                return Err(self.parse_err(format!(
                    "expected MAVLink message name after 'from', got {:?}",
                    other
                )))
            }
        };
        self.expect(&TokenKind::Dot)?;
        let field = match self.consume() {
            TokenKind::Ident(s) => s,
            other => {
                return Err(
                    self.parse_err(format!("expected field name after '.', got {:?}", other))
                )
            }
        };
        Ok(SensorBinding {
            name,
            message,
            field,
        })
    }

    /// Parse a single (non-compound) trigger condition.
    fn parse_single_condition(&mut self) -> Result<TriggerCondition, VosaError> {
        match self.consume() {
            TokenKind::Battery => {
                let operator = match self.consume() {
                    TokenKind::LessThan    => Operator::LessThan,
                    TokenKind::GreaterThan => Operator::GreaterThan,
                    other => return Err(self.parse_err(format!("expected '<' or '>' after 'battery', got {:?}", other))),
                };
                let threshold_percent = match self.consume() {
                    TokenKind::Quantity(v, Unit::Percent) => v,
                    TokenKind::Number(v) => v,
                    other => return Err(self.parse_err(format!("expected battery % threshold, got {:?}", other))),
                };
                Ok(TriggerCondition::Battery { operator, threshold_percent })
            }
            TokenKind::Wind => {
                let operator = match self.consume() {
                    TokenKind::LessThan    => Operator::LessThan,
                    TokenKind::GreaterThan => Operator::GreaterThan,
                    other => return Err(self.parse_err(format!("expected '<' or '>' after 'wind', got {:?}", other))),
                };
                let threshold_ms = match self.consume() {
                    TokenKind::Quantity(v, Unit::MetersPerSecond) => v,
                    TokenKind::Number(v) => v,
                    other => return Err(self.parse_err(format!("expected wind speed (m/s) threshold, got {:?}", other))),
                };
                Ok(TriggerCondition::Wind { operator, threshold_ms })
            }
            TokenKind::ObstacleDetected => Ok(TriggerCondition::ObstacleDetected),
            // Custom sensor declared via `sensor <name> from MESSAGE.field`
            TokenKind::Ident(name) => {
                let operator = match self.consume() {
                    TokenKind::LessThan    => Operator::LessThan,
                    TokenKind::GreaterThan => Operator::GreaterThan,
                    other => return Err(self.parse_err(format!(
                        "expected '<' or '>' after sensor name '{name}', got {:?}", other
                    ))),
                };
                let threshold = self.expect_number()?;
                Ok(TriggerCondition::Custom { name, operator, threshold })
            }
            other => Err(self.parse_err(format!(
                "'on' supports: battery, wind, obstacle_detected, or a declared sensor name — got {:?}", other
            ))),
        }
    }

    fn parse_command(&mut self) -> Result<Command, VosaError> {
        match self.consume() {
            TokenKind::Takeoff => self.parse_takeoff(),
            TokenKind::Land => {
                self.consume_empty_parens();
                Ok(Command::Land)
            }
            TokenKind::Hover => self.parse_hover(),
            TokenKind::Waypoint => self.parse_waypoint(),
            TokenKind::ReturnHome => {
                self.consume_empty_parens();
                Ok(Command::ReturnHome)
            }
            TokenKind::Camera => self.parse_camera(),
            other => Err(self.parse_err(format!("unknown command: {:?}", other))),
        }
    }

    /// Consume `()` if the next two tokens are `(` then `)` — for no-arg commands.
    fn consume_empty_parens(&mut self) {
        if self.peek() == &TokenKind::LParen {
            self.advance(); // (
                            // allow optional RParen
            if self.peek() == &TokenKind::RParen {
                self.advance(); // )
            }
        }
    }

    /// `takeoff(altitude: 10m)` or `takeoff(10m)`
    fn parse_takeoff(&mut self) -> Result<Command, VosaError> {
        self.expect(&TokenKind::LParen)?;
        // Support both `altitude: 10m` and bare `10m`
        let altitude = if matches!(self.peek(), TokenKind::Altitude | TokenKind::Alt) {
            self.advance();
            self.expect(&TokenKind::Colon)?;
            self.expect_number()?
        } else {
            self.expect_number()?
        };
        self.expect(&TokenKind::RParen)?;
        Ok(Command::Takeoff { altitude })
    }

    /// `hover(5s)` or `hover(duration: 5s)`
    fn parse_hover(&mut self) -> Result<Command, VosaError> {
        self.expect(&TokenKind::LParen)?;
        let duration = if self.peek() == &TokenKind::Duration {
            self.advance();
            self.expect(&TokenKind::Colon)?;
            self.expect_number()?
        } else {
            self.expect_number()?
        };
        self.expect(&TokenKind::RParen)?;
        Ok(Command::Hover { duration })
    }

    /// `waypoint(lat: 38.897, lon: -77.036, alt: 50m)`
    fn parse_waypoint(&mut self) -> Result<Command, VosaError> {
        self.expect(&TokenKind::LParen)?;

        let mut lat: Option<f64> = None;
        let mut lon: Option<f64> = None;
        let mut alt: Option<f64> = None;

        loop {
            match self.peek().clone() {
                TokenKind::RParen => {
                    self.advance();
                    break;
                }
                TokenKind::Eof => return Err(self.parse_err("unexpected EOF in waypoint")),
                TokenKind::Lat => {
                    self.advance();
                    self.expect(&TokenKind::Colon)?;
                    lat = Some(self.expect_number()?);
                    if self.peek() == &TokenKind::Comma {
                        self.advance();
                    }
                }
                TokenKind::Lon => {
                    self.advance();
                    self.expect(&TokenKind::Colon)?;
                    lon = Some(self.expect_number()?);
                    if self.peek() == &TokenKind::Comma {
                        self.advance();
                    }
                }
                TokenKind::Alt | TokenKind::Altitude => {
                    self.advance();
                    self.expect(&TokenKind::Colon)?;
                    alt = Some(self.expect_number()?);
                    if self.peek() == &TokenKind::Comma {
                        self.advance();
                    }
                }
                other => {
                    return Err(self.parse_err(format!("unexpected param in waypoint: {:?}", other)))
                }
            }
        }

        Ok(Command::Waypoint {
            lat: lat.ok_or_else(|| self.parse_err("waypoint requires 'lat'"))?,
            lon: lon.ok_or_else(|| self.parse_err("waypoint requires 'lon'"))?,
            alt: alt.ok_or_else(|| self.parse_err("waypoint requires 'alt'"))?,
        })
    }

    /// `camera(action: record, resolution: 4K)`
    fn parse_camera(&mut self) -> Result<Command, VosaError> {
        self.expect(&TokenKind::LParen)?;

        let mut action: Option<CameraAction> = None;
        let mut resolution: Option<String> = None;

        loop {
            match self.peek().clone() {
                TokenKind::RParen => {
                    self.advance();
                    break;
                }
                TokenKind::Eof => return Err(self.parse_err("unexpected EOF in camera")),
                TokenKind::Action => {
                    self.advance();
                    self.expect(&TokenKind::Colon)?;
                    action = Some(match self.consume() {
                        TokenKind::Record => CameraAction::Record,
                        TokenKind::Photo => CameraAction::Photo,
                        TokenKind::Stop => CameraAction::Stop,
                        other => {
                            return Err(
                                self.parse_err(format!("unknown camera action: {:?}", other))
                            )
                        }
                    });
                    if self.peek() == &TokenKind::Comma {
                        self.advance();
                    }
                }
                TokenKind::Resolution => {
                    self.advance();
                    self.expect(&TokenKind::Colon)?;
                    resolution = Some(match self.consume() {
                        TokenKind::StringLit(s) => s,
                        TokenKind::Ident(s) => s,
                        other => {
                            return Err(self
                                .parse_err(format!("expected resolution string, got {:?}", other)))
                        }
                    });
                    if self.peek() == &TokenKind::Comma {
                        self.advance();
                    }
                }
                other => {
                    return Err(self.parse_err(format!("unexpected param in camera: {:?}", other)))
                }
            }
        }

        Ok(Command::Camera {
            action: action.ok_or_else(|| self.parse_err("camera requires 'action'"))?,
            resolution,
        })
    }
}
