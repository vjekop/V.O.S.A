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
            Err(self.parse_err(format!(
                "expected {:?}, got {:?}",
                expected,
                self.peek()
            )))
        }
    }

    /// Consume the next token and return it as a clone.
    fn consume(&mut self) -> TokenKind {
        self.advance().kind.clone()
    }

    // ── Quantity helpers ──────────────────────────────────────────────────────

    /// Expect a Quantity with the given unit.
    fn expect_quantity(&mut self, unit: Unit) -> Result<f64, VosaError> {
        match self.consume() {
            TokenKind::Quantity(v, u) if u == unit => Ok(v),
            TokenKind::Number(v) => Ok(v), // bare numbers are accepted (unit inferred)
            other => Err(self.parse_err(format!("expected a quantity with unit {:?}, got {:?}", unit, other))),
        }
    }

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
            other => return Err(self.parse_err(format!("expected mission name string, got {:?}", other))),
        };

        self.expect(&TokenKind::LBrace)?;

        let mut vehicle: Option<VehicleKind> = None;
        let mut safety: Option<SafetyBlock> = None;
        let mut flight: Option<FlightConfig> = None;
        let mut sequence: Option<Sequence> = None;

        loop {
            match self.peek().clone() {
                TokenKind::RBrace => { self.advance(); break; }
                TokenKind::Eof    => return Err(self.parse_err("unexpected EOF inside mission block")),
                TokenKind::Vehicle   => { self.advance(); vehicle = Some(self.parse_vehicle()?); }
                TokenKind::Safety    => { self.advance(); safety = Some(self.parse_safety()?); }
                TokenKind::Flight    => { self.advance(); flight = Some(self.parse_flight()?); }
                TokenKind::Sequence  => { self.advance(); sequence = Some(self.parse_sequence()?); }
                other => return Err(self.parse_err(format!("unexpected token in mission body: {:?}", other))),
            }
        }

        self.expect(&TokenKind::Eof)?;

        Ok(Mission {
            name,
            vehicle,
            safety,
            flight,
            sequence: sequence.ok_or_else(|| self.parse_err("mission must have a sequence block"))?,
        })
    }

    // ── vehicle ───────────────────────────────────────────────────────────────

    fn parse_vehicle(&mut self) -> Result<VehicleKind, VosaError> {
        self.expect(&TokenKind::Colon)?;
        Ok(match self.consume() {
            TokenKind::Quadcopter => VehicleKind::Quadcopter,
            TokenKind::FixedWing  => VehicleKind::FixedWing,
            TokenKind::Hexacopter => VehicleKind::Hexacopter,
            TokenKind::Ident(s)   => VehicleKind::Custom(s),
            other => return Err(self.parse_err(format!("expected vehicle kind, got {:?}", other))),
        })
    }

    // ── safety { } ───────────────────────────────────────────────────────────

    fn parse_safety(&mut self) -> Result<SafetyBlock, VosaError> {
        self.expect(&TokenKind::LBrace)?;
        let mut block = SafetyBlock::default();

        loop {
            match self.peek().clone() {
                TokenKind::RBrace => { self.advance(); break; }
                TokenKind::Eof    => return Err(self.parse_err("unexpected EOF in safety block")),
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
                other => return Err(self.parse_err(format!("unexpected key in safety block: {:?}", other))),
            }
        }
        Ok(block)
    }

    fn parse_failsafe_action(&mut self) -> Result<FailsafeAction, VosaError> {
        match self.consume() {
            TokenKind::ReturnHome => Ok(FailsafeAction::ReturnHome),
            TokenKind::Land       => Ok(FailsafeAction::Land),
            TokenKind::Hover      => Ok(FailsafeAction::Hover),
            other => Err(self.parse_err(format!("expected failsafe action, got {:?}", other))),
        }
    }

    /// Parse `circle(center: home, radius: 500m)`
    fn parse_geofence(&mut self) -> Result<Geofence, VosaError> {
        self.expect(&TokenKind::Circle)?;
        self.expect(&TokenKind::LParen)?;

        let mut center: Option<GeoCenter> = None;
        let mut radius: Option<f64> = None;

        loop {
            match self.peek().clone() {
                TokenKind::RParen => { self.advance(); break; }
                TokenKind::Eof    => return Err(self.parse_err("unexpected EOF in geofence")),
                TokenKind::Center => {
                    self.advance();
                    self.expect(&TokenKind::Colon)?;
                    center = Some(match self.consume() {
                        TokenKind::Home => GeoCenter::Home,
                        other => return Err(self.parse_err(format!("expected geofence center, got {:?}", other))),
                    });
                    // optional comma
                    if self.peek() == &TokenKind::Comma { self.advance(); }
                }
                TokenKind::Radius => {
                    self.advance();
                    self.expect(&TokenKind::Colon)?;
                    radius = Some(self.expect_number()?);
                    if self.peek() == &TokenKind::Comma { self.advance(); }
                }
                other => return Err(self.parse_err(format!("unexpected token in geofence: {:?}", other))),
            }
        }

        Ok(Geofence::Circle {
            center: center.ok_or_else(|| self.parse_err("geofence circle requires 'center'"))?,
            radius: radius.ok_or_else(|| self.parse_err("geofence circle requires 'radius'"))?,
        })
    }

    // ── flight { } ───────────────────────────────────────────────────────────

    fn parse_flight(&mut self) -> Result<FlightConfig, VosaError> {
        self.expect(&TokenKind::LBrace)?;
        let mut cfg = FlightConfig::default();

        loop {
            match self.peek().clone() {
                TokenKind::RBrace => { self.advance(); break; }
                TokenKind::Eof    => return Err(self.parse_err("unexpected EOF in flight block")),
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
                other => return Err(self.parse_err(format!("unexpected key in flight block: {:?}", other))),
            }
        }
        Ok(cfg)
    }

    // ── sequence { } ─────────────────────────────────────────────────────────

    fn parse_sequence(&mut self) -> Result<Sequence, VosaError> {
        self.expect(&TokenKind::LBrace)?;
        let mut commands = Vec::new();

        loop {
            match self.peek().clone() {
                TokenKind::RBrace => { self.advance(); break; }
                TokenKind::Eof    => return Err(self.parse_err("unexpected EOF in sequence block")),
                _ => commands.push(self.parse_command()?),
            }
        }
        Ok(Sequence { commands })
    }

    fn parse_command(&mut self) -> Result<Command, VosaError> {
        match self.consume() {
            TokenKind::Takeoff    => self.parse_takeoff(),
            TokenKind::Land       => Ok(Command::Land),
            TokenKind::Hover      => self.parse_hover(),
            TokenKind::Waypoint   => self.parse_waypoint(),
            TokenKind::ReturnHome => Ok(Command::ReturnHome),
            TokenKind::Camera     => self.parse_camera(),
            other => Err(self.parse_err(format!("unknown command: {:?}", other))),
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
                TokenKind::RParen => { self.advance(); break; }
                TokenKind::Eof    => return Err(self.parse_err("unexpected EOF in waypoint")),
                TokenKind::Lat => {
                    self.advance();
                    self.expect(&TokenKind::Colon)?;
                    lat = Some(self.expect_number()?);
                    if self.peek() == &TokenKind::Comma { self.advance(); }
                }
                TokenKind::Lon => {
                    self.advance();
                    self.expect(&TokenKind::Colon)?;
                    lon = Some(self.expect_number()?);
                    if self.peek() == &TokenKind::Comma { self.advance(); }
                }
                TokenKind::Alt | TokenKind::Altitude => {
                    self.advance();
                    self.expect(&TokenKind::Colon)?;
                    alt = Some(self.expect_number()?);
                    if self.peek() == &TokenKind::Comma { self.advance(); }
                }
                other => return Err(self.parse_err(format!("unexpected param in waypoint: {:?}", other))),
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
                TokenKind::RParen => { self.advance(); break; }
                TokenKind::Eof    => return Err(self.parse_err("unexpected EOF in camera")),
                TokenKind::Action => {
                    self.advance();
                    self.expect(&TokenKind::Colon)?;
                    action = Some(match self.consume() {
                        TokenKind::Record => CameraAction::Record,
                        TokenKind::Photo  => CameraAction::Photo,
                        TokenKind::Stop   => CameraAction::Stop,
                        other => return Err(self.parse_err(format!("unknown camera action: {:?}", other))),
                    });
                    if self.peek() == &TokenKind::Comma { self.advance(); }
                }
                TokenKind::Resolution => {
                    self.advance();
                    self.expect(&TokenKind::Colon)?;
                    resolution = Some(match self.consume() {
                        TokenKind::StringLit(s) => s,
                        TokenKind::Ident(s)     => s,
                        other => return Err(self.parse_err(format!("expected resolution string, got {:?}", other))),
                    });
                    if self.peek() == &TokenKind::Comma { self.advance(); }
                }
                other => return Err(self.parse_err(format!("unexpected param in camera: {:?}", other))),
            }
        }

        Ok(Command::Camera {
            action: action.ok_or_else(|| self.parse_err("camera requires 'action'"))?,
            resolution,
        })
    }
}
