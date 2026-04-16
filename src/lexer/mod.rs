pub mod token;
pub use token::{Token, TokenKind, Unit};

use crate::error::VosaError;

/// The VOSA lexer converts raw source text into a flat list of tokens.
pub struct Lexer {
    src: Vec<char>,
    pos: usize,
    line: usize,
    col: usize,
}

impl Lexer {
    pub fn new(source: &str) -> Self {
        Lexer {
            src: source.chars().collect(),
            pos: 0,
            line: 1,
            col: 1,
        }
    }

    /// Consume the entire source and return all tokens (including a trailing `Eof`).
    pub fn tokenize(mut self) -> Result<Vec<Token>, VosaError> {
        let mut tokens = Vec::new();
        loop {
            let tok = self.next_token()?;
            let done = tok.kind == TokenKind::Eof;
            tokens.push(tok);
            if done {
                break;
            }
        }
        Ok(tokens)
    }

    // ── Internal helpers ─────────────────────────────────────────────────────

    fn peek(&self) -> Option<char> {
        self.src.get(self.pos).copied()
    }

    fn peek2(&self) -> Option<char> {
        self.src.get(self.pos + 1).copied()
    }

    fn advance(&mut self) -> Option<char> {
        let ch = self.src.get(self.pos).copied()?;
        self.pos += 1;
        if ch == '\n' {
            self.line += 1;
            self.col = 1;
        } else {
            self.col += 1;
        }
        Some(ch)
    }

    fn skip_whitespace_and_comments(&mut self) {
        loop {
            // Skip whitespace
            while matches!(self.peek(), Some(c) if c.is_whitespace()) {
                self.advance();
            }
            // Skip line comments  //
            if self.peek() == Some('/') && self.peek2() == Some('/') {
                while !matches!(self.peek(), Some('\n') | None) {
                    self.advance();
                }
            } else {
                break;
            }
        }
    }

    fn lex_err(&self, msg: impl Into<String>) -> VosaError {
        VosaError::LexError {
            col: self.col,
            msg: msg.into(),
        }
    }

    // ── Token productions ────────────────────────────────────────────────────

    fn next_token(&mut self) -> Result<Token, VosaError> {
        self.skip_whitespace_and_comments();
        let line = self.line;
        let col = self.col;

        let ch = match self.peek() {
            None => return Ok(Token::new(TokenKind::Eof, line, col)),
            Some(c) => c,
        };

        // Single-character punctuation
        let kind = match ch {
            '{' => { self.advance(); TokenKind::LBrace }
            '}' => { self.advance(); TokenKind::RBrace }
            '(' => { self.advance(); TokenKind::LParen }
            ')' => { self.advance(); TokenKind::RParen }
            ':' => { self.advance(); TokenKind::Colon }
            ',' => { self.advance(); TokenKind::Comma }
            '<' => { self.advance(); TokenKind::LessThan }
            '>' => { self.advance(); TokenKind::GreaterThan }
            '"' => self.lex_string()?,
            c if c.is_ascii_digit() || (c == '-' && self.peek2().map_or(false, |d| d.is_ascii_digit())) => {
                self.lex_number()?
            }
            c if c.is_alphabetic() || c == '_' => self.lex_identifier_or_keyword(),
            c => return Err(self.lex_err(format!("unexpected character '{c}'"))),
        };

        Ok(Token::new(kind, line, col))
    }

    fn lex_string(&mut self) -> Result<TokenKind, VosaError> {
        self.advance(); // eat opening "
        let mut s = String::new();
        loop {
            match self.advance() {
                None => return Err(self.lex_err("unterminated string literal")),
                Some('"') => break,
                Some(c) => s.push(c),
            }
        }
        Ok(TokenKind::StringLit(s))
    }

    fn lex_number(&mut self) -> Result<TokenKind, VosaError> {
        let mut raw = String::new();

        // Optional leading minus
        if self.peek() == Some('-') {
            raw.push('-');
            self.advance();
        }

        while matches!(self.peek(), Some(c) if c.is_ascii_digit() || c == '.') {
            raw.push(self.advance().unwrap());
        }

        let val: f64 = raw
            .parse()
            .map_err(|_| self.lex_err(format!("invalid number '{raw}'")))?;

        // Check for unit suffix
        let unit = match self.peek() {
            Some('m') => {
                self.advance(); // eat 'm'
                // Check for 'm/s'
                if self.peek() == Some('/') && self.peek2() == Some('s') {
                    self.advance(); // /
                    self.advance(); // s
                    Some(Unit::MetersPerSecond)
                } else {
                    Some(Unit::Meters)
                }
            }
            Some('s') => {
                self.advance();
                Some(Unit::Seconds)
            }
            Some('%') => {
                self.advance();
                Some(Unit::Percent)
            }
            Some('d') => {
                // deg
                if self.src.get(self.pos..self.pos + 3).map(|s| s.iter().collect::<String>()) == Some("deg".into()) {
                    self.advance(); self.advance(); self.advance();
                    Some(Unit::Degrees)
                } else {
                    None
                }
            }
            _ => None,
        };

        Ok(match unit {
            Some(u) => TokenKind::Quantity(val, u),
            None => TokenKind::Number(val),
        })
    }

    fn lex_identifier_or_keyword(&mut self) -> TokenKind {
        let mut s = String::new();
        while matches!(self.peek(), Some(c) if c.is_alphanumeric() || c == '_') {
            s.push(self.advance().unwrap());
        }
        self.keyword_or_ident(s)
    }

    fn keyword_or_ident(&self, s: String) -> TokenKind {
        match s.as_str() {
            // Block keywords
            "mission"       => TokenKind::Mission,
            "safety"        => TokenKind::Safety,
            "flight"        => TokenKind::Flight,
            "sequence"      => TokenKind::Sequence,
            "vehicle"       => TokenKind::Vehicle,
            "repeat"        => TokenKind::Repeat,
            "if"            => TokenKind::If,
            // Named values
            "home"          => TokenKind::Home,
            "circle"        => TokenKind::Circle,
            "radius"        => TokenKind::Radius,
            "center"        => TokenKind::Center,
            // Failsafe actions / commands
            "return_home"   => TokenKind::ReturnHome,
            "land"          => TokenKind::Land,
            "hover"         => TokenKind::Hover,
            "takeoff"       => TokenKind::Takeoff,
            "waypoint"      => TokenKind::Waypoint,
            "camera"        => TokenKind::Camera,
            // Camera actions
            "record"        => TokenKind::Record,
            "photo"         => TokenKind::Photo,
            "stop"          => TokenKind::Stop,
            // Vehicle types
            "Quadcopter"    => TokenKind::Quadcopter,
            "FixedWing"     => TokenKind::FixedWing,
            "Hexacopter"    => TokenKind::Hexacopter,
            // Safety keys
            "max_altitude"  => TokenKind::MaxAltitude,
            "min_altitude"  => TokenKind::MinAltitude,
            "max_speed"     => TokenKind::MaxSpeed,
            "geofence"      => TokenKind::Geofence,
            "battery_reserve" => TokenKind::BatteryReserve,
            "failsafe"      => TokenKind::Failsafe,
            // Flight keys
            "cruise_altitude" => TokenKind::CruiseAltitude,
            "cruise_speed"  => TokenKind::CruiseSpeed,
            // Param keys
            "lat"           => TokenKind::Lat,
            "lon"           => TokenKind::Lon,
            "alt"           => TokenKind::Alt,
            "altitude"      => TokenKind::Altitude,
            "duration"      => TokenKind::Duration,
            "action"        => TokenKind::Action,
            "resolution"    => TokenKind::Resolution,
            // State
            "battery"       => TokenKind::Battery,
            // Booleans
            "true"          => TokenKind::Bool(true),
            "false"         => TokenKind::Bool(false),
            // Catch-all
            _               => TokenKind::Ident(s),
        }
    }
}
