//! # V.O.S.A. — Vectorized Operational Safety Autonomy
//!
//! A Domain-Specific Language (DSL) for programming autonomous drones.
//! V.O.S.A. acts as the "nervous system" between high-level mission intent
//! and low-level machine execution, with safety enforced at the language level.

pub mod error;
pub mod lexer;
pub mod parser;
pub mod safety;
pub mod runtime;

pub use error::VosaError;

/// Parse and validate a VOSA source string, returning the mission AST.
pub fn parse(source: &str) -> Result<parser::ast::Mission, VosaError> {
    let tokens = lexer::Lexer::new(source).tokenize()?;
    let mission = parser::Parser::new(tokens).parse()?;
    Ok(mission)
}

/// Parse, validate safety constraints, and run a VOSA mission in the simulator.
pub fn run(source: &str) -> Result<runtime::ExecutionReport, VosaError> {
    let mission = parse(source)?;
    let sandbox = safety::SafetySandbox::new();
    sandbox.validate(&mission)?;
    let mut rt = runtime::Runtime::new();
    rt.execute(&mission)
}
