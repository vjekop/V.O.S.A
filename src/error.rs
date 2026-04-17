use thiserror::Error;

/// All errors that can occur during lexing, parsing, safety checking, or runtime.
#[derive(Debug, Error)]
pub enum VosaError {
    #[error("Lexer error at column {col}: {msg}")]
    LexError { col: usize, msg: String },

    #[error("Parse error on line {line}: {msg}")]
    ParseError { line: usize, msg: String },

    #[error("Safety violation: {0}")]
    SafetyViolation(String),

    #[error("Failsafe triggered: {0}")]
    FailsafeTriggered(String),

    #[error("Runtime error: {0}")]
    RuntimeError(String),

    #[error("IO error: {0}")]
    IoError(#[from] std::io::Error),
}
