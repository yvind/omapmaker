use thiserror::Error;

/// crate specific Result type
pub type Result<T> = std::result::Result<T, Error>;

/// crate specific Error enum
#[derive(Error, Debug)]
pub enum Error {
    #[error(transparent)]
    EframeError(#[from] eframe::Error),
}
