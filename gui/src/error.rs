use thiserror::Error;

/// crate specific Error enum
#[derive(Error, Debug)]
pub enum Error {
    #[error(transparent)]
    EframeError(#[from] eframe::Error),
}
