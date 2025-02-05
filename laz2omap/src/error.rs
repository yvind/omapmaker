use thiserror::Error;

pub type Result<T> = std::result::Result<T, Error>;

/// crate specific Error enum
#[derive(Error, Debug)]
pub enum Error {
    #[error(transparent)]
    EframeError(#[from] eframe::Error),
    #[error(transparent)]
    OmapError(#[from] omap::OmapError),
    #[error(transparent)]
    ProjError(#[from] proj4rs::errors::Error),
}
