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
    #[error("The chosen polygon filter does not intersect the lidar files")]
    MapAreaDistinctFromLidarArea,
    #[error("Cannot create a neighborhood without a center")]
    NeighborhoodError,
    #[error(transparent)]
    CopcError(#[from] copc_rs::Error),
    #[error(transparent)]
    LasError(#[from] las::Error),
    #[error("The area contains no ground points")]
    NoGroundPoints,
}
