use thiserror::Error;

pub type Result<T> = anyhow::Result<T>;

/// crate specific Error enum
#[derive(Error, Debug)]
pub enum Error {
    #[error(transparent)]
    Eframe(#[from] eframe::Error),
    #[error(transparent)]
    Omap(#[from] omap::Error),
    #[error(transparent)]
    Proj(#[from] proj_core::Error),
    #[error("The chosen polygon filter does not intersect the lidar files")]
    MapAreaDistinctFromLidarArea,
    #[error(transparent)]
    Copc(#[from] copc_converter::Error),
    #[error(transparent)]
    Las(#[from] las::Error),
    #[error("The area contains no ground points")]
    NoGroundPoints,
    #[error("{algorithm} is not available for {feature}; rebuild with the required capability")]
    AlgorithmUnavailable {
        feature: &'static str,
        algorithm: &'static str,
    },
}
