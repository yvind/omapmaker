mod algorithms;

use crate::{
    map::MapObject,
    parameters::{MapParameters, StreamAlgorithm},
    raster::D8Flow,
};

pub(crate) fn stream_features(
    prediction: &crate::inference::models::ditches_streams_svf_slope::Prediction,
    cut_overlay: &geo::Polygon,
    parameters: &MapParameters,
) -> crate::Result<Vec<MapObject>> {
    #[cfg(feature = "stream-svf-slope")]
    {
        algorithms::svf_slope::stream_features(prediction, cut_overlay, parameters)
    }
    #[cfg(not(feature = "stream-svf-slope"))]
    {
        let _ = (prediction, cut_overlay, parameters);
        Err(crate::Error::AlgorithmUnavailable {
            feature: "streams",
            algorithm: "ditches-streams-svf-slope",
        }
        .into())
    }
}

pub(crate) const fn available_algorithms() -> &'static [StreamAlgorithm] {
    algorithms::AVAILABLE
}

pub(crate) fn ensure_available(algorithm: StreamAlgorithm) -> crate::Result<()> {
    if available_algorithms().contains(&algorithm) {
        Ok(())
    } else {
        Err(crate::Error::AlgorithmUnavailable {
            feature: "streams",
            algorithm: "ditches-streams-svf-slope",
        }
        .into())
    }
}

pub(crate) fn compute_streams(
    flow: &D8Flow,
    cut_overlay: &geo::Polygon,
    params: &MapParameters,
) -> Vec<MapObject> {
    algorithms::compute(flow, cut_overlay, params)
}

pub(crate) fn compute_selected(
    flow: &D8Flow,
    cut_overlay: &geo::Polygon,
    params: &MapParameters,
    model: impl FnOnce() -> crate::Result<Vec<MapObject>>,
) -> crate::Result<Vec<MapObject>> {
    ensure_available(params.streams.algorithm)?;
    match params.streams.algorithm {
        StreamAlgorithm::Hydrological => Ok(algorithms::compute(flow, cut_overlay, params)),
        StreamAlgorithm::DitchesStreamsSvfSlope => model(),
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn registry_reports_only_compiled_algorithms() {
        assert!(available_algorithms().contains(&StreamAlgorithm::Hydrological));
        assert_eq!(
            available_algorithms().contains(&StreamAlgorithm::DitchesStreamsSvfSlope),
            cfg!(feature = "stream-svf-slope")
        );
    }
}
