#[cfg(feature = "stream-svf-slope")]
pub(crate) mod contract;
#[cfg(feature = "stream-svf-slope")]
mod input;
mod inputs;
#[cfg(feature = "stream-svf-slope")]
mod prediction;
#[cfg(feature = "stream-svf-slope")]
mod runtime;

use std::sync::Mutex;
#[cfg(feature = "stream-svf-slope")]
use std::{collections::VecDeque, sync::Arc};

pub(crate) use inputs::InputRasters;
#[cfg(feature = "stream-svf-slope")]
pub(crate) use prediction::PredictionRaster as StreamPrediction;
#[cfg(not(feature = "stream-svf-slope"))]
pub(crate) struct StreamPrediction;

#[cfg(feature = "stream-svf-slope")]
use input::build_input;
#[cfg(feature = "stream-svf-slope")]
use runtime::InferenceLane;

#[cfg(feature = "stream-svf-slope")]
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
struct PredictionCacheKey {
    tile_revision: u64,
    model_id: &'static str,
    onnx_sha256: &'static str,
    manifest_sha256: &'static str,
    contract_version: u32,
}

#[derive(Default)]
pub(crate) struct StreamPredictionCache {
    #[cfg(feature = "stream-svf-slope")]
    entries: VecDeque<(PredictionCacheKey, Arc<StreamPrediction>)>,
}

pub(crate) fn predict_stream(
    cache: &Mutex<StreamPredictionCache>,
    tile_revision: u64,
    rasters: InputRasters<'_>,
    cancellation: &crate::cancellation::CancellationToken,
) -> crate::Result<std::sync::Arc<StreamPrediction>> {
    #[cfg(feature = "stream-svf-slope")]
    {
        cancellation.check()?;
        let inference = inference_lane()?;
        let descriptor = inference.descriptor();
        let key = PredictionCacheKey {
            tile_revision,
            model_id: descriptor.id,
            onnx_sha256: descriptor.onnx_sha256,
            manifest_sha256: descriptor.manifest_sha256,
            contract_version: descriptor.contract_version,
        };
        let mut cache_guard = cache.lock().expect("prediction cache poisoned");
        if let Some(position) = cache_guard
            .entries
            .iter()
            .position(|(candidate, _)| *candidate == key)
        {
            let entry = cache_guard
                .entries
                .remove(position)
                .expect("cache position exists");
            let prediction = Arc::clone(&entry.1);
            cache_guard.entries.push_back(entry);
            return Ok(prediction);
        }
        drop(cache_guard);

        let input = build_input(&descriptor.input, rasters)?;
        cancellation.check()?;
        let prediction = Arc::new(inference.predict(input, cancellation)?);
        cancellation.check()?;
        let mut cache_guard = cache.lock().expect("prediction cache poisoned");
        if cache_guard.entries.len() == 2 {
            cache_guard.entries.pop_front();
        }
        cache_guard
            .entries
            .push_back((key, Arc::clone(&prediction)));
        Ok(prediction)
    }
    #[cfg(not(feature = "stream-svf-slope"))]
    {
        let _ = (cache, tile_revision, rasters, cancellation);
        Err(crate::Error::AlgorithmUnavailable {
            feature: "streams",
            algorithm: "ditches-streams-svf-slope",
        }
        .into())
    }
}

#[cfg(feature = "stream-svf-slope")]
const DITCHES_STREAMS_SVF_SLOPE_MODEL_ID: &str = "ditches_streams_svf_slope";

#[cfg(feature = "stream-svf-slope")]
fn inference_lane() -> crate::Result<Arc<InferenceLane>> {
    static LANE: Mutex<Option<Arc<InferenceLane>>> = Mutex::new(None);
    let mut slot = LANE.lock().expect("inference lane cache poisoned");
    if let Some(lane) = slot.as_ref() {
        return Ok(Arc::clone(lane));
    }
    let lane = Arc::new(InferenceLane::new(DITCHES_STREAMS_SVF_SLOPE_MODEL_ID)?);
    *slot = Some(Arc::clone(&lane));
    Ok(lane)
}
