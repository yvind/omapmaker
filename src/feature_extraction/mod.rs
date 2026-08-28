pub mod contract;
pub mod inference;
pub mod input;
pub mod postprocess;
pub mod prediction;

pub use inference::InferenceLane;
pub use input::build_input;
pub use prediction::PredictionRaster;

pub const DITCHES_STREAMS_SVF_SLOPE_MODEL_ID: &str = "ditches_streams_svf_slope";

#[cfg(feature = "deep-learning")]
pub fn inference_lane() -> crate::Result<std::sync::Arc<InferenceLane>> {
    use std::sync::{Arc, Mutex};

    static LANE: Mutex<Option<Arc<InferenceLane>>> = Mutex::new(None);
    let mut slot = LANE.lock().expect("inference lane cache poisoned");
    if let Some(lane) = slot.as_ref() {
        return Ok(Arc::clone(lane));
    }
    let lane = Arc::new(InferenceLane::new(DITCHES_STREAMS_SVF_SLOPE_MODEL_ID)?);
    *slot = Some(Arc::clone(&lane));
    Ok(lane)
}
