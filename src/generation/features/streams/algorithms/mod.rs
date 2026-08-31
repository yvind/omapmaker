mod d8;
#[cfg(feature = "stream-svf-slope")]
pub(super) mod svf_slope;

use crate::parameters::StreamAlgorithm;

#[cfg(feature = "stream-svf-slope")]
pub(super) const AVAILABLE: &[StreamAlgorithm] = &[
    StreamAlgorithm::Hydrological,
    StreamAlgorithm::DitchesStreamsSvfSlope,
];
#[cfg(not(feature = "stream-svf-slope"))]
pub(super) const AVAILABLE: &[StreamAlgorithm] = &[StreamAlgorithm::Hydrological];

pub(super) use d8::compute;
