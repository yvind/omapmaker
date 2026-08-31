mod detection;
mod vectorize;

use crate::raster::{Dfm, MarshMask, MarshProbability, MarshReason, MarshSupport, WetnessScore};

pub(crate) use detection::compute_marsh_detection;
pub(crate) use vectorize::marsh_objects;

/// Stable codes written into [`MarshDetection::reason`]. Exclusions take
/// precedence; otherwise the code names the strongest weighted positive
/// evidence family.
#[repr(u8)]
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum MarshReasonCode {
    None = 0,
    InsufficientObservationSupport = 1,
    OpenWater = 2,
    Building = 3,
    NonPlanarSurface = 4,
    EdgeDependentDrainage = 5,
    Terrain = 10,
    Hydrology = 11,
}

pub struct MarshDetection {
    /// Evidence score before observation confidence is applied.
    pub wetness_score: Dfm<WetnessScore>,
    /// Final bounded score used for seeding and growth.
    pub probability: Dfm<MarshProbability>,
    /// Local all-return/ground-return observation confidence.
    pub support: Dfm<MarshSupport>,
    /// Exclusion or strongest-evidence diagnostic code.
    pub reason: Dfm<MarshReason>,
    /// Segmented, morphologically cleaned output mask.
    pub mask: Dfm<MarshMask>,
}
