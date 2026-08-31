use crate::raster::{
    Dfm, Elevation, FilteredSurface, Ground, GroundPointDensity, GroundRelief2m, GroundRelief5m,
    HardObjectConfidence, HardObjectHeight, HeightAboveGround, HighVegetation, Intensity,
    LastReturn, LowVegetation, MediumVegetation, PointDensity, Returns, VegetationLikelihood,
    Water,
};

/// Type-stable boundary used even when the optional model runtime is absent.
#[cfg_attr(not(feature = "stream-svf-slope"), allow(dead_code))]
pub(crate) struct InputRasters<'a> {
    pub dem: &'a Dfm<Elevation>,
    pub return_number: &'a Dfm<Returns>,
    pub intensity: &'a Dfm<Intensity>,
    pub last_return: &'a Dfm<LastReturn>,
    pub ground_vegetation: &'a Dfm<Ground>,
    pub low_vegetation: &'a Dfm<LowVegetation>,
    pub medium_vegetation: &'a Dfm<MediumVegetation>,
    pub high_vegetation: &'a Dfm<HighVegetation>,
    pub ground_relief_2m: &'a Dfm<GroundRelief2m>,
    pub ground_relief_5m: &'a Dfm<GroundRelief5m>,
    pub hard_object_height: &'a Dfm<HardObjectHeight>,
    pub hard_object_confidence: &'a Dfm<HardObjectConfidence>,
    pub vegetation_likelihood: &'a Dfm<VegetationLikelihood>,
    pub filtered_surface: &'a Dfm<FilteredSurface>,
    pub water: &'a Dfm<Water>,
    pub canopy_height: &'a Dfm<HeightAboveGround>,
    pub point_density: &'a Dfm<PointDensity>,
    pub ground_point_density: &'a Dfm<GroundPointDensity>,
}
