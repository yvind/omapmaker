use std::path::PathBuf;

use proj_core::CrsDef;

#[derive(Clone, Debug, Default)]
pub struct OutputParameters {
    pub crs: Option<CrsDef>,
}

#[derive(Default, Clone)]
pub struct FileParameters {
    pub paths: Vec<PathBuf>,
    pub save_location: PathBuf,
    /// Preserve numeric raster samples instead of normalizing them to an
    /// 8-bit viewer image. Display-only rasters ignore this setting.
    pub write_raw_raster_values: bool,
    pub save_dem_raster: bool,
    pub save_slope_raster: bool,
    pub save_hillshade_raster: bool,
    pub save_intensity_raster: bool,
    pub save_last_return_raster: bool,
    pub save_canopy_height_raster: bool,
    pub save_surface_objects_raster: bool,
    pub save_ground_relief_2m_raster: bool,
    pub save_ground_relief_5m_raster: bool,
    pub save_hard_object_height_raster: bool,
    pub save_hard_object_confidence_raster: bool,
    pub save_vegetation_likelihood_raster: bool,
    pub save_filtered_surface_raster: bool,
    pub save_ndvd_raster: bool,
    pub save_point_density_raster: bool,
    pub save_flow_accumulation_raster: bool,
    pub save_building_height_raster: bool,
    pub save_building_planarity_raster: bool,
    pub save_building_residual_raster: bool,
    pub save_building_probability_raster: bool,
    pub save_building_plane_rejected_raster: bool,
    pub save_marsh_probability_raster: bool,
    pub save_marsh_support_raster: bool,
    pub save_marsh_wetness_raster: bool,
    pub save_marsh_reason_raster: bool,
    pub crs_epsg: Vec<Option<CrsDef>>,
}

#[derive(Debug, Default, Clone, Copy, PartialEq, Eq, Hash, PartialOrd, Ord)]
pub enum Scale {
    S10_000,
    #[default]
    S15_000,
}

impl Scale {
    pub fn denominator(self) -> f64 {
        match self {
            Self::S10_000 => 10_000.,
            Self::S15_000 => 15_000.,
        }
    }

    pub fn meters_to_paper_mm(self, meters: f64) -> f64 {
        meters * 1000. / self.denominator()
    }
}
