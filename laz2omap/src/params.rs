use std::path::PathBuf;

#[derive(Clone)]
pub struct MapParams {
    pub output_epsg: Option<u16>,

    pub scale: omap::Scale,

    // map parameters
    pub simplification_distance: f64,
    pub bezier_error: f64,
    pub basemap_interval: f64,
    pub contour_interval: f64,
    pub green: (f64, f64, f64),
    pub yellow: f64,
    pub cliff: f64,

    // debug params
    pub contour_algo_steps: u8,
    pub contour_algo_lambda: f64,

    pub basemap_contour: bool,
    pub formlines: bool,
    pub bezier_bool: bool,
}

impl Default for MapParams {
    fn default() -> Self {
        Self {
            scale: omap::Scale::S15_000,
            output_epsg: None,
            simplification_distance: 0.1,
            bezier_error: 0.4,
            basemap_interval: 0.5,
            contour_interval: 5.,
            green: (0.2, 0.5, 0.8),
            yellow: 0.5,
            contour_algo_steps: 5,
            contour_algo_lambda: 3.,
            basemap_contour: false,
            formlines: false,
            bezier_bool: true,
            cliff: 0.7,
        }
    }
}

#[derive(Default, Clone)]
pub struct FileParams {
    pub paths: Vec<PathBuf>,
    pub save_location: PathBuf,
    pub tiff_location: Option<PathBuf>,

    // lidar file overlay
    pub selected_file: Option<usize>,

    // lidar crs's
    pub crs_epsg: Vec<u16>,
}
