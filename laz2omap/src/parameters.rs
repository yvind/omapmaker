use std::{fmt::Display, path::PathBuf};

#[derive(Clone)]
pub struct MapParameters {
    pub output_epsg: Option<u16>,

    pub scale: omap::Scale,

    pub contour_algorithm: ContourAlgo,

    // map parameters
    pub bezier_error: f64,
    pub basemap_interval: f64,
    pub contour_interval: f64,
    pub green: (f64, f64, f64),
    pub yellow: f64,
    pub cliff: f64,

    // debug params
    pub contour_algo_steps: u8,
    pub contour_algo_lambda: f64,
    pub formline_prune: f64,

    pub basemap_contour: bool,
    pub formlines: bool,
    pub bezier_bool: bool,
}

impl Default for MapParameters {
    fn default() -> Self {
        Self {
            scale: omap::Scale::S15_000,
            output_epsg: None,
            bezier_error: 0.5,
            basemap_interval: 0.5,
            contour_interval: 5.,
            green: (0.4, 0.6, 0.8),
            yellow: 0.01,
            contour_algo_steps: 0,
            contour_algo_lambda: 0.1,
            basemap_contour: false,
            formlines: false,
            formline_prune: 0.5,
            bezier_bool: true,
            cliff: 0.75,
            contour_algorithm: Default::default(),
        }
    }
}

#[derive(Default, Clone)]
pub struct FileParameters {
    pub paths: Vec<PathBuf>,
    pub save_location: PathBuf,

    // lidar file overlay
    pub selected_file: Option<usize>,

    // lidar crs's
    pub crs_epsg: Vec<u16>,
}

#[derive(Debug, Default, Clone, Copy, PartialEq)]
pub enum ContourAlgo {
    AI,
    NaiveIterations,
    NormalFieldSmoothing,
    #[default]
    Raw,
}

impl Display for ContourAlgo {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            ContourAlgo::AI => f.write_str("AI"),
            ContourAlgo::NaiveIterations => f.write_str("Naive"),
            ContourAlgo::NormalFieldSmoothing => f.write_str("Smooth"),
            ContourAlgo::Raw => f.write_str("Raw"),
        }
    }
}
