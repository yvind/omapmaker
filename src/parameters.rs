use std::{fmt::Display, path::PathBuf};

use proj_core::CrsDef;

use crate::map_gen::egui_map::{AreaSymbol, LineSymbol, Symbol};

#[derive(Clone, Debug)]
pub struct MapParameters {
    pub output: OutputParameters,
    pub scale: Scale,
    pub contour: ContourParameters,
    pub vegetation: VegetationParameters,
    pub geometry: GeometryParameters,
    pub intensity: IntensityParameters,
}

#[derive(Clone, Debug, Default)]
pub struct OutputParameters {
    pub crs: Option<CrsDef>,
}

#[derive(Clone, Debug)]
pub struct ContourParameters {
    pub algorithm: ContourAlgo,
    pub basemap_interval: f64,
    pub interval: f64,
    pub dot_knoll_area: (f64, f64),
    pub algo_steps: u8,
    pub algo_lambda: f64,
    pub form_line_prune: f64,
    pub basemap_contour: bool,
    pub form_lines: bool,
}

#[derive(Clone, Debug)]
pub struct VegetationParameters {
    pub green: (f64, f64, f64),
    pub yellow: f64,
    pub cliff: f64,
}

#[derive(Clone, Debug, Default)]
pub struct GeometryParameters {
    pub contours: BezierParameters,
    pub openness: BufferedGeometryParameters,
    pub vegetation: BufferedGeometryParameters,
    pub cliffs: BezierParameters,
    pub intensity: BufferedGeometryParameters,
}

#[derive(Clone, Debug, Default)]
pub struct IntensityParameters {
    pub filters: Vec<IntensityFilter>,
}

impl Default for MapParameters {
    fn default() -> Self {
        Self {
            scale: Scale::S15_000,
            output: Default::default(),
            contour: Default::default(),
            vegetation: Default::default(),
            geometry: Default::default(),
            intensity: Default::default(),
        }
    }
}

impl Default for ContourParameters {
    fn default() -> Self {
        Self {
            algorithm: Default::default(),
            basemap_interval: 0.5,
            interval: 5.,
            dot_knoll_area: (10., 160.),
            algo_steps: 0,
            algo_lambda: 0.01,
            basemap_contour: false,
            form_lines: false,
            form_line_prune: 0.5,
        }
    }
}

impl Default for VegetationParameters {
    fn default() -> Self {
        Self {
            green: (0.4, 0.6, 0.8),
            yellow: 0.01,
            cliff: 0.75,
        }
    }
}

impl GeometryParameters {
    pub fn bezier_error_for_symbol(&self, symbol: Symbol) -> Option<f64> {
        let bezier = match symbol {
            Symbol::Line(LineSymbol::Contour)
            | Symbol::Line(LineSymbol::FormLine)
            | Symbol::Line(LineSymbol::IndexContour) => &self.contours,
            Symbol::Area(AreaSymbol::RoughOpenLand) => &self.openness.bezier,
            Symbol::Area(AreaSymbol::LightGreen)
            | Symbol::Area(AreaSymbol::MediumGreen)
            | Symbol::Area(AreaSymbol::DarkGreen) => &self.vegetation.bezier,
            Symbol::Area(AreaSymbol::GiganticBoulder) => &self.cliffs,
            Symbol::Area(_) => &self.intensity.bezier,
            Symbol::Line(_) | Symbol::Point(_) => return None,
        };

        bezier.enabled.then_some(bezier.error)
    }
}

#[derive(Clone, Debug, PartialEq)]
pub struct BezierParameters {
    pub error: f64,
    pub enabled: bool,
}

impl Default for BezierParameters {
    fn default() -> Self {
        Self {
            error: 0.5,
            enabled: true,
        }
    }
}

#[derive(Clone, Debug, Default, PartialEq)]
pub struct BufferedGeometryParameters {
    pub bezier: BezierParameters,
    pub buffer_rules: Vec<BufferRule>,
}

#[derive(Default, Clone)]
pub struct FileParameters {
    pub paths: Vec<PathBuf>,
    pub save_location: PathBuf,

    // lidar crs's
    pub crs_epsg: Vec<Option<CrsDef>>,
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

#[derive(Debug, Clone, PartialEq)]
pub struct IntensityFilter {
    pub low: f64,
    pub high: f64,
    pub symbol: AreaSymbol,
}

impl Default for IntensityFilter {
    fn default() -> Self {
        Self {
            low: 0.2,
            high: 0.4,
            symbol: AreaSymbol::BareRock,
        }
    }
}

#[derive(Debug, Clone, Copy, PartialEq)]
pub struct BufferRule {
    pub direction: BufferDirection,
    pub amount: f64,
}

impl Default for BufferRule {
    fn default() -> Self {
        Self {
            direction: BufferDirection::Grow,
            amount: 2.,
        }
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum BufferDirection {
    Grow,
    Shrink,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, PartialOrd, Ord)]
pub enum Scale {
    S10_000,
    S15_000,
}
