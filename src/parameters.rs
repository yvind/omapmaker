use std::{fmt::Display, path::PathBuf};

use proj_core::CrsDef;

use crate::map_gen::egui_map::{AreaSymbol, LineSymbol, Symbol};

#[derive(Clone, Debug, Default)]
pub struct MapParameters {
    pub output: OutputParameters,
    pub scale: Scale,
    pub contour: ContourParameters,
    pub vegetation: VegetationParameters,
    pub geometry: GeometryParameters,
    pub intensity: IntensityParameters,
    pub cliff: CliffParameters,
    pub water: WaterParameters,
}

#[derive(Clone, Debug, Default)]
pub struct OutputParameters {
    pub crs: Option<CrsDef>,
}

impl MapParameters {
    pub fn min_size_filter_symbols(
        &self,
        openness: bool,
        vegetation: bool,
        cliffs: bool,
        intensity: bool,
        water: bool,
    ) -> Vec<AreaSymbol> {
        let mut symbols = Vec::new();

        if openness && self.geometry.openness.min_size_filter {
            push_unique_area_symbol(&mut symbols, AreaSymbol::RoughOpenLand);
        }

        if vegetation && self.geometry.vegetation.min_size_filter {
            push_unique_area_symbol(&mut symbols, AreaSymbol::LightGreen);
            push_unique_area_symbol(&mut symbols, AreaSymbol::MediumGreen);
            push_unique_area_symbol(&mut symbols, AreaSymbol::DarkGreen);
        }

        if cliffs && self.geometry.cliffs.min_size_filter {
            push_unique_area_symbol(&mut symbols, AreaSymbol::GiganticBoulder);
        }

        if intensity && self.geometry.intensity.min_size_filter {
            for filter in &self.intensity.filters {
                push_unique_area_symbol(&mut symbols, filter.symbol);
            }
        }

        if water && self.geometry.water.min_size_filter {
            push_unique_area_symbol(&mut symbols, AreaSymbol::UncrossableWaterWithBankLine);
        }

        symbols
    }
}

fn push_unique_area_symbol(symbols: &mut Vec<AreaSymbol>, symbol: AreaSymbol) {
    if !symbols.contains(&symbol) {
        symbols.push(symbol);
    }
}

#[derive(Clone, Debug)]
pub struct ContourParameters {
    pub algorithm: ContourAlgo,
    pub basemap_interval: f64,
    pub interval: f64,
    pub dot_knoll_area: (f64, f64),
    pub algo_steps: u8,
    pub algo_lambda: f64,
    pub basemap_contour: bool,
    pub form_lines: bool,
}

#[derive(Clone, Debug)]
pub struct VegetationParameters {
    pub green: (f64, f64, f64),
    pub weights: VegetationWeights,
    pub yellow: f64,
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct VegetationWeights {
    pub low: f64,
    pub medium: f64,
    pub high: f64,
}

#[derive(Clone, Debug, Default)]
pub struct GeometryParameters {
    pub contours: BezierParameters,
    pub openness: BufferedGeometryParameters,
    pub vegetation: BufferedGeometryParameters,
    pub cliffs: BufferedGeometryParameters,
    pub intensity: BufferedGeometryParameters,
    pub water: BufferedGeometryParameters,
}

#[derive(Clone, Debug, PartialEq)]
pub struct WaterParameters {
    pub threshold: f64,
}

impl Default for WaterParameters {
    fn default() -> Self {
        Self { threshold: 0.65 }
    }
}

#[derive(Clone, Debug, PartialEq)]
pub struct CliffParameters {
    pub cliff: f64,
}

impl Default for CliffParameters {
    fn default() -> Self {
        Self { cliff: 2.5 }
    }
}

#[derive(Clone, Debug, Default, PartialEq)]
pub struct IntensityParameters {
    pub filters: Vec<IntensityFilter>,
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
        }
    }
}

impl Default for VegetationParameters {
    fn default() -> Self {
        Self {
            green: (0.4, 0.6, 0.8),
            weights: Default::default(),
            yellow: 0.01,
        }
    }
}

impl Default for VegetationWeights {
    fn default() -> Self {
        Self {
            low: 0.5,
            medium: 0.35,
            high: 0.15,
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
            Symbol::Area(AreaSymbol::GiganticBoulder) => &self.cliffs.bezier,
            Symbol::Area(AreaSymbol::UncrossableWaterWithBankLine) => &self.water.bezier,
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
            error: 2.0,
            enabled: true,
        }
    }
}

#[derive(Clone, Debug, Default, PartialEq)]
pub struct BufferedGeometryParameters {
    pub bezier: BezierParameters,
    pub buffer_rules: Vec<BufferRule>,
    pub min_size_filter: bool,
}

#[derive(Default, Clone)]
pub struct FileParameters {
    pub paths: Vec<PathBuf>,
    pub save_location: PathBuf,
    pub save_slope_raster: bool,
    pub save_hillshade_raster: bool,
    pub save_last_return_raster: bool,
    pub save_canopy_height_raster: bool,
    pub save_surface_objects_raster: bool,
    pub save_ndvd_raster: bool,

    // lidar crs's
    pub crs_epsg: Vec<Option<CrsDef>>,
}

#[derive(Debug, Default, Clone, Copy, PartialEq)]
pub enum ContourAlgo {
    NaiveIterations,
    NormalFieldSmoothing,
    #[default]
    Raw,
}

impl Display for ContourAlgo {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
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
