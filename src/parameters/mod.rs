mod contours;
mod geometry;
mod hydrology;
mod intensity;
mod output;
mod terrain_features;
mod vegetation;

pub use contours::{
    ContourAlgo, ContourFieldParameters, ContourGeneralization, ContourParameters,
    FormlinePruneAlgo,
};
pub use geometry::{BezierParameters, BufferDirection, BufferRule, GeometryParameters};
#[cfg(feature = "stream-svf-slope")]
pub use hydrology::OnnxStreamVectorizationParameters;
pub use hydrology::{
    MarshEvidenceWeights, MarshParameters, StreamAlgorithm, StreamParameters, WaterParameters,
};
pub use intensity::IntensityParameters;
pub use output::{FileParameters, OutputParameters, Scale};
pub use terrain_features::{
    BuildingClassificationEvidence, BuildingParameters, CliffAlgorithm, CliffParameters,
};
pub use vegetation::{VegetationParameters, VegetationWeights};

use crate::map::AreaSymbol;

#[derive(Clone, Debug, Default)]
pub struct MapParameters {
    pub output: OutputParameters,
    pub scale: Scale,
    pub contour: ContourParameters,
    pub vegetation: VegetationParameters,
    pub building: BuildingParameters,
    pub geometry: GeometryParameters,
    pub intensity: IntensityParameters,
    pub cliff: CliffParameters,
    pub water: WaterParameters,
    pub marsh: MarshParameters,
    pub streams: StreamParameters,
}

impl MapParameters {
    pub fn min_size_filter_symbols(
        &self,
        openness: bool,
        vegetation: bool,
        buildings: bool,
        cliffs: bool,
        intensity: bool,
        water: bool,
    ) -> Vec<AreaSymbol> {
        let mut symbols = Vec::new();
        let mut push_unique = |symbol| {
            if !symbols.contains(&symbol) {
                symbols.push(symbol);
            }
        };

        if openness && self.geometry.openness.min_size_filter {
            push_unique(AreaSymbol::RoughOpenLand);
        }
        if vegetation && self.geometry.vegetation.min_size_filter {
            push_unique(AreaSymbol::LightGreen);
            push_unique(AreaSymbol::MediumGreen);
            push_unique(AreaSymbol::DarkGreen);
        }
        if buildings && self.geometry.buildings.min_size_filter {
            push_unique(AreaSymbol::Building);
        }
        if cliffs && self.geometry.cliffs.min_size_filter {
            push_unique(AreaSymbol::GiganticBoulder);
        }
        if intensity && self.geometry.intensity.min_size_filter {
            for filter in &self.intensity.filters {
                push_unique(filter.symbol);
            }
        }
        if water && self.geometry.water.min_size_filter {
            push_unique(AreaSymbol::UncrossableWaterWithBankLine);
        }
        if water && self.geometry.marsh.min_size_filter {
            push_unique(AreaSymbol::Marsh);
        }

        symbols
    }
}
