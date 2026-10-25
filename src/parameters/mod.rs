mod boulder_fields;
mod boulders;
mod contours;
mod geometry;
mod hydrology;
mod intensity;
mod output;
mod terrain_features;
mod vegetation;

pub use boulder_fields::BoulderFieldParameters;
pub use boulders::BoulderParameters;
pub use contours::{
    ContourAlgo, ContourFieldParameters, ContourGeneralization, ContourParameters,
    FormlinePruneAlgo,
};
pub use geometry::{
    BezierParameters, BufferDirection, BufferLineCap, BufferLineJoin, BufferRule,
    BuildingGeometryParameters, GeometryParameters, RdpParameters,
};
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
    pub boulder: BoulderParameters,
    pub boulder_fields: BoulderFieldParameters,
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

        if openness && self.vegetation.openness_enabled && self.geometry.openness.min_size_filter {
            push_unique(AreaSymbol::RoughOpenLand);
        }
        if vegetation && self.vegetation.enabled && self.geometry.vegetation.min_size_filter {
            push_unique(AreaSymbol::LightGreen);
            push_unique(AreaSymbol::MediumGreen);
            push_unique(AreaSymbol::DarkGreen);
            if self.vegetation.undergrowth_enabled {
                push_unique(AreaSymbol::Undergrowth);
            }
        }
        if buildings && self.building.enabled && self.geometry.buildings.min_size_filter {
            push_unique(AreaSymbol::Building);
        }
        if cliffs && self.cliff.enabled && self.geometry.cliffs.min_size_filter {
            push_unique(AreaSymbol::GiganticBoulder);
        }
        if intensity && self.intensity.enabled && self.geometry.intensity.min_size_filter {
            for filter in &self.intensity.filters {
                push_unique(filter.symbol);
            }
        }
        if water && self.water.enabled && self.geometry.water.min_size_filter {
            push_unique(AreaSymbol::UncrossableWaterWithBankLine);
        }
        if water && self.marsh.enabled && self.geometry.marsh.min_size_filter {
            push_unique(AreaSymbol::Marsh);
        }

        symbols
    }
}

#[cfg(test)]
mod tests {
    use super::MapParameters;

    #[test]
    fn every_generation_step_defaults_to_enabled() {
        let params = MapParameters::default();
        assert!(params.contour.enabled);
        assert!(params.contour.basemap_contour);
        assert!(params.vegetation.openness_enabled);
        assert!(params.vegetation.enabled);
        assert!(params.building.enabled);
        assert!(params.cliff.enabled);
        assert!(params.boulder.enabled);
        assert!(params.boulder_fields.enabled);
        assert!(params.water.enabled);
        assert!(params.marsh.enabled);
        assert!(params.streams.enabled);
        assert!(params.intensity.enabled);
    }

    #[test]
    fn disabled_intensity_does_not_apply_its_size_filter_to_vegetation() {
        let mut params = MapParameters::default();
        params.geometry.vegetation.min_size_filter = false;
        params.geometry.intensity.min_size_filter = true;
        params.intensity.filters.push(Default::default());
        params.intensity.filters[0].symbol = crate::map::AreaSymbol::DarkGreen;
        assert!(
            params
                .min_size_filter_symbols(true, true, true, true, true, true)
                .contains(&crate::map::AreaSymbol::DarkGreen)
        );
        params.intensity.enabled = false;
        assert!(
            !params
                .min_size_filter_symbols(true, true, true, true, true, true)
                .contains(&crate::map::AreaSymbol::DarkGreen)
        );
    }
}
