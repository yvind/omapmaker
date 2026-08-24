use super::quadratic::TerrainDerivatives;
use crate::parameters::ContourFieldParameters;
use crate::raster::Dfm;
use crate::raster::dfm::{AlignmentConfidence, ContourCost, SmoothnessWeight, TerrainSalience};

pub(super) struct SalienceWeights {
    pub(super) salience: Dfm<TerrainSalience>,
    pub(super) contour_cost: Dfm<ContourCost>,
    pub(super) smoothness: Dfm<SmoothnessWeight>,
    pub(super) alignment_confidence: Dfm<AlignmentConfidence>,
}

pub(super) fn calculate(
    derivatives: &TerrainDerivatives,
    params: &ContourFieldParameters,
) -> crate::Result<SalienceWeights> {
    let weight_sum =
        params.slope_weight + params.profile_change_weight + params.tangent_change_weight;
    anyhow::ensure!(
        weight_sum > 0.,
        "at least one terrain-salience weight must be enabled"
    );
    let mut salience = Dfm::new_like(&derivatives.slope);
    let mut contour_cost = Dfm::new_like(&derivatives.slope);
    let mut smoothness = Dfm::new_like(&derivatives.slope);
    let mut alignment_confidence = Dfm::new_like(&derivatives.slope);
    for index in 0..salience.field.len() {
        let transform = |value: f32, reference: f32| value / (value + reference);
        let direction_confidence = derivatives.direction_confidence.field[index];
        let change_confidence = direction_confidence * derivatives.fit_confidence.field[index];
        let slope_importance = transform(derivatives.slope.field[index], params.slope_reference);
        let change_weight = params.profile_change_weight + params.tangent_change_weight;
        let change_importance = if change_weight > 0. {
            change_confidence
                * (params.profile_change_weight
                    * transform(
                        derivatives.profile_change.field[index],
                        params.profile_change_reference,
                    )
                    + params.tangent_change_weight
                        * transform(
                            derivatives.tangent_change.field[index],
                            params.tangent_change_reference,
                        ))
                / change_weight
        } else {
            0.
        };
        let combined_importance = (params.slope_weight * slope_importance
            + params.profile_change_weight
                * change_confidence
                * transform(
                    derivatives.profile_change.field[index],
                    params.profile_change_reference,
                )
            + params.tangent_change_weight
                * change_confidence
                * transform(
                    derivatives.tangent_change.field[index],
                    params.tangent_change_reference,
                ))
            / weight_sum;
        // A weighted average alone dilutes coherent slope changes whenever the
        // absolute slope is modest. Keep a separate change channel so broad
        // breaks remain competitive with steep-slope attraction.
        let importance = combined_importance.max(change_importance).clamp(0., 1.);
        salience.field[index] = importance;
        contour_cost.field[index] = params.minimum_contour_cost
            + (1. - params.minimum_contour_cost) * (1. - importance).powf(params.salience_power);
        smoothness.field[index] = params.minimum_smoothness_weight
            + params.smoothness_scale * (1. - importance).powf(params.smoothness_power);
        alignment_confidence.field[index] = direction_confidence;
    }
    Ok(SalienceWeights {
        salience,
        contour_cost,
        smoothness,
        alignment_confidence,
    })
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::raster::DfmGrid;
    use crate::raster::dfm::{
        DirectionConfidence, Elevation, FitConfidence, IsolineTangentX, IsolineTangentY,
        ProfileChange, Slope, TangentChange,
    };

    #[test]
    fn strong_slope_and_profile_change_create_attractive_bands() {
        let grid = DfmGrid::new(3, 3, 0.5, geo::coord! { x: 0., y: 1. }).unwrap();
        let source = Dfm::<Elevation>::new(grid);
        let mut derivatives = TerrainDerivatives {
            slope: Dfm::<Slope>::new_like(&source),
            profile_change: Dfm::<ProfileChange>::new_like(&source),
            tangent_change: Dfm::<TangentChange>::new_like(&source),
            direction_confidence: Dfm::<DirectionConfidence>::new_like(&source),
            fit_confidence: Dfm::<FitConfidence>::new_like(&source),
            isoline_tangent_x: Dfm::<IsolineTangentX>::new_like(&source),
            isoline_tangent_y: Dfm::<IsolineTangentY>::new_like(&source),
        };
        derivatives.slope.field.fill(0.);
        derivatives.profile_change.field.fill(0.);
        derivatives.tangent_change.field.fill(0.);
        derivatives.direction_confidence.field.fill(1.);
        derivatives.fit_confidence.field.fill(1.);
        derivatives.isoline_tangent_x.field.fill(0.);
        derivatives.isoline_tangent_y.field.fill(-1.);
        derivatives.slope.field[1] = 1.;
        derivatives.profile_change.field[2] = 1.;
        derivatives.profile_change.field[3] = 1.;
        derivatives.fit_confidence.field[3] = 0.;

        let weights = calculate(&derivatives, &ContourFieldParameters::default()).unwrap();

        assert_eq!(weights.contour_cost.field[0], 1.);
        assert!(weights.contour_cost.field[1] < 0.2);
        assert!(weights.contour_cost.field[2] < 0.2);
        assert_eq!(weights.contour_cost.field[3], 1.);
        assert!(weights.smoothness.field[1] < 0.2);
        assert!(weights.smoothness.field[2] < 0.2);
        assert!(weights.salience.field[2] > weights.salience.field[1]);
        assert_eq!(weights.salience.field[3], weights.salience.field[0]);
        assert_eq!(weights.alignment_confidence.field[0], 1.);
        assert_eq!(weights.alignment_confidence.field[3], 1.);
    }
}
