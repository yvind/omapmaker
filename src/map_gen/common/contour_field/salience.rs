use super::quadratic::TerrainDerivatives;
use crate::parameters::ContourFieldParameters;
use crate::raster::Dfm;
use crate::raster::dfm::{ContourCost, SmoothnessWeight, TerrainSalience};

pub(super) struct SalienceWeights {
    pub(super) salience: Dfm<TerrainSalience>,
    pub(super) contour_cost: Dfm<ContourCost>,
    pub(super) smoothness: Dfm<SmoothnessWeight>,
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
    for index in 0..salience.field.len() {
        let transform = |value: f32, reference: f32| value / (value + reference);
        let confidence =
            derivatives.direction_confidence.field[index] * derivatives.fit_confidence.field[index];
        let importance = (params.slope_weight
            * transform(derivatives.slope.field[index], params.slope_reference)
            + params.profile_change_weight
                * confidence
                * transform(
                    derivatives.profile_change.field[index],
                    params.profile_change_reference,
                )
            + params.tangent_change_weight
                * confidence
                * transform(
                    derivatives.tangent_change.field[index],
                    params.tangent_change_reference,
                ))
            / weight_sum;
        let importance = importance.clamp(0., 1.);
        salience.field[index] = importance;
        contour_cost.field[index] = params.minimum_contour_cost
            + (1. - params.minimum_contour_cost) * (1. - importance).powf(params.salience_power);
        smoothness.field[index] = params.minimum_smoothness_weight
            + params.smoothness_scale * (1. - importance).powf(params.smoothness_power);
    }
    Ok(SalienceWeights {
        salience,
        contour_cost,
        smoothness,
    })
}
