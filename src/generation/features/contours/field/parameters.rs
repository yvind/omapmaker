use crate::parameters::ContourFieldParameters;

#[derive(Clone, Copy)]
pub(super) struct ValidatedParameters<'a> {
    pub(super) inner: &'a ContourFieldParameters,
}

impl<'a> ValidatedParameters<'a> {
    pub(super) fn new(inner: &'a ContourFieldParameters, interval: f32) -> crate::Result<Self> {
        anyhow::ensure!(
            interval.is_finite() && interval > 0.,
            "contour interval must be positive"
        );
        anyhow::ensure!(
            inner.max_iterations > 0,
            "contour optimizer needs at least one iteration"
        );
        anyhow::ensure!(
            inner.convergence_tolerance.is_finite() && inner.convergence_tolerance > 0.,
            "contour convergence tolerance must be positive"
        );
        for (name, value) in [
            ("fidelity weight", inner.fidelity_weight),
            ("weighted-TV weight", inner.weighted_tv_weight),
            ("Hessian weight", inner.hessian_weight),
            ("minimum contour cost", inner.minimum_contour_cost),
            ("minimum smoothness weight", inner.minimum_smoothness_weight),
            ("smoothness scale", inner.smoothness_scale),
            ("slope epsilon", inner.slope_epsilon),
            ("RMSE reference", inner.rmse_reference),
        ] {
            anyhow::ensure!(value.is_finite() && value > 0., "{name} must be positive");
        }
        anyhow::ensure!(
            inner.alignment_weight.is_finite() && inner.alignment_weight >= 0.,
            "isoline-alignment weight must be nonnegative"
        );
        anyhow::ensure!(
            inner.minimum_contour_cost <= 1.,
            "minimum contour cost must not exceed one"
        );
        for (name, value) in [
            ("salience power", inner.salience_power),
            ("smoothness power", inner.smoothness_power),
        ] {
            anyhow::ensure!(value.is_finite() && value > 0., "{name} must be positive");
        }
        for (name, value) in [
            ("slope weight", inner.slope_weight),
            ("profile-change weight", inner.profile_change_weight),
            ("tangent-change weight", inner.tangent_change_weight),
        ] {
            anyhow::ensure!(
                value.is_finite() && value >= 0.,
                "{name} must be nonnegative"
            );
        }
        for (name, value) in [
            ("slope reference", inner.slope_reference),
            ("profile-change reference", inner.profile_change_reference),
            ("tangent-change reference", inner.tangent_change_reference),
        ] {
            anyhow::ensure!(value.is_finite() && value > 0., "{name} must be positive");
        }
        anyhow::ensure!(
            inner.persistence_threshold_fraction.is_finite()
                && (0. ..=0.5).contains(&inner.persistence_threshold_fraction),
            "persistence threshold fraction must be between 0 and 0.5"
        );
        anyhow::ensure!(
            inner
                .multiresolution_levels_m
                .iter()
                .all(|level| level.is_finite() && *level > 0.),
            "multiresolution cell sizes must be positive and finite"
        );
        anyhow::ensure!(
            inner.slope_fit_radius_m > 0.
                && inner.curvature_fit_radius_m > 0.
                && inner.solver_guard_distance_m >= 0.,
            "physical contour-field radii must be valid"
        );
        Ok(Self { inner })
    }
}
