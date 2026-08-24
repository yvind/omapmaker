mod operators;
mod parameters;
mod persistence;
mod quadratic;
mod salience;
mod solver;

use self::parameters::ValidatedParameters;
use crate::STANDARD_CELL_SIZE_METERS;
use crate::parameters::ContourFieldParameters;
use crate::raster::Dfm;
use crate::raster::dfm::{
    AdjustedElevation, AdjustmentBoundMask, ContourCost, DirectionConfidence, Elevation,
    FitConfidence, ProfileChange, Slope, SmoothnessWeight, TangentChange, TargetElevation,
    TerrainSalience, VerticalAdjustment,
};

use std::time::{Duration, Instant};

#[derive(Clone, Debug)]
pub struct ContourFieldDiagnostics {
    pub iterations: usize,
    pub runtime: Duration,
    pub fidelity_energy: f64,
    pub weighted_tv_energy: f64,
    pub hessian_energy: f64,
    pub maximum_adjustment: f32,
    pub rms_adjustment: f32,
    pub fraction_at_bound: f32,
    pub persistence_pairs_removed: usize,
    pub persistence_pairs_preserved: usize,
    pub protected_extrema: Vec<geo::Coord>,
    #[allow(dead_code)]
    pub debug_rasters: Option<Box<ContourFieldDebugRasters>>,
}

#[allow(dead_code)]
#[derive(Clone, Debug)]
pub struct ContourFieldDebugRasters {
    pub f_target: Dfm<TargetElevation>,
    pub adjusted_field: Dfm<AdjustedElevation>,
    pub vertical_adjustment: Dfm<VerticalAdjustment>,
    pub slope: Dfm<Slope>,
    pub profile_change: Dfm<ProfileChange>,
    pub tangent_change: Dfm<TangentChange>,
    pub direction_confidence: Dfm<DirectionConfidence>,
    pub fit_confidence: Dfm<FitConfidence>,
    pub salience: Dfm<TerrainSalience>,
    pub contour_cost: Dfm<ContourCost>,
    pub smoothness_weight: Dfm<SmoothnessWeight>,
    pub at_adjustment_bound: Dfm<AdjustmentBoundMask>,
    pub removed_extrema: Vec<geo::Coord>,
    pub preserved_extrema: Vec<geo::Coord>,
}

pub fn optimize_contour_field(
    source: &Dfm<Elevation>,
    regular_interval: f32,
    params: &ContourFieldParameters,
) -> crate::Result<(Dfm<AdjustedElevation>, ContourFieldDiagnostics)> {
    let params = ValidatedParameters::new(params, regular_interval)?.inner;
    anyhow::ensure!(
        (source.grid.cell_size_m - STANDARD_CELL_SIZE_METERS).abs() <= 1e-9,
        "final contour optimization requires the canonical {STANDARD_CELL_SIZE_METERS} m grid"
    );
    anyhow::ensure!(
        source
            .field
            .iter()
            .all(|value| value.is_finite() && *value != f32::MIN),
        "contour optimization requires a finite elevation in every source cell"
    );
    let started = Instant::now();
    let adjustment_bound = regular_interval * 0.25;
    let persistence_threshold = params.persistence_threshold_fraction * regular_interval;
    let (target, mut persistence) =
        persistence::simplify_bounded(source, source, persistence_threshold, adjustment_bound);
    let derivatives = quadratic::calculate(source, regular_interval, params)?;
    let weights = salience::calculate(&derivatives, params)?;

    // All level grids share the same cell-edge extent. Salience and topology
    // are interpreted once on the canonical grid, then restricted.
    let mut levels = Vec::new();
    for (index, &cell_size) in params.multiresolution_levels_m.iter().enumerate() {
        if cell_size + 1e-9 < source.grid.cell_size_m {
            continue;
        }
        let grid = if (cell_size - source.grid.cell_size_m).abs() <= 1e-9 {
            Some(source.grid.clone())
        } else {
            source.grid.aligned_coarsened(cell_size).ok()
        };
        if let Some(grid) = grid {
            let iterations = params
                .iterations_per_level
                .get(index)
                .copied()
                .unwrap_or(params.max_iterations);
            levels.push((grid, iterations));
        }
    }
    if !levels
        .iter()
        .any(|(grid, _)| grid.same_layout(&source.grid))
    {
        levels.push((source.grid.clone(), params.max_iterations));
    }
    levels.sort_by(|a, b| b.0.cell_size_m.total_cmp(&a.0.cell_size_m));
    levels.dedup_by(|a, b| (a.0.cell_size_m - b.0.cell_size_m).abs() <= 1e-9);
    anyhow::ensure!(
        params.max_iterations >= levels.len(),
        "maximum contour iterations must cover every enabled solver level"
    );
    let configured_total = levels
        .iter()
        .map(|(_, iterations)| *iterations)
        .sum::<usize>();
    let mut budget = params.max_iterations;
    let level_count = levels.len();
    for (index, (_, iterations)) in levels.iter_mut().enumerate() {
        let levels_after = level_count - index - 1;
        *iterations = if levels_after == 0 {
            budget
        } else {
            ((*iterations * params.max_iterations / configured_total.max(1)).max(1))
                .min(budget.saturating_sub(levels_after))
        };
        budget = budget.saturating_sub(*iterations);
    }

    let mut current = None::<Dfm<AdjustedElevation>>;
    let mut total_iterations = 0;
    let mut final_solver = solver::SolverDiagnostics::default();
    for (grid, configured_iterations) in levels {
        let level_source = if grid.same_layout(&source.grid) {
            source.clone()
        } else {
            source.restrict_to(&grid)?
        };
        let level_target = if grid.same_layout(&source.grid) {
            target.clone()
        } else {
            target.restrict_to(&grid)?
        };
        let level_cost = if grid.same_layout(&source.grid) {
            weights.contour_cost.clone()
        } else {
            weights.contour_cost.restrict_to(&grid)?
        };
        let level_smoothness = if grid.same_layout(&source.grid) {
            weights.smoothness.clone()
        } else {
            weights.smoothness.restrict_to(&grid)?
        };
        let mut initial = current.as_ref().map(|previous| {
            if previous.grid.same_layout(&grid) {
                previous.clone()
            } else {
                previous
                    .prolong_to(&grid)
                    .expect("validated adjacent solver grids")
            }
        });
        if let Some(initial) = initial.as_mut() {
            for (value, source_value) in initial.field.iter_mut().zip(&level_source.field) {
                *value = value.clamp(
                    *source_value - adjustment_bound,
                    *source_value + adjustment_bound,
                );
            }
        }
        let (field, diagnostics) = solver::solve(
            solver::SolverRasters {
                source: &level_source,
                target: &level_target,
                contour_cost: &level_cost,
                smoothness: &level_smoothness,
            },
            initial.as_ref().map(|raster| raster.field.as_ref()),
            regular_interval,
            configured_iterations,
            params,
        );
        let mut adjusted = Dfm::new(grid);
        adjusted.field = field.into_boxed_slice();
        total_iterations += diagnostics.iterations;
        final_solver = diagnostics;
        current = Some(adjusted);
    }

    let mut adjusted = current.expect("at least the canonical solver level is present");
    if !adjusted.grid.same_layout(&source.grid) {
        adjusted = adjusted.prolong_to(&source.grid)?;
    }
    for (value, source_value) in adjusted.field.iter_mut().zip(&source.field) {
        *value = value.clamp(
            *source_value - adjustment_bound,
            *source_value + adjustment_bound,
        );
    }
    let mut audit_input = Dfm::<Elevation>::new_like(source);
    audit_input.field.copy_from_slice(&adjusted.field);
    let (audited, audit) = persistence::simplify_bounded(
        source,
        &audit_input,
        persistence_threshold,
        adjustment_bound,
    );
    adjusted.field.copy_from_slice(&audited.field);
    persistence.removed += audit.removed;
    persistence.preserved += audit.preserved;
    persistence
        .removed_extrema
        .extend(audit.removed_extrema.iter().copied());
    persistence
        .preserved_extrema
        .extend(audit.preserved_extrema.iter().copied());

    let to_coordinates = |indices: &[usize]| {
        let mut indices = indices.to_vec();
        indices.sort_unstable();
        indices.dedup();
        indices
            .into_iter()
            .map(|index| source.index2coord(index / source.width(), index % source.width()))
            .collect::<Vec<_>>()
    };
    let protected_extrema = to_coordinates(&audit.preserved_extrema);

    let mut maximum_adjustment = 0_f32;
    let mut squared_adjustment = 0_f64;
    let mut at_bound = 0;
    for (&value, &original) in adjusted.field.iter().zip(&source.field) {
        anyhow::ensure!(
            value.is_finite(),
            "contour optimizer produced a non-finite value"
        );
        let adjustment = (value - original).abs();
        anyhow::ensure!(
            adjustment <= adjustment_bound + 1e-5,
            "contour optimizer exceeded its elevation bound"
        );
        maximum_adjustment = maximum_adjustment.max(adjustment);
        squared_adjustment += f64::from(adjustment).powi(2);
        at_bound += usize::from(adjustment >= adjustment_bound - 1e-5);
    }
    let debug_rasters = params.collect_debug_rasters.then(|| {
        let mut vertical_adjustment = Dfm::new_like(source);
        let mut at_adjustment_bound = Dfm::new_like(source);
        for i in 0..adjusted.field.len() {
            vertical_adjustment.field[i] = adjusted.field[i] - source.field[i];
            at_adjustment_bound.field[i] =
                if vertical_adjustment.field[i].abs() >= adjustment_bound - 1e-5 {
                    1.
                } else {
                    0.
                };
        }
        Box::new(ContourFieldDebugRasters {
            f_target: target,
            adjusted_field: adjusted.clone(),
            vertical_adjustment,
            slope: derivatives.slope,
            profile_change: derivatives.profile_change,
            tangent_change: derivatives.tangent_change,
            direction_confidence: derivatives.direction_confidence,
            fit_confidence: derivatives.fit_confidence,
            salience: weights.salience,
            contour_cost: weights.contour_cost,
            smoothness_weight: weights.smoothness,
            at_adjustment_bound,
            removed_extrema: to_coordinates(&persistence.removed_extrema),
            preserved_extrema: to_coordinates(&persistence.preserved_extrema),
        })
    });
    let diagnostics = ContourFieldDiagnostics {
        iterations: total_iterations,
        runtime: started.elapsed(),
        fidelity_energy: final_solver.fidelity_energy,
        weighted_tv_energy: final_solver.weighted_tv_energy,
        hessian_energy: final_solver.hessian_energy,
        maximum_adjustment,
        rms_adjustment: (squared_adjustment / adjusted.field.len() as f64).sqrt() as f32,
        fraction_at_bound: at_bound as f32 / adjusted.field.len() as f32,
        persistence_pairs_removed: persistence.removed,
        persistence_pairs_preserved: persistence.preserved,
        protected_extrema,
        debug_rasters,
    };
    Ok((adjusted, diagnostics))
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::raster::DfmGrid;

    #[test]
    fn multiresolution_plane_is_deterministic_and_bounded() {
        let grid = DfmGrid::new(16, 16, 0.5, geo::coord! { x: 0.25, y: 7.75 }).unwrap();
        let mut source = Dfm::<Elevation>::new(grid);
        for y in 0..source.height() {
            for x in 0..source.width() {
                let point = source.index2coord(y, x);
                source[(y, x)] = (10. + 0.3 * point.x + 0.15 * point.y) as f32;
            }
        }
        let mut params = ContourFieldParameters {
            max_iterations: 15,
            iterations_per_level: vec![5, 5, 5],
            slope_fit_radius_m: 1.5,
            curvature_fit_radius_m: 2.,
            solver_guard_distance_m: 1.,
            collect_debug_rasters: true,
            ..Default::default()
        };
        let (first, diagnostics) = optimize_contour_field(&source, 1., &params).unwrap();
        params.collect_debug_rasters = false;
        let (second, _) = optimize_contour_field(&source, 1., &params).unwrap();
        assert_eq!(first.field, second.field);
        assert!(
            first
                .field
                .iter()
                .zip(&source.field)
                .all(|(adjusted, original)| (adjusted - original).abs() <= 0.25 + 1e-6)
        );
        for line in first.marching_squares(12.).iter() {
            for &coordinate in &line.0 {
                if coordinate.x > source.grid.top_left.x
                    && coordinate.x < source.index2coord(0, source.width() - 1).x
                    && coordinate.y < source.grid.top_left.y
                    && coordinate.y > source.index2coord(source.height() - 1, 0).y
                    && let Some(original) = source.sample_bilinear(coordinate)
                {
                    assert!(
                        (original - 12.).abs() <= 0.251,
                        "{coordinate:?}: original={original}"
                    );
                }
            }
        }
        assert!(diagnostics.debug_rasters.is_some());
    }

    #[test]
    fn flat_field_stays_finite_and_constant() {
        let grid = DfmGrid::new(8, 8, 0.5, geo::coord! { x: 0.25, y: 3.75 }).unwrap();
        let mut source = Dfm::<Elevation>::new(grid);
        source.field.fill(100.);
        let params = ContourFieldParameters {
            max_iterations: 6,
            iterations_per_level: vec![2, 2, 2],
            slope_fit_radius_m: 1.,
            curvature_fit_radius_m: 1.,
            solver_guard_distance_m: 0.5,
            ..Default::default()
        };
        let (adjusted, _) = optimize_contour_field(&source, 5., &params).unwrap();
        assert!(
            adjusted
                .field
                .iter()
                .all(|value| value.is_finite() && (*value - 100.).abs() < 1e-4)
        );
    }
}
