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
    AdjustedElevation, AdjustmentBoundMask, AlignmentConfidence, ContourCost, DirectionConfidence,
    Elevation, FitConfidence, IsolineTangentX, IsolineTangentY, ProfileChange, RasterMarker, Slope,
    SmoothnessWeight, TangentChange, TargetElevation, TerrainSalience, VerticalAdjustment,
};

use std::time::{Duration, Instant};

pub(crate) use quadratic::FittedTerrain;

pub(super) const MAX_VERTICAL_ADJUSTMENT_FRACTION: f32 = 0.25;

pub(super) fn adjustment_bound(regular_interval: f32) -> f32 {
    regular_interval.abs() * MAX_VERTICAL_ADJUSTMENT_FRACTION
}

#[repr(u8)]
#[derive(Clone, Copy, Debug, Eq, Hash, Ord, PartialEq, PartialOrd)]
pub enum ExtremumKind {
    Minimum,
    Maximum,
}

#[derive(Clone, Debug, PartialEq)]
pub struct ProtectedPersistenceFeature {
    pub pair_id: u64,
    pub kind: ExtremumKind,
    pub(crate) extremum_index: usize,
    pub extremum: geo::Coord,
    pub extremum_elevation: f32,
    pub saddle_elevation: f32,
    pub persistence: f32,
}

#[derive(Clone, Debug)]
pub struct ContourFieldDiagnostics {
    pub solver: ContourSolverDiagnostics,
    pub published: PublishedFieldDiagnostics,
    pub persistence: PersistenceDiagnostics,
    pub timings: ContourFieldStageTimings,
    pub protected_features: Vec<ProtectedPersistenceFeature>,
    #[allow(dead_code)]
    pub debug_rasters: Option<Box<ContourFieldDebugRasters>>,
}

#[derive(Clone, Debug)]
pub struct ContourSolverDiagnostics {
    pub iterations: usize,
}

#[derive(Clone, Debug, Default)]
pub struct ContourFieldStageTimings {
    pub total: Duration,
    pub target_persistence: Duration,
    pub derivatives: Duration,
    pub salience: Duration,
    pub levels: Vec<SolverLevelTiming>,
    pub publication_persistence: Duration,
    pub published_diagnostics: Duration,
}

#[derive(Clone, Debug, Default)]
pub struct SolverLevelTiming {
    pub cell_size_m: f64,
    pub transfer: Duration,
    pub operator_norm: Duration,
    pub solve: Duration,
    pub iterations: usize,
}

#[derive(Clone, Copy, Debug, Default, Eq, PartialEq)]
pub struct PersistenceWork {
    pub diagram_builds: usize,
    pub cancellation_passes: usize,
    pub candidates_considered: usize,
    pub cancellations_applied: usize,
    pub affected_cells_written: usize,
}

impl std::ops::AddAssign for PersistenceWork {
    fn add_assign(&mut self, other: Self) {
        self.diagram_builds += other.diagram_builds;
        self.cancellation_passes += other.cancellation_passes;
        self.candidates_considered += other.candidates_considered;
        self.cancellations_applied += other.cancellations_applied;
        self.affected_cells_written += other.affected_cells_written;
    }
}

#[derive(Clone, Debug)]
pub struct PublishedFieldDiagnostics {
    pub fidelity_energy: f64,
    pub weighted_tv_energy: f64,
    pub alignment_energy: f64,
    pub hessian_energy: f64,
    pub maximum_adjustment: f32,
    pub rms_adjustment: f32,
    pub fraction_at_bound: f32,
}

#[derive(Clone, Debug)]
pub struct PersistenceDiagnostics {
    pub requested: usize,
    pub verified_removed: usize,
    pub preserved: usize,
    pub unresolved: usize,
    pub target_work: PersistenceWork,
    pub publication_work: PersistenceWork,
}

enum LevelRaster<'a, T: RasterMarker> {
    Borrowed(&'a Dfm<T>),
    Owned(Dfm<T>),
}

impl<T: RasterMarker> LevelRaster<'_, T> {
    fn as_ref(&self) -> &Dfm<T> {
        match self {
            Self::Borrowed(raster) => raster,
            Self::Owned(raster) => raster,
        }
    }
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
    pub isoline_tangent_x: Dfm<IsolineTangentX>,
    pub isoline_tangent_y: Dfm<IsolineTangentY>,
    pub alignment_confidence: Dfm<AlignmentConfidence>,
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
    optimize_contour_field_impl(source, regular_interval, params, None)
}

pub(crate) fn fit_terrain(
    source: &Dfm<Elevation>,
    params: &ContourFieldParameters,
) -> crate::Result<FittedTerrain> {
    quadratic::fit(source, params)
}

pub(crate) fn optimize_contour_field_with_fitted(
    source: &Dfm<Elevation>,
    regular_interval: f32,
    params: &ContourFieldParameters,
    fitted: &FittedTerrain,
) -> crate::Result<(Dfm<AdjustedElevation>, ContourFieldDiagnostics)> {
    optimize_contour_field_impl(source, regular_interval, params, Some(fitted))
}

fn optimize_contour_field_impl(
    source: &Dfm<Elevation>,
    regular_interval: f32,
    params: &ContourFieldParameters,
    fitted: Option<&FittedTerrain>,
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
    validate_solver_halo(source, params);
    let started = Instant::now();
    let mut timings = ContourFieldStageTimings::default();
    let adjustment_bound = adjustment_bound(regular_interval);
    let persistence_threshold = params.persistence_threshold_fraction * regular_interval;
    let mut persistence_workspace = persistence::PersistenceWorkspace::default();
    let stage_started = Instant::now();
    let (target, target_persistence) = persistence::simplify_bounded(
        source,
        source,
        persistence_threshold,
        adjustment_bound,
        &mut persistence_workspace,
    );
    timings.target_persistence = stage_started.elapsed();
    let stage_started = Instant::now();
    let fitted_owned;
    let fitted = if let Some(fitted) = fitted {
        fitted
    } else {
        fitted_owned = quadratic::fit(source, params)?;
        &fitted_owned
    };
    let derivatives = quadratic::calculate_from_fitted(source, fitted, regular_interval, params)?;
    timings.derivatives = stage_started.elapsed();
    let stage_started = Instant::now();
    let weights = salience::calculate(&derivatives, params)?;
    timings.salience = stage_started.elapsed();

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
    let mut solver_workspace = solver::SolverWorkspace::default();
    for (grid, configured_iterations) in levels {
        let transfer_started = Instant::now();
        let level_source = if grid.same_layout(&source.grid) {
            LevelRaster::Borrowed(source)
        } else {
            LevelRaster::Owned(source.restrict_to(&grid)?)
        };
        let level_target = if grid.same_layout(&source.grid) {
            LevelRaster::Borrowed(&target)
        } else {
            LevelRaster::Owned(target.restrict_to(&grid)?)
        };
        let level_cost = if grid.same_layout(&source.grid) {
            LevelRaster::Borrowed(&weights.contour_cost)
        } else {
            LevelRaster::Owned(weights.contour_cost.restrict_to(&grid)?)
        };
        let level_smoothness = if grid.same_layout(&source.grid) {
            LevelRaster::Borrowed(&weights.smoothness)
        } else {
            LevelRaster::Owned(weights.smoothness.restrict_to(&grid)?)
        };
        let level_tangent_x = if grid.same_layout(&source.grid) {
            LevelRaster::Borrowed(&derivatives.isoline_tangent_x)
        } else {
            LevelRaster::Owned(derivatives.isoline_tangent_x.restrict_to(&grid)?)
        };
        let level_tangent_y = if grid.same_layout(&source.grid) {
            LevelRaster::Borrowed(&derivatives.isoline_tangent_y)
        } else {
            LevelRaster::Owned(derivatives.isoline_tangent_y.restrict_to(&grid)?)
        };
        let level_alignment_confidence = if grid.same_layout(&source.grid) {
            LevelRaster::Borrowed(&weights.alignment_confidence)
        } else {
            LevelRaster::Owned(weights.alignment_confidence.restrict_to(&grid)?)
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
            for (value, source_value) in initial.field.iter_mut().zip(&level_source.as_ref().field)
            {
                *value = value.clamp(
                    *source_value - adjustment_bound,
                    *source_value + adjustment_bound,
                );
            }
        }
        let transfer = transfer_started.elapsed();
        let level_source = level_source.as_ref();
        let level_target = level_target.as_ref();
        let level_cost = level_cost.as_ref();
        let level_smoothness = level_smoothness.as_ref();
        let level_tangent_x = level_tangent_x.as_ref();
        let level_tangent_y = level_tangent_y.as_ref();
        let level_alignment_confidence = level_alignment_confidence.as_ref();
        let norm_started = Instant::now();
        let operator_norm = operators::norm(
            level_source.width(),
            level_source.height(),
            level_source.grid.cell_size_m as f32,
        );
        let operator_norm_timing = norm_started.elapsed();
        let solve_started = Instant::now();
        let (field, diagnostics) = solver::solve(
            solver::SolverRasters {
                source: level_source,
                target: level_target,
                contour_cost: level_cost,
                smoothness: level_smoothness,
                isoline_tangent_x: level_tangent_x,
                isoline_tangent_y: level_tangent_y,
                alignment_confidence: level_alignment_confidence,
            },
            initial.as_ref().map(|raster| raster.field.as_ref()),
            regular_interval,
            configured_iterations,
            operator_norm,
            params,
            &mut solver_workspace,
        );
        let solve_timing = solve_started.elapsed();
        let mut adjusted = Dfm::new(grid);
        adjusted.field = field.into_boxed_slice();
        total_iterations += diagnostics.iterations;
        timings.levels.push(SolverLevelTiming {
            cell_size_m: adjusted.grid.cell_size_m,
            transfer,
            operator_norm: operator_norm_timing,
            solve: solve_timing,
            iterations: diagnostics.iterations,
        });
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
    let stage_started = Instant::now();
    let (audited, audit) = persistence::simplify_bounded(
        source,
        &adjusted,
        persistence_threshold,
        adjustment_bound,
        &mut persistence_workspace,
    );
    adjusted.field.copy_from_slice(&audited.field);
    timings.publication_persistence = stage_started.elapsed();

    let stage_started = Instant::now();
    let to_coordinates = |indices: &[usize]| {
        let mut indices = indices.to_vec();
        indices.sort_unstable();
        indices.dedup();
        indices
            .into_iter()
            .map(|index| source.index2coord(index / source.width(), index % source.width()))
            .collect::<Vec<_>>()
    };

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
    let published_energies = solver::evaluate(
        &adjusted.field,
        solver::SolverRasters {
            source,
            target: &target,
            contour_cost: &weights.contour_cost,
            smoothness: &weights.smoothness,
            isoline_tangent_x: &derivatives.isoline_tangent_x,
            isoline_tangent_y: &derivatives.isoline_tangent_y,
            alignment_confidence: &weights.alignment_confidence,
        },
        params,
        &mut solver_workspace,
    );
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
            isoline_tangent_x: derivatives.isoline_tangent_x,
            isoline_tangent_y: derivatives.isoline_tangent_y,
            alignment_confidence: weights.alignment_confidence,
            salience: weights.salience,
            contour_cost: weights.contour_cost,
            smoothness_weight: weights.smoothness,
            at_adjustment_bound,
            removed_extrema: to_coordinates(&audit.removed_extrema),
            preserved_extrema: audit
                .protected_features
                .iter()
                .map(|feature| feature.extremum)
                .collect(),
        })
    });
    let diagnostics = ContourFieldDiagnostics {
        solver: ContourSolverDiagnostics {
            iterations: total_iterations,
        },
        published: PublishedFieldDiagnostics {
            fidelity_energy: published_energies.fidelity,
            weighted_tv_energy: published_energies.weighted_tv,
            alignment_energy: published_energies.alignment,
            hessian_energy: published_energies.hessian,
            maximum_adjustment,
            rms_adjustment: (squared_adjustment / adjusted.field.len() as f64).sqrt() as f32,
            fraction_at_bound: at_bound as f32 / adjusted.field.len() as f32,
        },
        persistence: PersistenceDiagnostics {
            requested: audit.requested,
            verified_removed: audit.removed,
            preserved: audit.preserved,
            unresolved: audit.unresolved,
            target_work: target_persistence.work,
            publication_work: audit.work,
        },
        timings: {
            timings.published_diagnostics = stage_started.elapsed();
            timings.total = started.elapsed();
            timings
        },
        protected_features: audit.protected_features,
        debug_rasters,
    };
    Ok((adjusted, diagnostics))
}

fn validate_solver_halo(source: &Dfm<Elevation>, params: &ContourFieldParameters) {
    if source.grid.inner.is_empty() {
        return;
    }
    let inner = source.grid.inner;
    let halo_cells = inner
        .top
        .min(inner.left)
        .min(source.height().saturating_sub(inner.bottom))
        .min(source.width().saturating_sub(inner.right));
    let halo_m = halo_cells as f64 * source.grid.cell_size_m;
    let required_m = params.slope_fit_radius_m.max(params.curvature_fit_radius_m)
        + params.solver_guard_distance_m;
    if halo_m + 1e-9 < required_m {
        log::warn!(
            "contour optimizer halo is {halo_m:.2} m but derivative fitting and the solver guard \
             require {required_m:.2} m; tile-edge contours may differ from a supertile result"
        );
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::raster::{DfmGrid, dfm::RasterMarker};

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
        let fitted = fit_terrain(&source, &params).unwrap();
        let (second, _) =
            optimize_contour_field_with_fitted(&source, 1., &params, &fitted).unwrap();
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
        let debug = diagnostics.debug_rasters.as_ref().unwrap();
        let independently_computed = solver::evaluate(
            &first.field,
            solver::SolverRasters {
                source: &source,
                target: &debug.f_target,
                contour_cost: &debug.contour_cost,
                smoothness: &debug.smoothness_weight,
                isoline_tangent_x: &debug.isoline_tangent_x,
                isoline_tangent_y: &debug.isoline_tangent_y,
                alignment_confidence: &debug.alignment_confidence,
            },
            &params,
            &mut solver::SolverWorkspace::default(),
        );
        assert!(
            (diagnostics.published.fidelity_energy - independently_computed.fidelity).abs() < 1e-9
        );
        assert!(
            (diagnostics.published.weighted_tv_energy - independently_computed.weighted_tv).abs()
                < 1e-9
        );
        assert!(
            (diagnostics.published.alignment_energy - independently_computed.alignment).abs()
                < 1e-9
        );
        assert!(
            (diagnostics.published.hessian_energy - independently_computed.hessian).abs() < 1e-9
        );
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

    #[test]
    fn slope_break_attracts_a_nearby_isoline() {
        let grid = DfmGrid::new(64, 32, 0.5, geo::coord! { x: 0.25, y: 15.75 }).unwrap();
        let mut source = Dfm::<Elevation>::new(grid);
        let break_x = 12.;
        for y in 0..source.height() {
            for x in 0..source.width() {
                let distance = x as f32 * 0.5;
                source[(y, x)] = if distance <= break_x {
                    0.1 * distance
                } else if distance <= break_x + 2. {
                    1.2 + distance - break_x
                } else {
                    3.2 + 0.1 * (distance - break_x - 2.)
                };
            }
        }
        let crossing = |field: &[f32]| {
            let row = source.height() / 2;
            let row = &field[row * source.width()..(row + 1) * source.width()];
            row.windows(2)
                .enumerate()
                .find_map(|(x, values)| {
                    (values[0] <= 1. && values[1] >= 1.).then(|| {
                        (x as f32 + (1. - values[0]) / (values[1] - values[0]).max(f32::EPSILON))
                            * 0.5
                    })
                })
                .unwrap()
        };
        let params = ContourFieldParameters {
            max_iterations: 160,
            iterations_per_level: vec![40, 50, 70],
            slope_fit_radius_m: 1.5,
            curvature_fit_radius_m: 2.5,
            solver_guard_distance_m: 0.,
            persistence_threshold_fraction: 0.,
            ..Default::default()
        };
        let original_crossing = crossing(&source.field);
        let regular_interval = 1.;
        let bound = adjustment_bound(regular_interval);
        let (adjusted, diagnostics) =
            optimize_contour_field(&source, regular_interval, &params).unwrap();
        let adjusted_crossing = crossing(&adjusted.field);
        assert!(
            adjusted_crossing >= original_crossing + 1.9,
            "{original_crossing} -> {adjusted_crossing}"
        );
        assert!(
            (adjusted_crossing - break_x).abs() <= 0.1,
            "{original_crossing} -> {adjusted_crossing}, break={break_x}"
        );
        assert!(
            adjusted
                .field
                .iter()
                .zip(&source.field)
                .all(|(adjusted, original)| (adjusted - original).abs() <= bound + 1e-6)
        );
        assert!(diagnostics.published.maximum_adjustment <= bound + 1e-6);

        let wider_interval = 2.;
        let (_, wider_diagnostics) =
            optimize_contour_field(&source, wider_interval, &params).unwrap();
        assert!(
            wider_diagnostics.published.maximum_adjustment > 0.25,
            "the interval-relative bound was incorrectly treated as a fixed 0.25 m"
        );
        assert!(
            wider_diagnostics.published.maximum_adjustment
                <= adjustment_bound(wider_interval) + 1e-6
        );
    }

    #[test]
    fn macroscopic_slope_transition_concentrates_contours() {
        let grid = DfmGrid::new(96, 32, 0.5, geo::coord! { x: 0.25, y: 15.75 }).unwrap();
        let mut profile = vec![0_f32; grid.width];
        for x in 1..grid.width {
            let distance = x as f32 * 0.5;
            let transition = ((distance - 16.) / 8.).clamp(0., 1.);
            let transition = transition * transition * (3. - 2. * transition);
            let slope = 0.05 + 0.35 * transition;
            profile[x] = profile[x - 1] + 0.5 * slope;
        }
        let mut source = Dfm::<Elevation>::new(grid);
        let width = source.width();
        for y in 0..source.height() {
            for (value, profile) in source.field[y * width..(y + 1) * width]
                .iter_mut()
                .zip(&profile)
            {
                *value = 0.4 + profile;
            }
        }
        let params = ContourFieldParameters {
            persistence_threshold_fraction: 0.,
            collect_debug_rasters: true,
            ..Default::default()
        };
        let regular_interval = 1.;
        let (adjusted, diagnostics) =
            optimize_contour_field(&source, regular_interval, &params).unwrap();
        let debug = diagnostics.debug_rasters.as_ref().unwrap();
        let mean_dx = |field: &[f32], start_m: f32, end_m: f32| {
            let row = source.height() / 2;
            let start = (start_m / 0.5) as usize;
            let end = (end_m / 0.5) as usize;
            (start..end)
                .map(|x| {
                    (field[row * source.width() + x + 1] - field[row * source.width() + x]).abs()
                })
                .sum::<f32>()
                / (end - start) as f32
        };
        let source_concentration =
            mean_dx(&source.field, 18., 32.) / mean_dx(&source.field, 8., 14.);
        let adjusted_concentration =
            mean_dx(&adjusted.field, 18., 32.) / mean_dx(&adjusted.field, 8., 14.);
        assert!(
            adjusted_concentration >= source_concentration * 1.06,
            "macroscopic contour concentration {source_concentration} -> \
             {adjusted_concentration}"
        );
        let row = source.height() / 2;
        let gentle_cost = debug.contour_cost.field[row * source.width() + 20];
        let transition_cost = debug.contour_cost.field[row * source.width() + 44];
        assert!(
            transition_cost < gentle_cost * 0.2,
            "macroscopic attraction cost {gentle_cost} -> {transition_cost}"
        );
        assert!(
            diagnostics.published.maximum_adjustment <= adjustment_bound(regular_interval) + 1e-6
        );
    }

    #[test]
    #[ignore = "release-only weighted scalar-field benchmark"]
    #[allow(clippy::assertions_on_constants)]
    fn benchmark_weighted_scalar_field() {
        assert!(
            !cfg!(debug_assertions),
            "run with: cargo test --release benchmark_weighted_scalar_field -- --ignored --nocapture"
        );
        let max_size = std::env::var("OMAP_CONTOUR_BENCH_MAX_SIZE")
            .ok()
            .and_then(|value| value.parse::<usize>().ok())
            .unwrap_or(512);
        let mut fixtures = vec![
            ("flat", benchmark_fixture(64, |_, _| 100.)),
            (
                "plane",
                benchmark_fixture(64, |y, x| 100. + 0.03 * x as f32 - 0.02 * y as f32),
            ),
            (
                "hill",
                benchmark_fixture(64, |y, x| {
                    let dx = x as f32 - 31.5;
                    let dy = y as f32 - 31.5;
                    100. + 4. * (-(dx * dx + dy * dy) / 300.).exp()
                }),
            ),
            (
                "nested",
                benchmark_fixture(64, |y, x| {
                    let dx = x as f32 - 31.5;
                    let dy = y as f32 - 31.5;
                    let radius = (dx * dx + dy * dy).sqrt();
                    100. + (radius * 0.7).sin() * (-(radius / 28.).powi(2)).exp()
                }),
            ),
            (
                "bound_limited",
                benchmark_fixture(64, |y, x| {
                    100. + ((x * 17 + y * 31) % 11) as f32 * 0.08
                        + if (x + y) % 7 == 0 { -0.28 } else { 0. }
                }),
            ),
        ];
        for size in [40, 64, 128, 256, 512] {
            if size <= max_size {
                fixtures.push((
                    "rough",
                    benchmark_fixture(size, |y, x| {
                        let index = y * size + x;
                        let mixed = (index as u64)
                            .wrapping_mul(6_364_136_223_846_793_005)
                            .rotate_left(17);
                        100. + (mixed as u32) as f32 / u32::MAX as f32
                    }),
                ));
            }
        }

        let rustc = std::process::Command::new("rustc")
            .arg("--version")
            .output()
            .ok()
            .map(|output| String::from_utf8_lossy(&output.stdout).trim().to_owned())
            .unwrap_or_else(|| "unknown".to_owned());
        let cpu = std::fs::read_to_string("/proc/cpuinfo")
            .ok()
            .and_then(|contents| {
                contents
                    .lines()
                    .find_map(|line| line.strip_prefix("model name\t: ").map(str::to_owned))
            })
            .unwrap_or_else(|| "unknown".to_owned());
        println!(
            "# cpu={cpu}; rayon_threads={}; compiler={rustc}",
            rayon::current_num_threads()
        );
        println!(
            "fixture,width,height,total_us,target_persistence_us,derivatives_us,salience_us,\
             transfer_us,norm_us,solve_us,publication_persistence_us,diagnostics_us,iterations,\
             diagrams,passes,candidates,cancellations,cells_written,requested,removed,preserved,\
             unresolved,peak_rss_kib,field_hash,contour_hash"
        );
        for (name, source) in fixtures {
            let mut params = ContourFieldParameters::default();
            if let Ok(iterations) = std::env::var("OMAP_CONTOUR_BENCH_ITERATIONS")
                .ok()
                .and_then(|value| value.parse::<usize>().ok())
                .ok_or(())
            {
                params.max_iterations = iterations.max(3);
            }
            let (adjusted, diagnostics) = optimize_contour_field(&source, 1., &params).unwrap();
            let transfer = diagnostics
                .timings
                .levels
                .iter()
                .map(|level| level.transfer)
                .sum::<Duration>();
            let norm = diagnostics
                .timings
                .levels
                .iter()
                .map(|level| level.operator_norm)
                .sum::<Duration>();
            let solve = diagnostics
                .timings
                .levels
                .iter()
                .map(|level| level.solve)
                .sum::<Duration>();
            let mut work = diagnostics.persistence.target_work;
            work += diagnostics.persistence.publication_work;
            println!(
                "{}",
                [
                    name.to_owned(),
                    source.width().to_string(),
                    source.height().to_string(),
                    diagnostics.timings.total.as_micros().to_string(),
                    diagnostics
                        .timings
                        .target_persistence
                        .as_micros()
                        .to_string(),
                    diagnostics.timings.derivatives.as_micros().to_string(),
                    diagnostics.timings.salience.as_micros().to_string(),
                    transfer.as_micros().to_string(),
                    norm.as_micros().to_string(),
                    solve.as_micros().to_string(),
                    diagnostics
                        .timings
                        .publication_persistence
                        .as_micros()
                        .to_string(),
                    diagnostics
                        .timings
                        .published_diagnostics
                        .as_micros()
                        .to_string(),
                    diagnostics.solver.iterations.to_string(),
                    work.diagram_builds.to_string(),
                    work.cancellation_passes.to_string(),
                    work.candidates_considered.to_string(),
                    work.cancellations_applied.to_string(),
                    work.affected_cells_written.to_string(),
                    diagnostics.persistence.requested.to_string(),
                    diagnostics.persistence.verified_removed.to_string(),
                    diagnostics.persistence.preserved.to_string(),
                    diagnostics.persistence.unresolved.to_string(),
                    peak_rss_kib().to_string(),
                    format!("{:016x}", hash_field(&adjusted.field)),
                    format!("{:016x}", hash_contours(&adjusted, 1.)),
                ]
                .join(",")
            );
        }
    }

    fn benchmark_fixture(size: usize, value: impl Fn(usize, usize) -> f32) -> Dfm<Elevation> {
        let grid = DfmGrid::new(
            size,
            size,
            STANDARD_CELL_SIZE_METERS,
            geo::coord! { x: 0.25, y: size as f64 * STANDARD_CELL_SIZE_METERS - 0.25 },
        )
        .unwrap();
        let mut source = Dfm::new(grid);
        for y in 0..size {
            for x in 0..size {
                source[(y, x)] = value(y, x);
            }
        }
        source
    }

    fn hash_field(field: &[f32]) -> u64 {
        let mut hash = 0xcbf29ce484222325_u64;
        for value in field {
            for byte in value.to_bits().to_le_bytes() {
                hash ^= u64::from(byte);
                hash = hash.wrapping_mul(0x100000001b3);
            }
        }
        hash
    }

    fn hash_contours<T: RasterMarker>(field: &Dfm<T>, interval: f32) -> u64 {
        let min = field.field.iter().copied().min_by(f32::total_cmp).unwrap();
        let max = field.field.iter().copied().max_by(f32::total_cmp).unwrap();
        let first = (min / interval).floor() as i64;
        let last = (max / interval).ceil() as i64;
        let mut hash = 0xcbf29ce484222325_u64;
        for ordinal in first..=last {
            for line in field.marching_squares(ordinal as f32 * interval) {
                for coordinate in line.0 {
                    for value in [coordinate.x.to_bits(), coordinate.y.to_bits()] {
                        for byte in value.to_le_bytes() {
                            hash ^= u64::from(byte);
                            hash = hash.wrapping_mul(0x100000001b3);
                        }
                    }
                }
            }
        }
        hash
    }

    fn peak_rss_kib() -> usize {
        std::fs::read_to_string("/proc/self/status")
            .ok()
            .and_then(|contents| {
                contents.lines().find_map(|line| {
                    line.strip_prefix("VmHWM:")
                        .and_then(|value| value.split_whitespace().next())
                        .and_then(|value| value.parse().ok())
                })
            })
            .unwrap_or(0)
    }
}
