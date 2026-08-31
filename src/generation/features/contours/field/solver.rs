use super::operators;
use crate::parameters::ContourFieldParameters;
use crate::raster::{
    AlignmentConfidence, ContourCost, Dfm, Elevation, IsolineTangentX, IsolineTangentY,
    SmoothnessWeight, TargetElevation,
};
use rayon::prelude::*;

const REDUCTION_CHUNK: usize = 1024;

#[derive(Clone, Copy, Debug, Default)]
pub(super) struct SolverDiagnostics {
    pub(super) iterations: usize,
}

pub(super) struct SolverRasters<'a> {
    pub(super) source: &'a Dfm<Elevation>,
    pub(super) target: &'a Dfm<TargetElevation>,
    pub(super) contour_cost: &'a Dfm<ContourCost>,
    pub(super) smoothness: &'a Dfm<SmoothnessWeight>,
    pub(super) isoline_tangent_x: &'a Dfm<IsolineTangentX>,
    pub(super) isoline_tangent_y: &'a Dfm<IsolineTangentY>,
    pub(super) alignment_confidence: &'a Dfm<AlignmentConfidence>,
}

#[derive(Default)]
pub(super) struct SolverWorkspace {
    extrapolated: Vec<f32>,
    previous: Vec<f32>,
    gradient: Vec<[f32; 2]>,
    hessian: Vec<[f32; 3]>,
    gradient_dual: Vec<[f32; 2]>,
    alignment_dual: Vec<f32>,
    hessian_dual: Vec<[f32; 3]>,
    adjoint: Vec<f32>,
}

impl SolverWorkspace {
    fn prepare(&mut self, len: usize, primal: &[f32]) {
        self.extrapolated.resize(len, 0.);
        self.extrapolated.copy_from_slice(primal);
        self.previous.resize(len, 0.);
        self.gradient.resize(len, [0.; 2]);
        self.hessian.resize(len, [0.; 3]);
        self.gradient_dual.resize(len, [0.; 2]);
        self.gradient_dual.fill([0.; 2]);
        self.alignment_dual.resize(len, 0.);
        self.alignment_dual.fill(0.);
        self.hessian_dual.resize(len, [0.; 3]);
        self.hessian_dual.fill([0.; 3]);
        self.adjoint.resize(len, 0.);
        self.adjoint.fill(0.);
    }
}

pub(super) fn solve(
    rasters: SolverRasters<'_>,
    initial: Option<&[f32]>,
    interval: f32,
    max_iterations: usize,
    operator_norm: f32,
    params: &ContourFieldParameters,
    workspace: &mut SolverWorkspace,
) -> (Vec<f32>, SolverDiagnostics) {
    let SolverRasters {
        source,
        target,
        contour_cost,
        smoothness,
        isoline_tangent_x,
        isoline_tangent_y,
        alignment_confidence,
    } = rasters;
    source
        .grid
        .ensure_compatible(&target.grid)
        .and_then(|_| source.grid.ensure_compatible(&contour_cost.grid))
        .and_then(|_| source.grid.ensure_compatible(&smoothness.grid))
        .and_then(|_| source.grid.ensure_compatible(&isoline_tangent_x.grid))
        .and_then(|_| source.grid.ensure_compatible(&isoline_tangent_y.grid))
        .and_then(|_| source.grid.ensure_compatible(&alignment_confidence.grid))
        .expect("contour solver requires matching grids");
    let len = source.field.len();
    let width = source.width();
    let height = source.height();
    let cell = source.grid.cell_size_m as f32;
    let bound = super::adjustment_bound(interval);
    let mut primal = initial
        .filter(|values| values.len() == len)
        .map_or_else(|| target.field.to_vec(), <[f32]>::to_vec);
    for (value, source) in primal.iter_mut().zip(&source.field) {
        *value = value.clamp(*source - bound, *source + bound);
    }
    workspace.prepare(len, &primal);
    let SolverWorkspace {
        extrapolated,
        previous,
        gradient,
        hessian,
        gradient_dual,
        alignment_dual,
        hessian_dual,
        adjoint,
    } = workspace;
    let step = 0.95_f32.sqrt() / operator_norm.max(1e-6);
    let guard = ((params.solver_guard_distance_m / source.grid.cell_size_m).ceil() as usize)
        .min(width.min(height).saturating_sub(2) / 2);
    let mut stable_iterations = 0;
    let mut completed = 0;
    let generalization = params.generalization.factor();

    for iteration in 0..max_iterations {
        operators::apply(extrapolated, width, height, cell, gradient, hessian);
        let dual_partials = gradient_dual
            .par_chunks_mut(REDUCTION_CHUNK)
            .zip(alignment_dual.par_chunks_mut(REDUCTION_CHUNK))
            .zip(hessian_dual.par_chunks_mut(REDUCTION_CHUNK))
            .zip(gradient.par_chunks(REDUCTION_CHUNK))
            .zip(hessian.par_chunks(REDUCTION_CHUNK))
            .zip(contour_cost.field.par_chunks(REDUCTION_CHUNK))
            .zip(smoothness.field.par_chunks(REDUCTION_CHUNK))
            .zip(isoline_tangent_x.field.par_chunks(REDUCTION_CHUNK))
            .zip(isoline_tangent_y.field.par_chunks(REDUCTION_CHUNK))
            .zip(alignment_confidence.field.par_chunks(REDUCTION_CHUNK))
            .map(
                |(
                    (
                        (
                            (
                                (
                                    (
                                        (((gradient_dual, alignment_dual), hessian_dual), gradient),
                                        hessian,
                                    ),
                                    contour_cost,
                                ),
                                smoothness,
                            ),
                            tangent_x,
                        ),
                        tangent_y,
                    ),
                    alignment_confidence,
                )| {
                    let mut change_squared = 0_f64;
                    let mut norm_squared = 0_f64;
                    for i in 0..gradient_dual.len() {
                        let previous_gradient = gradient_dual[i];
                        let previous_alignment = alignment_dual[i];
                        let previous_hessian = hessian_dual[i];
                        gradient_dual[i][0] += step * gradient[i][0];
                        gradient_dual[i][1] += step * gradient[i][1];
                        let radius = generalization * params.weighted_tv_weight * contour_cost[i];
                        let length = gradient_dual[i][0].hypot(gradient_dual[i][1]);
                        if length > radius {
                            gradient_dual[i][0] *= radius / length;
                            gradient_dual[i][1] *= radius / length;
                        }
                        let alignment_strength = params.alignment_weight * alignment_confidence[i];
                        alignment_dual[i] = if alignment_strength > 1e-12 {
                            (alignment_dual[i]
                                + step
                                    * (tangent_x[i] * gradient[i][0]
                                        + tangent_y[i] * gradient[i][1]))
                                / (1. + step / alignment_strength)
                        } else {
                            0.
                        };
                        let denominator = 1.
                            + step
                                / (generalization * params.hessian_weight * smoothness[i])
                                    .max(1e-12);
                        for component in 0..3 {
                            hessian_dual[i][component] = (hessian_dual[i][component]
                                + step * hessian[i][component])
                                / denominator;
                            change_squared +=
                                f64::from(hessian_dual[i][component] - previous_hessian[component])
                                    .powi(2);
                            norm_squared += f64::from(hessian_dual[i][component]).powi(2);
                        }
                        change_squared += f64::from(gradient_dual[i][0] - previous_gradient[0])
                            .powi(2)
                            + f64::from(gradient_dual[i][1] - previous_gradient[1]).powi(2)
                            + f64::from(alignment_dual[i] - previous_alignment).powi(2);
                        norm_squared += f64::from(gradient_dual[i][0]).powi(2)
                            + f64::from(gradient_dual[i][1]).powi(2)
                            + f64::from(alignment_dual[i]).powi(2);
                    }
                    (change_squared, norm_squared)
                },
            )
            .collect::<Vec<_>>();
        let (dual_change_squared, dual_norm_squared) = dual_partials
            .into_iter()
            .fold((0., 0.), |(change, norm), partial| {
                (change + partial.0, norm + partial.1)
            });
        operators::adjoint_with_alignment(
            gradient_dual,
            hessian_dual,
            operators::AlignmentAdjoint {
                dual: alignment_dual,
                tangent_x: &isoline_tangent_x.field,
                tangent_y: &isoline_tangent_y.field,
            },
            width,
            height,
            cell,
            adjoint,
        );
        let primal_partials = primal
            .par_chunks_mut(REDUCTION_CHUNK)
            .zip(previous.par_chunks_mut(REDUCTION_CHUNK))
            .zip(extrapolated.par_chunks_mut(REDUCTION_CHUNK))
            .zip(adjoint.par_chunks(REDUCTION_CHUNK))
            .zip(source.field.par_chunks(REDUCTION_CHUNK))
            .zip(target.field.par_chunks(REDUCTION_CHUNK))
            .enumerate()
            .map(
                |(chunk, (((((primal, previous), extrapolated), adjoint), source), target))| {
                    let mut change_squared = 0_f64;
                    let mut previous_squared = 0_f64;
                    for i in 0..primal.len() {
                        let index = chunk * REDUCTION_CHUNK + i;
                        let old = primal[i];
                        previous[i] = old;
                        let value = (old - step * adjoint[i]
                            + step * params.fidelity_weight * target[i])
                            / (1. + step * params.fidelity_weight);
                        let mut value = value.clamp(source[i] - bound, source[i] + bound);
                        if guard > 0 {
                            let y = index / width;
                            let x = index % width;
                            if y < guard || y + guard >= height || x < guard || x + guard >= width {
                                value = target[i].clamp(source[i] - bound, source[i] + bound);
                            }
                        }
                        primal[i] = value;
                        extrapolated[i] = 2. * value - old;
                        change_squared += f64::from(value - old).powi(2);
                        previous_squared += f64::from(old).powi(2);
                    }
                    (change_squared, previous_squared)
                },
            )
            .collect::<Vec<_>>();
        let (change_squared, previous_squared) = primal_partials
            .into_iter()
            .fold((0., 0.), |(change, previous), partial| {
                (change + partial.0, previous + partial.1)
            });
        let change = (change_squared / previous_squared.max(1e-12)).sqrt() as f32;
        let dual_change = (dual_change_squared / dual_norm_squared.max(1e-12)).sqrt() as f32;
        stable_iterations = if change < params.convergence_tolerance
            && dual_change < params.convergence_tolerance
        {
            stable_iterations + 1
        } else {
            0
        };
        completed = iteration + 1;
        if stable_iterations >= 5 {
            break;
        }
    }

    (
        primal,
        SolverDiagnostics {
            iterations: completed,
        },
    )
}

#[derive(Clone, Copy, Debug)]
pub(super) struct FieldEnergies {
    pub(super) fidelity: f64,
    pub(super) weighted_tv: f64,
    pub(super) alignment: f64,
    pub(super) hessian: f64,
}

pub(super) fn evaluate(
    field: &[f32],
    rasters: SolverRasters<'_>,
    params: &ContourFieldParameters,
    workspace: &mut SolverWorkspace,
) -> FieldEnergies {
    let SolverRasters {
        source,
        target,
        contour_cost,
        smoothness,
        isoline_tangent_x,
        isoline_tangent_y,
        alignment_confidence,
    } = rasters;
    source
        .grid
        .ensure_compatible(&target.grid)
        .and_then(|_| source.grid.ensure_compatible(&contour_cost.grid))
        .and_then(|_| source.grid.ensure_compatible(&smoothness.grid))
        .and_then(|_| source.grid.ensure_compatible(&isoline_tangent_x.grid))
        .and_then(|_| source.grid.ensure_compatible(&isoline_tangent_y.grid))
        .and_then(|_| source.grid.ensure_compatible(&alignment_confidence.grid))
        .expect("contour energy evaluation requires matching grids");
    assert_eq!(field.len(), target.field.len());
    let width = target.width();
    let height = target.height();
    let cell = target.grid.cell_size_m as f32;
    workspace.gradient.resize(field.len(), [0.; 2]);
    workspace.hessian.resize(field.len(), [0.; 3]);
    operators::apply(
        field,
        width,
        height,
        cell,
        &mut workspace.gradient,
        &mut workspace.hessian,
    );
    let generalization = params.generalization.factor();
    let pixel_area = f64::from(cell * cell);
    let fidelity = field
        .iter()
        .zip(&target.field)
        .map(|(a, b)| 0.5 * f64::from(params.fidelity_weight * (a - b).powi(2)))
        .sum::<f64>()
        * pixel_area;
    let weighted_tv_energy = workspace
        .gradient
        .iter()
        .zip(&contour_cost.field)
        .map(|(value, weight)| {
            f64::from(
                generalization * params.weighted_tv_weight * weight * value[0].hypot(value[1]),
            )
        })
        .sum::<f64>()
        * pixel_area;
    let alignment_energy = workspace
        .gradient
        .iter()
        .zip(&isoline_tangent_x.field)
        .zip(&isoline_tangent_y.field)
        .zip(&alignment_confidence.field)
        .map(|(((gradient, tangent_x), tangent_y), confidence)| {
            0.5 * f64::from(
                params.alignment_weight
                    * confidence
                    * (tangent_x * gradient[0] + tangent_y * gradient[1]).powi(2),
            )
        })
        .sum::<f64>()
        * pixel_area;
    let hessian_energy = workspace
        .hessian
        .iter()
        .zip(&smoothness.field)
        .map(|(value, weight)| {
            0.5 * f64::from(
                generalization
                    * params.hessian_weight
                    * weight
                    * (value[0].powi(2) + value[1].powi(2) + value[2].powi(2)),
            )
        })
        .sum::<f64>()
        * pixel_area;
    FieldEnergies {
        fidelity,
        weighted_tv: weighted_tv_energy,
        alignment: alignment_energy,
        hessian: hessian_energy,
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::raster::DfmGrid;
    use rayon::ThreadPoolBuilder;

    #[test]
    fn solver_never_leaves_elevation_box() {
        let grid = DfmGrid::new(9, 7, 0.5, geo::coord! { x: 0., y: 3. }).unwrap();
        let mut source = Dfm::<Elevation>::new(grid);
        for (i, value) in source.field.iter_mut().enumerate() {
            *value = (i as f32 * 0.3).sin();
        }
        let mut target = Dfm::<TargetElevation>::new_like(&source);
        target.field.fill(100.);
        let mut cost = Dfm::<ContourCost>::new_like(&source);
        cost.field.fill(1.);
        let mut smoothness = Dfm::<SmoothnessWeight>::new_like(&source);
        smoothness.field.fill(1.);
        let mut tangent_x = Dfm::<IsolineTangentX>::new_like(&source);
        tangent_x.field.fill(1.);
        let mut tangent_y = Dfm::<IsolineTangentY>::new_like(&source);
        tangent_y.field.fill(0.);
        let mut alignment_confidence = Dfm::<AlignmentConfidence>::new_like(&source);
        alignment_confidence.field.fill(1.);
        let (adjusted, _) = solve(
            SolverRasters {
                source: &source,
                target: &target,
                contour_cost: &cost,
                smoothness: &smoothness,
                isoline_tangent_x: &tangent_x,
                isoline_tangent_y: &tangent_y,
                alignment_confidence: &alignment_confidence,
            },
            None,
            2.,
            8,
            operators::norm(source.width(), source.height(), 0.5),
            &ContourFieldParameters::default(),
            &mut SolverWorkspace::default(),
        );
        assert!(
            adjusted
                .iter()
                .zip(&source.field)
                .all(|(a, b)| (a - b).abs() <= 0.5 + 1e-6)
        );
    }

    #[test]
    fn solver_is_thread_count_deterministic() {
        let grid = DfmGrid::new(65, 40, 0.5, geo::coord! { x: 0., y: 20. }).unwrap();
        let mut source = Dfm::<Elevation>::new(grid);
        for (i, value) in source.field.iter_mut().enumerate() {
            *value = (i as f32 * 0.037).sin();
        }
        let mut target = Dfm::<TargetElevation>::new_like(&source);
        for (i, value) in target.field.iter_mut().enumerate() {
            *value = source.field[i] + (i as f32 * 0.071).cos() * 0.2;
        }
        let mut cost = Dfm::<ContourCost>::new_like(&source);
        cost.field.fill(1.);
        let mut smoothness = Dfm::<SmoothnessWeight>::new_like(&source);
        smoothness.field.fill(1.);
        let mut tangent_x = Dfm::<IsolineTangentX>::new_like(&source);
        tangent_x.field.fill(1.);
        let mut tangent_y = Dfm::<IsolineTangentY>::new_like(&source);
        tangent_y.field.fill(0.);
        let mut alignment_confidence = Dfm::<AlignmentConfidence>::new_like(&source);
        alignment_confidence.field.fill(1.);
        let params = ContourFieldParameters {
            solver_guard_distance_m: 0.5,
            ..Default::default()
        };
        let run = |threads| {
            ThreadPoolBuilder::new()
                .num_threads(threads)
                .build()
                .unwrap()
                .install(|| {
                    solve(
                        SolverRasters {
                            source: &source,
                            target: &target,
                            contour_cost: &cost,
                            smoothness: &smoothness,
                            isoline_tangent_x: &tangent_x,
                            isoline_tangent_y: &tangent_y,
                            alignment_confidence: &alignment_confidence,
                        },
                        None,
                        2.,
                        12,
                        operators::norm(source.width(), source.height(), 0.5),
                        &params,
                        &mut SolverWorkspace::default(),
                    )
                })
        };
        let (serial, serial_diagnostics) = run(1);
        let (parallel, parallel_diagnostics) = run(4);
        assert_eq!(serial, parallel);
        assert_eq!(
            serial_diagnostics.iterations,
            parallel_diagnostics.iterations
        );
    }

    #[test]
    fn isoline_alignment_rejects_an_orthogonal_attraction_band() {
        let grid = DfmGrid::new(65, 65, 0.5, geo::coord! { x: 0., y: 32. }).unwrap();
        let mut source = Dfm::<Elevation>::new(grid);
        for y in 0..source.height() {
            for x in 0..source.width() {
                source[(y, x)] = x as f32 * 0.05;
            }
        }
        let mut target = Dfm::<TargetElevation>::new_like(&source);
        target.field.copy_from_slice(&source.field);
        let mut cost = Dfm::<ContourCost>::new_like(&source);
        for y in 0..source.height() {
            for x in 0..source.width() {
                cost[(y, x)] = if x.abs_diff(y) <= 1 { 0.001 } else { 1. };
            }
        }
        let mut smoothness = Dfm::<SmoothnessWeight>::new_like(&source);
        smoothness.field.fill(0.001);
        let mut tangent_x = Dfm::<IsolineTangentX>::new_like(&source);
        tangent_x.field.fill(0.);
        let mut tangent_y = Dfm::<IsolineTangentY>::new_like(&source);
        tangent_y.field.fill(1.);
        let mut alignment_confidence = Dfm::<AlignmentConfidence>::new_like(&source);
        alignment_confidence.field.fill(1.);
        let run = |alignment_weight| {
            let params = ContourFieldParameters {
                fidelity_weight: 0.001,
                hessian_weight: 0.001,
                alignment_weight,
                solver_guard_distance_m: 0.,
                ..Default::default()
            };
            solve(
                SolverRasters {
                    source: &source,
                    target: &target,
                    contour_cost: &cost,
                    smoothness: &smoothness,
                    isoline_tangent_x: &tangent_x,
                    isoline_tangent_y: &tangent_y,
                    alignment_confidence: &alignment_confidence,
                },
                None,
                1.,
                160,
                operators::norm(source.width(), source.height(), 0.5),
                &params,
                &mut SolverWorkspace::default(),
            )
            .0
        };
        let width = source.width();
        let height = source.height();
        let tangent_energy = |field: &[f32]| {
            (0..height - 1)
                .flat_map(|y| {
                    (0..width).map(move |x| {
                        let i = y * width + x;
                        (field[i + width] - field[i]).powi(2)
                    })
                })
                .sum::<f32>()
        };

        let unaligned = run(0.);
        let aligned = run(ContourFieldParameters::default().alignment_weight);
        let unaligned_energy = tangent_energy(&unaligned);
        let aligned_energy = tangent_energy(&aligned);
        assert!(
            aligned_energy * 4. < unaligned_energy,
            "tangent energy {unaligned_energy} -> {aligned_energy}"
        );
    }
}
