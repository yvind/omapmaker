use super::operators;
use crate::parameters::ContourFieldParameters;
use crate::raster::Dfm;
use crate::raster::dfm::{ContourCost, Elevation, SmoothnessWeight, TargetElevation};

#[derive(Clone, Copy, Debug, Default)]
pub(super) struct SolverDiagnostics {
    pub(super) iterations: usize,
    pub(super) fidelity_energy: f64,
    pub(super) weighted_tv_energy: f64,
    pub(super) hessian_energy: f64,
}

pub(super) struct SolverRasters<'a> {
    pub(super) source: &'a Dfm<Elevation>,
    pub(super) target: &'a Dfm<TargetElevation>,
    pub(super) contour_cost: &'a Dfm<ContourCost>,
    pub(super) smoothness: &'a Dfm<SmoothnessWeight>,
}

pub(super) fn solve(
    rasters: SolverRasters<'_>,
    initial: Option<&[f32]>,
    interval: f32,
    max_iterations: usize,
    params: &ContourFieldParameters,
) -> (Vec<f32>, SolverDiagnostics) {
    let SolverRasters {
        source,
        target,
        contour_cost,
        smoothness,
    } = rasters;
    source
        .grid
        .ensure_compatible(&target.grid)
        .and_then(|_| source.grid.ensure_compatible(&contour_cost.grid))
        .and_then(|_| source.grid.ensure_compatible(&smoothness.grid))
        .expect("contour solver requires matching grids");
    let len = source.field.len();
    let width = source.width();
    let height = source.height();
    let cell = source.grid.cell_size_m as f32;
    let bound = interval.abs() * 0.25;
    let mut primal = initial
        .filter(|values| values.len() == len)
        .map_or_else(|| target.field.to_vec(), <[f32]>::to_vec);
    for (value, source) in primal.iter_mut().zip(&source.field) {
        *value = value.clamp(*source - bound, *source + bound);
    }
    let mut extrapolated = primal.clone();
    let mut previous = primal.clone();
    let mut gradient = vec![[0.; 2]; len];
    let mut hessian = vec![[0.; 3]; len];
    let mut gradient_dual = vec![[0.; 2]; len];
    let mut hessian_dual = vec![[0.; 3]; len];
    let mut adjoint = vec![0.; len];
    let operator_norm = operators::norm(width, height, cell).max(1e-6);
    let step = 0.95_f32.sqrt() / operator_norm;
    let guard = ((params.solver_guard_distance_m / source.grid.cell_size_m).ceil() as usize)
        .min(width.min(height).saturating_sub(2) / 2);
    let mut stable_iterations = 0;
    let mut completed = 0;
    let generalization = params.generalization.factor();

    for iteration in 0..max_iterations {
        operators::apply(
            &extrapolated,
            width,
            height,
            cell,
            &mut gradient,
            &mut hessian,
        );
        let mut dual_change_squared = 0_f64;
        let mut dual_norm_squared = 0_f64;
        for i in 0..len {
            let previous_gradient = gradient_dual[i];
            let previous_hessian = hessian_dual[i];
            gradient_dual[i][0] += step * gradient[i][0];
            gradient_dual[i][1] += step * gradient[i][1];
            let radius = generalization * params.weighted_tv_weight * contour_cost.field[i];
            let length = gradient_dual[i][0].hypot(gradient_dual[i][1]);
            if length > radius {
                gradient_dual[i][0] *= radius / length;
                gradient_dual[i][1] *= radius / length;
            }
            let denominator = 1.
                + step / (generalization * params.hessian_weight * smoothness.field[i]).max(1e-12);
            for component in 0..3 {
                hessian_dual[i][component] =
                    (hessian_dual[i][component] + step * hessian[i][component]) / denominator;
                dual_change_squared +=
                    f64::from(hessian_dual[i][component] - previous_hessian[component]).powi(2);
                dual_norm_squared += f64::from(hessian_dual[i][component]).powi(2);
            }
            dual_change_squared += f64::from(gradient_dual[i][0] - previous_gradient[0]).powi(2)
                + f64::from(gradient_dual[i][1] - previous_gradient[1]).powi(2);
            dual_norm_squared +=
                f64::from(gradient_dual[i][0]).powi(2) + f64::from(gradient_dual[i][1]).powi(2);
        }
        operators::adjoint(
            &gradient_dual,
            &hessian_dual,
            width,
            height,
            cell,
            &mut adjoint,
        );
        previous.copy_from_slice(&primal);
        for i in 0..len {
            let value = (primal[i] - step * adjoint[i]
                + step * params.fidelity_weight * target.field[i])
                / (1. + step * params.fidelity_weight);
            primal[i] = value.clamp(source.field[i] - bound, source.field[i] + bound);
        }
        if guard > 0 {
            for y in 0..height {
                for x in 0..width {
                    if y < guard || y + guard >= height || x < guard || x + guard >= width {
                        let index = y * width + x;
                        primal[index] = target.field[index]
                            .clamp(source.field[index] - bound, source.field[index] + bound);
                    }
                }
            }
        }
        let change = primal
            .iter()
            .zip(&previous)
            .map(|(a, b)| (a - b).powi(2))
            .sum::<f32>()
            .sqrt()
            / previous.iter().map(|x| x * x).sum::<f32>().sqrt().max(1e-6);
        let dual_change = (dual_change_squared / dual_norm_squared.max(1e-12)).sqrt() as f32;
        for i in 0..len {
            extrapolated[i] = 2. * primal[i] - previous[i];
        }
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

    operators::apply(&primal, width, height, cell, &mut gradient, &mut hessian);
    let pixel_area = f64::from(cell * cell);
    let fidelity_energy = primal
        .iter()
        .zip(&target.field)
        .map(|(a, b)| 0.5 * f64::from(params.fidelity_weight * (a - b).powi(2)))
        .sum::<f64>()
        * pixel_area;
    let weighted_tv_energy = gradient
        .iter()
        .zip(&contour_cost.field)
        .map(|(value, weight)| {
            f64::from(
                generalization * params.weighted_tv_weight * weight * value[0].hypot(value[1]),
            )
        })
        .sum::<f64>()
        * pixel_area;
    let hessian_energy = hessian
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
    (
        primal,
        SolverDiagnostics {
            iterations: completed,
            fidelity_energy,
            weighted_tv_energy,
            hessian_energy,
        },
    )
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::raster::DfmGrid;

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
        let (adjusted, _) = solve(
            SolverRasters {
                source: &source,
                target: &target,
                contour_cost: &cost,
                smoothness: &smoothness,
            },
            None,
            2.,
            8,
            &ContourFieldParameters::default(),
        );
        assert!(
            adjusted
                .iter()
                .zip(&source.field)
                .all(|(a, b)| (a - b).abs() <= 0.5 + 1e-6)
        );
    }
}
