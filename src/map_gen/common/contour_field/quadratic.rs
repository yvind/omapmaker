use crate::parameters::ContourFieldParameters;
use crate::raster::Dfm;
use crate::raster::dfm::{
    DirectionConfidence, Elevation, FitConfidence, IsolineTangentX, IsolineTangentY, ProfileChange,
    Slope, TangentChange,
};

use rayon::prelude::*;
use std::collections::HashMap;
use std::sync::{Arc, Mutex, OnceLock};

pub(super) struct TerrainDerivatives {
    pub(super) slope: Dfm<Slope>,
    pub(super) profile_change: Dfm<ProfileChange>,
    pub(super) tangent_change: Dfm<TangentChange>,
    pub(super) direction_confidence: Dfm<DirectionConfidence>,
    pub(super) fit_confidence: Dfm<FitConfidence>,
    pub(super) isoline_tangent_x: Dfm<IsolineTangentX>,
    pub(super) isoline_tangent_y: Dfm<IsolineTangentY>,
}

struct FitKernel {
    radius: usize,
    samples: Vec<FitSample>,
}

struct FitSample {
    dy: isize,
    dx: isize,
    weighted_design: [f64; 6],
    coefficients: [f64; 6],
    weight: f64,
}

struct AddressedFitKernel {
    radius: usize,
    samples: Vec<AddressedFitSample>,
}

struct AddressedFitSample {
    dy: isize,
    dx: isize,
    linear_offset: isize,
    weighted_design: [f64; 6],
    coefficients: [f64; 6],
    weight: f64,
}

struct AddressedGradientKernel {
    radius: usize,
    samples: Vec<AddressedGradientSample>,
}

struct AddressedGradientSample {
    dy: isize,
    dx: isize,
    linear_offset: isize,
    coefficients: [f64; 2],
}

impl FitKernel {
    fn new(cell: f64, radius_m: f64, sigma_m: f64) -> crate::Result<Self> {
        let radius = (radius_m / cell).ceil().max(1.) as isize;
        let mut normal = [[0.; 6]; 6];
        let mut raw = Vec::with_capacity((2 * radius + 1).pow(2) as usize);
        for dy in -radius..=radius {
            for dx in -radius..=radius {
                let x = dx as f64 * cell;
                let y = -(dy as f64) * cell;
                let design = [1., x, y, x * x, x * y, y * y];
                let weight = (-(x * x + y * y) / (2. * sigma_m * sigma_m)).exp();
                for row in 0..6 {
                    for column in 0..6 {
                        normal[row][column] += weight * design[row] * design[column];
                    }
                }
                raw.push((dy, dx, design.map(|value| value * weight), weight));
            }
        }
        let inverse = invert(normal)?;
        let samples = raw
            .into_iter()
            .map(|(dy, dx, weighted_design, weight)| {
                let mut coefficients = [0.; 6];
                for row in 0..6 {
                    coefficients[row] = (0..6)
                        .map(|column| inverse[row][column] * weighted_design[column])
                        .sum();
                }
                FitSample {
                    dy,
                    dx,
                    weighted_design,
                    coefficients,
                    weight,
                }
            })
            .collect();
        Ok(Self {
            radius: radius as usize,
            samples,
        })
    }

    fn cached(cell: f64, radius_m: f64, sigma_m: f64) -> crate::Result<Arc<Self>> {
        type Cache = Mutex<HashMap<(u64, u64, u64), Arc<FitKernel>>>;
        static CACHE: OnceLock<Cache> = OnceLock::new();
        let key = (cell.to_bits(), radius_m.to_bits(), sigma_m.to_bits());
        if let Some(kernel) = CACHE
            .get_or_init(Default::default)
            .lock()
            .expect("quadratic-kernel cache poisoned")
            .get(&key)
            .cloned()
        {
            return Ok(kernel);
        }
        let kernel = Arc::new(Self::new(cell, radius_m, sigma_m)?);
        CACHE
            .get_or_init(Default::default)
            .lock()
            .expect("quadratic-kernel cache poisoned")
            .insert(key, Arc::clone(&kernel));
        Ok(kernel)
    }

    fn addressed_fit(&self, width: usize) -> AddressedFitKernel {
        AddressedFitKernel {
            radius: self.radius,
            samples: self
                .samples
                .iter()
                .map(|sample| AddressedFitSample {
                    dy: sample.dy,
                    dx: sample.dx,
                    linear_offset: sample.dy * width as isize + sample.dx,
                    weighted_design: sample.weighted_design,
                    coefficients: sample.coefficients,
                    weight: sample.weight,
                })
                .collect(),
        }
    }

    fn addressed_gradient(&self, width: usize) -> AddressedGradientKernel {
        AddressedGradientKernel {
            radius: self.radius,
            samples: self
                .samples
                .iter()
                .map(|sample| AddressedGradientSample {
                    dy: sample.dy,
                    dx: sample.dx,
                    linear_offset: sample.dy * width as isize + sample.dx,
                    coefficients: [sample.coefficients[1], sample.coefficients[2]],
                })
                .collect(),
        }
    }
}

impl AddressedFitKernel {
    fn apply(&self, source: &Dfm<Elevation>, y: usize, x: usize) -> ([f64; 6], f64) {
        let mut right_hand_side = [0.; 6];
        let mut coefficients = [0.; 6];
        let mut weighted_squares = 0.;
        let mut weight_sum = 0.;
        let interior = y >= self.radius
            && y + self.radius < source.height()
            && x >= self.radius
            && x + self.radius < source.width();
        let center = y * source.width() + x;
        for sample in &self.samples {
            let index = if interior {
                (center as isize + sample.linear_offset) as usize
            } else {
                let row = (y as isize + sample.dy).clamp(0, source.height() as isize - 1) as usize;
                let column =
                    (x as isize + sample.dx).clamp(0, source.width() as isize - 1) as usize;
                row * source.width() + column
            };
            let value = f64::from(source.field[index]);
            for i in 0..6 {
                right_hand_side[i] += sample.weighted_design[i] * value;
                coefficients[i] += sample.coefficients[i] * value;
            }
            weighted_squares += sample.weight * value * value;
            weight_sum += sample.weight;
        }
        let residual = (weighted_squares
            - coefficients
                .iter()
                .zip(right_hand_side)
                .map(|(coefficient, rhs)| coefficient * rhs)
                .sum::<f64>())
        .max(0.);
        (coefficients, (residual / weight_sum).sqrt())
    }
}

impl AddressedGradientKernel {
    fn apply(&self, source: &Dfm<Elevation>, y: usize, x: usize) -> [f64; 2] {
        let interior = y >= self.radius
            && y + self.radius < source.height()
            && x >= self.radius
            && x + self.radius < source.width();
        let center = y * source.width() + x;
        let mut gradient = [0.; 2];
        for sample in &self.samples {
            let index = if interior {
                (center as isize + sample.linear_offset) as usize
            } else {
                let row = (y as isize + sample.dy).clamp(0, source.height() as isize - 1) as usize;
                let column =
                    (x as isize + sample.dx).clamp(0, source.width() as isize - 1) as usize;
                row * source.width() + column
            };
            let value = f64::from(source.field[index]);
            gradient[0] += sample.coefficients[0] * value;
            gradient[1] += sample.coefficients[1] * value;
        }
        gradient
    }
}

pub(super) fn calculate(
    source: &Dfm<Elevation>,
    interval: f32,
    params: &ContourFieldParameters,
) -> crate::Result<TerrainDerivatives> {
    let slope_kernel = FitKernel::cached(
        source.grid.cell_size_m,
        params.slope_fit_radius_m,
        params.slope_fit_radius_m / 2.,
    )?;
    let curvature_kernel = FitKernel::cached(
        source.grid.cell_size_m,
        params.curvature_fit_radius_m,
        params.curvature_fit_radius_m / 2.,
    )?;
    let width = source.width();
    let slope_kernel = slope_kernel.addressed_gradient(width);
    let curvature_kernel = curvature_kernel.addressed_fit(width);
    let mut slope = Dfm::new_like(source);
    let mut profile_change = Dfm::new_like(source);
    let mut tangent_change = Dfm::new_like(source);
    let mut direction_confidence = Dfm::new_like(source);
    let mut fit_confidence = Dfm::new_like(source);
    let mut isoline_tangent_x = Dfm::new_like(source);
    let mut isoline_tangent_y = Dfm::new_like(source);
    slope
        .field
        .par_iter_mut()
        .zip(profile_change.field.par_iter_mut())
        .zip(tangent_change.field.par_iter_mut())
        .zip(direction_confidence.field.par_iter_mut())
        .zip(fit_confidence.field.par_iter_mut())
        .zip(isoline_tangent_x.field.par_iter_mut())
        .zip(isoline_tangent_y.field.par_iter_mut())
        .enumerate()
        .for_each(
            |(
                index,
                (
                    (
                        (
                            (((slope_output, profile_output), tangent_output), direction_output),
                            fit_output,
                        ),
                        tangent_x_output,
                    ),
                    tangent_y_output,
                ),
            )| {
                let y = index / width;
                let x = index % width;
                let slope_fit = slope_kernel.apply(source, y, x);
                let (curvature, rmse) = curvature_kernel.apply(source, y, x);
                let gx = slope_fit[0];
                let gy = slope_fit[1];
                let slope = gx.hypot(gy);
                let epsilon = f64::from(params.slope_epsilon);
                let direction_confidence = slope * slope / (slope * slope + epsilon * epsilon);
                let (nx, ny) = if slope > epsilon {
                    (gx / slope, gy / slope)
                } else {
                    (0., 0.)
                };
                let (tx, ty) = (-ny, nx);
                let hxx = 2. * curvature[3];
                let hxy = curvature[4];
                let hyy = 2. * curvature[5];
                let scale = f64::from(interval.abs());
                let profile =
                    (nx * (hxx * nx + hxy * ny) + ny * (hxy * nx + hyy * ny)).abs() * scale;
                let tangent =
                    (tx * (hxx * tx + hxy * ty) + ty * (hxy * tx + hyy * ty)).abs() * scale;
                let fit_confidence = (-(rmse / f64::from(params.rmse_reference)).powi(2)).exp();
                *slope_output = slope as f32;
                *profile_output = profile as f32;
                *tangent_output = tangent as f32;
                *direction_output = direction_confidence as f32;
                *fit_output = fit_confidence as f32;
                *tangent_x_output = tx as f32;
                // Solver gradients use increasing raster-row coordinates,
                // opposite to the fitted surface's world-Y coordinate.
                *tangent_y_output = -ty as f32;
            },
        );

    Ok(TerrainDerivatives {
        slope,
        profile_change,
        tangent_change,
        direction_confidence,
        fit_confidence,
        isoline_tangent_x,
        isoline_tangent_y,
    })
}

fn invert(mut matrix: [[f64; 6]; 6]) -> crate::Result<[[f64; 6]; 6]> {
    let mut inverse = [[0.; 6]; 6];
    for (i, row) in inverse.iter_mut().enumerate() {
        row[i] = 1.;
    }
    for column in 0..6 {
        let pivot = (column..6)
            .max_by(|&a, &b| matrix[a][column].abs().total_cmp(&matrix[b][column].abs()))
            .expect("nonempty pivot range");
        anyhow::ensure!(
            matrix[pivot][column].abs() > 1e-12,
            "quadratic fit is singular"
        );
        matrix.swap(column, pivot);
        inverse.swap(column, pivot);
        let scale = matrix[column][column];
        for x in 0..6 {
            matrix[column][x] /= scale;
            inverse[column][x] /= scale;
        }
        for row in 0..6 {
            if row == column {
                continue;
            }
            let scale = matrix[row][column];
            for x in 0..6 {
                matrix[row][x] -= scale * matrix[column][x];
                inverse[row][x] -= scale * inverse[column][x];
            }
        }
    }
    Ok(inverse)
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::raster::DfmGrid;

    #[test]
    fn quadratic_fit_recovers_rotated_surface() {
        let grid = DfmGrid::new(21, 21, 0.5, geo::coord! { x: -5., y: 5. }).unwrap();
        let mut source = Dfm::new(grid);
        for y in 0..source.height() {
            for x in 0..source.width() {
                let c = source.index2coord(y, x);
                source[(y, x)] = (3. + 2. * c.x - c.y + 0.4 * c.x * c.x + 0.3 * c.x * c.y
                    - 0.2 * c.y * c.y) as f32;
            }
        }
        let kernel = FitKernel::new(0.5, 3., 1.5).unwrap();
        let (fit, rmse) = kernel.addressed_fit(source.width()).apply(&source, 10, 10);
        for (actual, expected) in fit.iter().zip([3., 2., -1., 0.4, 0.3, -0.2]) {
            assert!((actual - expected).abs() < 1e-5);
        }
        assert!(rmse < 1e-5);
    }

    #[test]
    fn specialized_gradient_matches_full_fit_at_interior_and_boundaries() {
        let grid = DfmGrid::new(17, 13, 0.5, geo::coord! { x: -4., y: 3. }).unwrap();
        let mut source = Dfm::new(grid);
        for (index, value) in source.field.iter_mut().enumerate() {
            *value = (index as f32 * 0.37).sin();
        }
        let kernel = FitKernel::new(0.5, 2., 1.).unwrap();
        let fit = kernel.addressed_fit(source.width());
        let gradient = kernel.addressed_gradient(source.width());
        for (y, x) in [(0, 0), (0, 8), (6, 0), (12, 16), (6, 8)] {
            let (full, _) = fit.apply(&source, y, x);
            let specialized = gradient.apply(&source, y, x);
            assert!((full[1] - specialized[0]).abs() < 1e-12);
            assert!((full[2] - specialized[1]).abs() < 1e-12);
        }
    }

    #[test]
    fn stored_isoline_tangent_matches_raster_gradient_coordinates() {
        let grid = DfmGrid::new(21, 21, 0.5, geo::coord! { x: -5., y: 5. }).unwrap();
        let mut source = Dfm::<Elevation>::new(grid);
        for y in 0..source.height() {
            for x in 0..source.width() {
                let coordinate = source.index2coord(y, x);
                source[(y, x)] = (2. * coordinate.x - coordinate.y) as f32;
            }
        }
        let params = ContourFieldParameters {
            slope_fit_radius_m: 2.,
            curvature_fit_radius_m: 2.,
            ..Default::default()
        };
        let derivatives = calculate(&source, 1., &params).unwrap();
        let (y, x) = (10, 10);
        let index = y * source.width() + x;
        let cell = source.grid.cell_size_m as f32;
        let raster_gradient = [
            (source.field[index + 1] - source.field[index]) / cell,
            (source.field[index + source.width()] - source.field[index]) / cell,
        ];
        let tangent = [
            derivatives.isoline_tangent_x.field[index],
            derivatives.isoline_tangent_y.field[index],
        ];

        assert!((tangent[0] * raster_gradient[0] + tangent[1] * raster_gradient[1]).abs() < 1e-5);
        assert!((tangent[0].hypot(tangent[1]) - 1.).abs() < 1e-5);
    }
}
