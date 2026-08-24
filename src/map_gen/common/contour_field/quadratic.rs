use crate::parameters::ContourFieldParameters;
use crate::raster::Dfm;
use crate::raster::dfm::{
    DirectionConfidence, Elevation, FitConfidence, ProfileChange, Slope, TangentChange,
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
}

struct FitKernel {
    radius: isize,
    inverse: [[f64; 6]; 6],
    samples: Vec<([f64; 6], [f64; 6], f64)>,
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
                raw.push((design.map(|value| value * weight), weight));
            }
        }
        let inverse = invert(normal)?;
        let samples = raw
            .into_iter()
            .map(|(weighted_design, weight)| {
                let mut coefficients = [0.; 6];
                for row in 0..6 {
                    coefficients[row] = (0..6)
                        .map(|column| inverse[row][column] * weighted_design[column])
                        .sum();
                }
                (weighted_design, coefficients, weight)
            })
            .collect();
        Ok(Self {
            radius,
            inverse,
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

    fn gradient(&self, source: &Dfm<Elevation>, y: usize, x: usize) -> [f64; 2] {
        let mut gradient = [0.; 2];
        for (sample_index, (_, kernel, _)) in self.samples.iter().enumerate() {
            let side = (2 * self.radius + 1) as usize;
            let dy = sample_index / side;
            let dx = sample_index % side;
            let row = (y as isize + dy as isize - self.radius)
                .clamp(0, source.height() as isize - 1) as usize;
            let column = (x as isize + dx as isize - self.radius)
                .clamp(0, source.width() as isize - 1) as usize;
            let value = f64::from(source[(row, column)]);
            gradient[0] += kernel[1] * value;
            gradient[1] += kernel[2] * value;
        }
        gradient
    }

    fn fit(&self, source: &Dfm<Elevation>, y: usize, x: usize) -> ([f64; 6], f64) {
        let mut right_hand_side = [0.; 6];
        let mut weighted_squares = 0.;
        let mut weight_sum = 0.;
        for (sample_index, (weighted_design, _, weight)) in self.samples.iter().enumerate() {
            let side = (2 * self.radius + 1) as usize;
            let dy = sample_index / side;
            let dx = sample_index % side;
            let row = (y as isize + dy as isize - self.radius)
                .clamp(0, source.height() as isize - 1) as usize;
            let column = (x as isize + dx as isize - self.radius)
                .clamp(0, source.width() as isize - 1) as usize;
            let value = f64::from(source[(row, column)]);
            for i in 0..6 {
                right_hand_side[i] += weighted_design[i] * value;
            }
            weighted_squares += weight * value * value;
            weight_sum += weight;
        }
        let mut coefficients = [0.; 6];
        for (row, coefficient) in coefficients.iter_mut().enumerate() {
            *coefficient = (0..6)
                .map(|column| self.inverse[row][column] * right_hand_side[column])
                .sum();
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
    let mut values = vec![(0_f32, 0_f32, 0_f32, 0_f32, 0_f32); source.field.len()];
    let width = source.width();
    values
        .par_iter_mut()
        .enumerate()
        .for_each(|(index, output)| {
            let y = index / width;
            let x = index % width;
            let gradient = slope_kernel.gradient(source, y, x);
            let (curvature, rmse) = curvature_kernel.fit(source, y, x);
            let gx = gradient[0];
            let gy = gradient[1];
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
            let profile = (nx * (hxx * nx + hxy * ny) + ny * (hxy * nx + hyy * ny)).abs() * scale;
            let tangent = (tx * (hxx * tx + hxy * ty) + ty * (hxy * tx + hyy * ty)).abs() * scale;
            let fit_confidence = (-(rmse / f64::from(params.rmse_reference)).powi(2)).exp();
            *output = (
                slope as f32,
                profile as f32,
                tangent as f32,
                direction_confidence as f32,
                fit_confidence as f32,
            );
        });

    let mut slope = Dfm::new_like(source);
    let mut profile_change = Dfm::new_like(source);
    let mut tangent_change = Dfm::new_like(source);
    let mut direction_confidence = Dfm::new_like(source);
    let mut fit_confidence = Dfm::new_like(source);
    for (index, &(s, profile, tangent, direction, fit)) in values.iter().enumerate() {
        slope.field[index] = s;
        profile_change.field[index] = profile;
        tangent_change.field[index] = tangent;
        direction_confidence.field[index] = direction;
        fit_confidence.field[index] = fit;
    }
    Ok(TerrainDerivatives {
        slope,
        profile_change,
        tangent_change,
        direction_confidence,
        fit_confidence,
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
        let (fit, rmse) = kernel.fit(&source, 10, 10);
        for (actual, expected) in fit.iter().zip([3., 2., -1., 0.4, 0.3, -0.2]) {
            assert!((actual - expected).abs() < 1e-5);
        }
        assert!(rmse < 1e-5);
    }
}
