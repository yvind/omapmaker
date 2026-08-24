use std::collections::HashMap;
use std::f32::consts::SQRT_2;
use std::sync::{Mutex, OnceLock};

pub(super) fn apply(
    source: &[f32],
    width: usize,
    height: usize,
    cell: f32,
    gradient: &mut [[f32; 2]],
    hessian: &mut [[f32; 3]],
) {
    let cell_squared = cell * cell;
    for y in 0..height {
        let top = y.saturating_sub(1);
        let bottom = (y + 1).min(height - 1);
        for x in 0..width {
            let left = x.saturating_sub(1);
            let right = (x + 1).min(width - 1);
            let i = y * width + x;
            gradient[i] = [
                if x + 1 < width {
                    (source[i + 1] - source[i]) / cell
                } else {
                    0.
                },
                if y + 1 < height {
                    (source[i + width] - source[i]) / cell
                } else {
                    0.
                },
            ];
            hessian[i] = [
                (source[y * width + left] - 2. * source[i] + source[y * width + right])
                    / cell_squared,
                SQRT_2
                    * (source[top * width + right]
                        - source[top * width + left]
                        - source[bottom * width + right]
                        + source[bottom * width + left])
                    / (4. * cell_squared),
                (source[top * width + x] - 2. * source[i] + source[bottom * width + x])
                    / cell_squared,
            ];
        }
    }
}

pub(super) fn adjoint(
    gradient: &[[f32; 2]],
    hessian: &[[f32; 3]],
    width: usize,
    height: usize,
    cell: f32,
    output: &mut [f32],
) {
    output.fill(0.);
    let cell_squared = cell * cell;
    for y in 0..height {
        let top = y.saturating_sub(1);
        let bottom = (y + 1).min(height - 1);
        for x in 0..width {
            let left = x.saturating_sub(1);
            let right = (x + 1).min(width - 1);
            let i = y * width + x;
            if x + 1 < width {
                output[i] -= gradient[i][0] / cell;
                output[i + 1] += gradient[i][0] / cell;
            }
            if y + 1 < height {
                output[i] -= gradient[i][1] / cell;
                output[i + width] += gradient[i][1] / cell;
            }
            let xx = hessian[i][0] / cell_squared;
            output[y * width + left] += xx;
            output[i] -= 2. * xx;
            output[y * width + right] += xx;
            let xy = hessian[i][1] * SQRT_2 / (4. * cell_squared);
            output[top * width + right] += xy;
            output[top * width + left] -= xy;
            output[bottom * width + right] -= xy;
            output[bottom * width + left] += xy;
            let yy = hessian[i][2] / cell_squared;
            output[top * width + x] += yy;
            output[i] -= 2. * yy;
            output[bottom * width + x] += yy;
        }
    }
}

pub(super) fn norm(width: usize, height: usize, cell: f32) -> f32 {
    type NormCache = Mutex<HashMap<(usize, usize, u32), f32>>;
    static CACHE: OnceLock<NormCache> = OnceLock::new();
    let key = (width, height, cell.to_bits());
    if let Some(norm) = CACHE
        .get_or_init(Default::default)
        .lock()
        .expect("operator-norm cache poisoned")
        .get(&key)
        .copied()
    {
        return norm;
    }
    let len = width * height;
    let mut value = (0..len)
        .map(|i| ((i * 37 % 101) as f32 - 50.) / 50.)
        .collect::<Vec<_>>();
    let mut gradient = vec![[0.; 2]; len];
    let mut hessian = vec![[0.; 3]; len];
    let mut adjoint_value = vec![0.; len];
    let mut estimate = 1.;
    for _ in 0..30 {
        let length = value.iter().map(|x| x * x).sum::<f32>().sqrt().max(1e-12);
        value.iter_mut().for_each(|x| *x /= length);
        apply(&value, width, height, cell, &mut gradient, &mut hessian);
        adjoint(&gradient, &hessian, width, height, cell, &mut adjoint_value);
        estimate = adjoint_value.iter().map(|x| x * x).sum::<f32>().sqrt();
        std::mem::swap(&mut value, &mut adjoint_value);
    }
    let norm = estimate.sqrt();
    CACHE
        .get_or_init(Default::default)
        .lock()
        .expect("operator-norm cache poisoned")
        .insert(key, norm);
    norm
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn combined_operator_has_exact_adjoint() {
        let (width, height) = (7, 5);
        let source = (0..width * height)
            .map(|i| (i as f32 * 0.37).sin())
            .collect::<Vec<_>>();
        let gradient_dual = (0..width * height)
            .map(|i| [(i as f32 * 0.21).cos(), (i as f32 * 0.13).sin()])
            .collect::<Vec<_>>();
        let hessian_dual = (0..width * height)
            .map(|i| {
                [
                    (i as f32 * 0.11).sin(),
                    (i as f32 * 0.17).cos(),
                    (i as f32 * 0.19).sin(),
                ]
            })
            .collect::<Vec<_>>();
        let mut gradient = vec![[0.; 2]; source.len()];
        let mut hessian = vec![[0.; 3]; source.len()];
        apply(&source, width, height, 0.5, &mut gradient, &mut hessian);
        let left = gradient
            .iter()
            .zip(&gradient_dual)
            .map(|(a, b)| a[0] * b[0] + a[1] * b[1])
            .sum::<f32>()
            + hessian
                .iter()
                .zip(&hessian_dual)
                .map(|(a, b)| a[0] * b[0] + a[1] * b[1] + a[2] * b[2])
                .sum::<f32>();
        let mut adjoint_value = vec![0.; source.len()];
        adjoint(
            &gradient_dual,
            &hessian_dual,
            width,
            height,
            0.5,
            &mut adjoint_value,
        );
        let right = source
            .iter()
            .zip(adjoint_value)
            .map(|(a, b)| a * b)
            .sum::<f32>();
        assert!((left - right).abs() < 2e-3, "{left} != {right}");
    }
}
