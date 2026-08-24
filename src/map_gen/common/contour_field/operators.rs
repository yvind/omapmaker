use rayon::prelude::*;
use std::f32::consts::SQRT_2;

pub(super) fn apply(
    source: &[f32],
    width: usize,
    height: usize,
    cell: f32,
    gradient: &mut [[f32; 2]],
    hessian: &mut [[f32; 3]],
) {
    assert_eq!(source.len(), width * height);
    assert_eq!(gradient.len(), source.len());
    assert_eq!(hessian.len(), source.len());
    assert!(width > 0 && height > 0 && cell > 0.);
    let cell_squared = cell * cell;
    gradient
        .par_chunks_mut(width)
        .zip(hessian.par_chunks_mut(width))
        .enumerate()
        .for_each(|(y, (gradient_row, hessian_row))| {
            if y > 0 && y + 1 < height && width > 2 {
                apply_boundary_cell(
                    source,
                    width,
                    height,
                    cell,
                    cell_squared,
                    y,
                    0,
                    &mut gradient_row[0],
                    &mut hessian_row[0],
                );
                for x in 1..width - 1 {
                    let i = y * width + x;
                    gradient_row[x] = [
                        (source[i + 1] - source[i]) / cell,
                        (source[i + width] - source[i]) / cell,
                    ];
                    hessian_row[x] = [
                        (source[i - 1] - 2. * source[i] + source[i + 1]) / cell_squared,
                        SQRT_2
                            * (source[i - width + 1]
                                - source[i - width - 1]
                                - source[i + width + 1]
                                + source[i + width - 1])
                            / (4. * cell_squared),
                        (source[i - width] - 2. * source[i] + source[i + width]) / cell_squared,
                    ];
                }
                apply_boundary_cell(
                    source,
                    width,
                    height,
                    cell,
                    cell_squared,
                    y,
                    width - 1,
                    &mut gradient_row[width - 1],
                    &mut hessian_row[width - 1],
                );
            } else {
                for x in 0..width {
                    apply_boundary_cell(
                        source,
                        width,
                        height,
                        cell,
                        cell_squared,
                        y,
                        x,
                        &mut gradient_row[x],
                        &mut hessian_row[x],
                    );
                }
            }
        });
}

#[allow(clippy::too_many_arguments)]
fn apply_boundary_cell(
    source: &[f32],
    width: usize,
    height: usize,
    cell: f32,
    cell_squared: f32,
    y: usize,
    x: usize,
    gradient: &mut [f32; 2],
    hessian: &mut [f32; 3],
) {
    let top = y.saturating_sub(1);
    let bottom = (y + 1).min(height - 1);
    let left = x.saturating_sub(1);
    let right = (x + 1).min(width - 1);
    let i = y * width + x;
    *gradient = [
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
    *hessian = [
        (source[y * width + left] - 2. * source[i] + source[y * width + right]) / cell_squared,
        SQRT_2
            * (source[top * width + right]
                - source[top * width + left]
                - source[bottom * width + right]
                + source[bottom * width + left])
            / (4. * cell_squared),
        (source[top * width + x] - 2. * source[i] + source[bottom * width + x]) / cell_squared,
    ];
}

#[cfg(test)]
pub(super) fn adjoint(
    gradient: &[[f32; 2]],
    hessian: &[[f32; 3]],
    width: usize,
    height: usize,
    cell: f32,
    output: &mut [f32],
) {
    adjoint_impl(gradient, hessian, None, width, height, cell, output);
}

pub(super) struct AlignmentAdjoint<'a> {
    pub(super) dual: &'a [f32],
    pub(super) tangent_x: &'a [f32],
    pub(super) tangent_y: &'a [f32],
}

pub(super) fn adjoint_with_alignment(
    gradient: &[[f32; 2]],
    hessian: &[[f32; 3]],
    alignment: AlignmentAdjoint<'_>,
    width: usize,
    height: usize,
    cell: f32,
    output: &mut [f32],
) {
    assert_eq!(alignment.dual.len(), gradient.len());
    assert_eq!(alignment.tangent_x.len(), gradient.len());
    assert_eq!(alignment.tangent_y.len(), gradient.len());
    adjoint_impl(
        gradient,
        hessian,
        Some(alignment),
        width,
        height,
        cell,
        output,
    );
}

fn adjoint_impl(
    gradient: &[[f32; 2]],
    hessian: &[[f32; 3]],
    alignment: Option<AlignmentAdjoint<'_>>,
    width: usize,
    height: usize,
    cell: f32,
    output: &mut [f32],
) {
    assert_eq!(gradient.len(), width * height);
    assert_eq!(hessian.len(), gradient.len());
    assert_eq!(output.len(), gradient.len());
    assert!(width > 0 && height > 0 && cell > 0.);
    let inverse_cell = cell.recip();
    let cell_squared = cell * cell;
    let inverse_cell_squared = cell_squared.recip();
    let mixed_scale = SQRT_2 / (4. * cell_squared);
    output
        .par_chunks_mut(width)
        .enumerate()
        .for_each(|(y, row)| {
            for (x, value) in row.iter_mut().enumerate() {
                let i = y * width + x;
                let mut result = 0.;
                let aligned_x = |index: usize| {
                    gradient[index][0]
                        + alignment.as_ref().map_or(0., |alignment| {
                            alignment.dual[index] * alignment.tangent_x[index]
                        })
                };
                let aligned_y = |index: usize| {
                    gradient[index][1]
                        + alignment.as_ref().map_or(0., |alignment| {
                            alignment.dual[index] * alignment.tangent_y[index]
                        })
                };
                if x > 0 {
                    result += aligned_x(i - 1) * inverse_cell;
                }
                if x + 1 < width {
                    result -= aligned_x(i) * inverse_cell;
                }
                if y > 0 {
                    result += aligned_y(i - width) * inverse_cell;
                }
                if y + 1 < height {
                    result -= aligned_y(i) * inverse_cell;
                }

                let left = if x > 0 { i - 1 } else { i };
                let right = if x + 1 < width { i + 1 } else { i };
                result += (hessian[left][0] - 2. * hessian[i][0] + hessian[right][0])
                    * inverse_cell_squared;
                let top = if y > 0 { i - width } else { i };
                let bottom = if y + 1 < height { i + width } else { i };
                result += (hessian[top][2] - 2. * hessian[i][2] + hessian[bottom][2])
                    * inverse_cell_squared;

                if x > 0 && x + 1 < width && y > 0 && y + 1 < height {
                    result += (hessian[(y - 1) * width + x + 1][1]
                        - hessian[(y - 1) * width + x - 1][1]
                        - hessian[(y + 1) * width + x + 1][1]
                        + hessian[(y + 1) * width + x - 1][1])
                        * mixed_scale;
                } else {
                    let (x_terms, x_count) = difference_adjoint_terms(x, width);
                    let (mut y_terms, y_count) = difference_adjoint_terms(y, height);
                    for term in &mut y_terms[..y_count] {
                        term.1 = -term.1;
                    }
                    for &(source_y, y_weight) in &y_terms[..y_count] {
                        for &(source_x, x_weight) in &x_terms[..x_count] {
                            result += hessian[source_y * width + source_x][1]
                                * (x_weight * y_weight * mixed_scale);
                        }
                    }
                }
                *value = result;
            }
        });
}

fn difference_adjoint_terms(index: usize, len: usize) -> ([(usize, f32); 4], usize) {
    let mut terms = [(0, 0.); 4];
    let mut count = 0;
    if len == 1 {
        terms[count] = (0, 1.);
        count += 1;
    } else if index + 1 == len {
        terms[count] = (len - 2, 1.);
        count += 1;
        terms[count] = (len - 1, 1.);
        count += 1;
    } else if index > 0 {
        terms[count] = (index - 1, 1.);
        count += 1;
    }

    if len == 1 {
        terms[count] = (0, -1.);
        count += 1;
    } else if index == 0 {
        terms[count] = (0, -1.);
        count += 1;
        terms[count] = (1, -1.);
        count += 1;
    } else if index + 1 < len {
        terms[count] = (index + 1, -1.);
        count += 1;
    }
    (terms, count)
}

pub(super) fn norm(width: usize, height: usize, cell: f32) -> f32 {
    assert!(width > 0 && height > 0 && cell > 0.);
    let inverse_cell_squared = cell.recip().powi(2);
    // ||Dx|| and ||Dy|| are at most 2/c. The source-isoline directional
    // derivative is bounded by the full gradient norm. The replicated-edge second
    // differences are at most 4/c², and the scaled mixed derivative is at
    // most sqrt(2)/c². For the vertically stacked operator, the square of its
    // norm is bounded by the sum of those squared component bounds.
    (16. * inverse_cell_squared + 34. * inverse_cell_squared.powi(2)).sqrt()
}

#[cfg(test)]
mod tests {
    use super::*;
    use rayon::ThreadPoolBuilder;

    fn scatter_adjoint(
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

    #[test]
    fn gather_adjoint_matches_scatter_on_boundaries_and_degenerate_grids() {
        for (width, height) in [(1, 1), (1, 5), (5, 1), (2, 2), (7, 5)] {
            let len = width * height;
            let gradient = (0..len)
                .map(|i| [(i as f32 * 0.21).cos(), (i as f32 * 0.13).sin()])
                .collect::<Vec<_>>();
            let hessian = (0..len)
                .map(|i| {
                    [
                        (i as f32 * 0.11).sin(),
                        (i as f32 * 0.17).cos(),
                        (i as f32 * 0.19).sin(),
                    ]
                })
                .collect::<Vec<_>>();
            let mut expected = vec![0.; len];
            let mut actual = vec![0.; len];
            scatter_adjoint(&gradient, &hessian, width, height, 0.5, &mut expected);
            adjoint(&gradient, &hessian, width, height, 0.5, &mut actual);
            for (index, (&expected, &actual)) in expected.iter().zip(&actual).enumerate() {
                assert!(
                    (expected - actual).abs() < 2e-5,
                    "{width}x{height} cell {index}: {expected} != {actual}"
                );
            }
        }
    }

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

    #[test]
    fn source_isoline_alignment_operator_has_exact_adjoint() {
        let (width, height) = (7, 5);
        let source = (0..width * height)
            .map(|i| (i as f32 * 0.37).sin())
            .collect::<Vec<_>>();
        let gradient_dual = (0..source.len())
            .map(|i| [(i as f32 * 0.21).cos(), (i as f32 * 0.13).sin()])
            .collect::<Vec<_>>();
        let hessian_dual = (0..source.len())
            .map(|i| {
                [
                    (i as f32 * 0.11).sin(),
                    (i as f32 * 0.17).cos(),
                    (i as f32 * 0.19).sin(),
                ]
            })
            .collect::<Vec<_>>();
        let alignment_dual = (0..source.len())
            .map(|i| (i as f32 * 0.29).cos())
            .collect::<Vec<_>>();
        let tangent_x = (0..source.len())
            .map(|i| (i as f32 * 0.07).cos())
            .collect::<Vec<_>>();
        let tangent_y = tangent_x
            .iter()
            .map(|x| (1. - x * x).max(0.).sqrt())
            .collect::<Vec<_>>();
        let mut gradient = vec![[0.; 2]; source.len()];
        let mut hessian = vec![[0.; 3]; source.len()];
        apply(&source, width, height, 0.5, &mut gradient, &mut hessian);
        let left = gradient
            .iter()
            .zip(&gradient_dual)
            .zip(&alignment_dual)
            .zip(&tangent_x)
            .zip(&tangent_y)
            .map(|((((value, dual), alignment), tx), ty)| {
                value[0] * dual[0]
                    + value[1] * dual[1]
                    + alignment * (tx * value[0] + ty * value[1])
            })
            .sum::<f32>()
            + hessian
                .iter()
                .zip(&hessian_dual)
                .map(|(a, b)| a[0] * b[0] + a[1] * b[1] + a[2] * b[2])
                .sum::<f32>();
        let mut adjoint_value = vec![0.; source.len()];
        adjoint_with_alignment(
            &gradient_dual,
            &hessian_dual,
            AlignmentAdjoint {
                dual: &alignment_dual,
                tangent_x: &tangent_x,
                tangent_y: &tangent_y,
            },
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

    #[test]
    fn parallel_operators_are_thread_count_deterministic() {
        let (width, height) = (31, 23);
        let source = (0..width * height)
            .map(|i| (i as f32 * 0.37).sin())
            .collect::<Vec<_>>();
        let run = |threads| {
            ThreadPoolBuilder::new()
                .num_threads(threads)
                .build()
                .unwrap()
                .install(|| {
                    let mut gradient = vec![[0.; 2]; source.len()];
                    let mut hessian = vec![[0.; 3]; source.len()];
                    let mut output = vec![0.; source.len()];
                    apply(&source, width, height, 0.5, &mut gradient, &mut hessian);
                    adjoint(&gradient, &hessian, width, height, 0.5, &mut output);
                    (gradient, hessian, output)
                })
        };
        assert_eq!(run(1), run(4));
    }

    #[test]
    fn analytic_norm_bounds_sampled_operator_ratios() {
        for (width, height, cell) in [(1, 1, 0.5), (7, 5, 0.5), (32, 24, 2.)] {
            let source = (0..width * height)
                .map(|i| ((i * 37 % 101) as f32 - 50.) / 50.)
                .collect::<Vec<_>>();
            let mut gradient = vec![[0.; 2]; source.len()];
            let mut hessian = vec![[0.; 3]; source.len()];
            apply(&source, width, height, cell, &mut gradient, &mut hessian);
            let input_norm = source.iter().map(|value| value * value).sum::<f32>().sqrt();
            let output_norm = gradient
                .iter()
                .map(|value| value[0].powi(2) + value[1].powi(2))
                .chain(
                    hessian
                        .iter()
                        .map(|value| value[0].powi(2) + value[1].powi(2) + value[2].powi(2)),
                )
                .sum::<f32>()
                .sqrt();
            assert!(output_norm <= norm(width, height, cell) * input_norm + 1e-5);
        }
    }
}
