use std::collections::HashMap;

use geo::{BooleanOps, Distance, Euclidean, Simplify};

use crate::{
    map::{LineSymbol, MapObject},
    parameters::VegetationBoundaryParameters,
    raster::{Dfm, HeightAboveGround, PointDensity},
};

/// Detect ridges of canopy-height gradient, rather than the two outlines of
/// a thresholded slope region. Length filtering happens on the assembled map
/// so that an edge crossing tile boundaries is measured as one continuous line.
pub(crate) fn compute_vegetation_boundaries(
    canopy: &Dfm<HeightAboveGround>,
    point_density: &Dfm<PointDensity>,
    hull: &geo::Polygon,
    cut_overlay: &geo::Polygon,
    params: &VegetationBoundaryParameters,
) -> Vec<MapObject> {
    if !params.enabled
        || !params.smoothing_m.is_finite()
        || !(0.0..=3.).contains(&params.smoothing_m)
        || !params.minimum_height_change_m.is_finite()
        || params.minimum_height_change_m <= 0.
        || !params.minimum_gradient.is_finite()
        || params.minimum_gradient <= 0.
        || !params.minimum_length_m.is_finite()
        || params.minimum_length_m < 0.
        || canopy.width() < 5
        || canopy.height() < 5
    {
        return Vec::new();
    }

    assert!(canopy.grid.same_layout(&point_density.grid));
    let smoothed = smooth_canopy(canopy, point_density, params.smoothing_m);
    let width = canopy.width();
    let height = canopy.height();
    let cell_size = canopy.grid.cell_size_m;
    // Sobel derivatives are in raster coordinates: x right, y down.
    let mut gradients = vec![[0_f32; 2]; width * height];
    let mut strength = Dfm::<HeightAboveGround>::new_like(canopy);
    strength.field.fill(0.);
    for y in 1..height - 1 {
        for x in 1..width - 1 {
            if (y - 1..=y + 1)
                .any(|row| (x - 1..=x + 1).any(|col| !valid_height(smoothed[(row, col)])))
            {
                continue;
            }
            let gx = ((smoothed[(y - 1, x + 1)] - smoothed[(y - 1, x - 1)])
                + 2. * (smoothed[(y, x + 1)] - smoothed[(y, x - 1)])
                + (smoothed[(y + 1, x + 1)] - smoothed[(y + 1, x - 1)]))
                / (8. * cell_size as f32);
            let gy = ((smoothed[(y + 1, x - 1)] - smoothed[(y - 1, x - 1)])
                + 2. * (smoothed[(y + 1, x)] - smoothed[(y - 1, x)])
                + (smoothed[(y + 1, x + 1)] - smoothed[(y - 1, x + 1)]))
                / (8. * cell_size as f32);
            gradients[y * width + x] = [gx, gy];
            strength[(y, x)] = gx.hypot(gy);
        }
    }

    let mut edges = vec![false; width * height];
    let sample_distance = (3. * params.smoothing_m).max(2. * cell_size);
    for y in 2..height - 2 {
        for x in 2..width - 2 {
            let magnitude = strength[(y, x)];
            if magnitude < params.minimum_gradient {
                continue;
            }
            let [gx, gy] = gradients[y * width + x];
            let mut normal = [f64::from(gx / magnitude), f64::from(gy / magnitude)];
            // Canonical normal and asymmetric ties select just one pixel on a
            // flat gradient ridge, independently of which side is taller.
            let dominant = if normal[0].abs() >= normal[1].abs() {
                0
            } else {
                1
            };
            if normal[dominant] < 0. {
                normal = normal.map(|v| -v);
            }
            let coordinate = canopy.index2coord(y, x);
            let offset = |distance: f64| {
                geo::coord! {
                    x: coordinate.x + normal[0] * distance,
                    y: coordinate.y - normal[1] * distance,
                }
            };
            let before = strength.sample_bilinear(offset(-cell_size)).unwrap_or(0.);
            let after = strength.sample_bilinear(offset(cell_size)).unwrap_or(0.);
            if magnitude < before || magnitude <= after {
                continue;
            }
            let (Some(low), Some(high)) = (
                smoothed.sample_bilinear(offset(-sample_distance)),
                smoothed.sample_bilinear(offset(sample_distance)),
            ) else {
                continue;
            };
            if valid_height(low)
                && valid_height(high)
                && (high - low).abs() >= params.minimum_height_change_m
            {
                edges[y * width + x] = true;
            }
        }
    }

    let lines = trace_edges(canopy, &edges, &gradients);
    let lines = hull.clip(&lines, false);
    cut_overlay
        .clip(&lines, false)
        .simplify(crate::SIMPLIFICATION_DIST)
        .into_iter()
        .filter(|line| line.0.len() >= 2)
        .map(|line| {
            let on_seam = |coordinate: geo::Coord| {
                std::iter::once(cut_overlay.exterior())
                    .chain(cut_overlay.interiors())
                    .any(|ring| Euclidean.distance(&geo::Point::from(coordinate), ring) <= 1.0e-6)
            };
            let start = on_seam(line.0[0]);
            let end = on_seam(*line.0.last().unwrap());
            let mut object = MapObject::Line {
                object: line,
                symbol: LineSymbol::ClearVegetationBoundary,
                tags: HashMap::new(),
            };
            object.mark_tile_boundary_endpoints(start, end);
            object
        })
        .collect()
}

fn valid_height(height: f32) -> bool {
    height.is_finite() && height >= 0.
}

/// Normalized Gaussian convolution ignores empty cells rather than treating
/// their stored zero as open ground. Require at least half of the kernel's
/// weight to have observations; larger gaps remain unknown.
fn smooth_canopy(
    canopy: &Dfm<HeightAboveGround>,
    point_density: &Dfm<PointDensity>,
    sigma_m: f64,
) -> Dfm<HeightAboveGround> {
    let mut canopy = canopy.clone();
    for (value, &density) in canopy.field.iter_mut().zip(point_density.field.iter()) {
        if !valid_height(*value) || !density.is_finite() || density <= 0. {
            *value = f32::MIN;
        }
    }
    if sigma_m == 0. {
        return canopy;
    }
    let sigma = sigma_m / canopy.grid.cell_size_m;
    let radius = (3. * sigma).ceil() as usize;
    let mut output = Dfm::new_like(&canopy);
    if radius >= canopy.width().min(canopy.height()) {
        return output;
    }
    let mut weights = (0..=2 * radius)
        .map(|index| (-0.5 * ((index as f64 - radius as f64) / sigma).powi(2)).exp())
        .collect::<Vec<_>>();
    let weight_sum = weights.iter().sum::<f64>();
    for weight in &mut weights {
        *weight /= weight_sum;
    }
    let mut sums = canopy
        .field
        .iter()
        .map(|&value| {
            if valid_height(value) {
                f64::from(value)
            } else {
                0.
            }
        })
        .collect::<Vec<_>>();
    let mut support = canopy
        .field
        .iter()
        .map(|&value| if valid_height(value) { 1. } else { 0. })
        .collect::<Vec<_>>();
    for horizontal in [true, false] {
        let mut next_sums = vec![0.; sums.len()];
        let mut next_support = vec![0.; support.len()];
        for y in 0..canopy.height() {
            for x in 0..canopy.width() {
                let position = if horizontal { x } else { y };
                let extent = if horizontal {
                    canopy.width()
                } else {
                    canopy.height()
                };
                let output_index = y * canopy.width() + x;
                for (index, weight) in weights.iter().enumerate() {
                    let along = position as isize + index as isize - radius as isize;
                    if along < 0 || along >= extent as isize {
                        continue;
                    }
                    let source_index = if horizontal {
                        y * canopy.width() + along as usize
                    } else {
                        along as usize * canopy.width() + x
                    };
                    next_sums[output_index] += sums[source_index] * weight;
                    next_support[output_index] += support[source_index] * weight;
                }
            }
        }
        sums = next_sums;
        support = next_support;
    }
    for index in 0..sums.len() {
        if support[index] >= 0.5 {
            output.field[index] = (sums[index] / support[index]) as f32;
        }
    }
    output
}

fn trace_edges(
    canopy: &Dfm<HeightAboveGround>,
    edges: &[bool],
    gradients: &[[f32; 2]],
) -> geo::MultiLineString {
    let width = canopy.width();
    // Sparse graph: each edge pixel has at most eight neighbors. Avoid
    // diagonal shortcuts around an existing orthogonal connection.
    let mut adjacency = HashMap::<usize, Vec<usize>>::new();
    for (index, &edge) in edges.iter().enumerate() {
        if !edge {
            continue;
        }
        let (y, x) = (index / width, index % width);
        let neighbors = adjacency.entry(index).or_default();
        for dy in -1_isize..=1 {
            for dx in -1_isize..=1 {
                if dx == 0 && dy == 0 {
                    continue;
                }
                let ny = (y as isize + dy) as usize;
                let nx = (x as isize + dx) as usize;
                let next = ny * width + nx;
                if !edges[next]
                    || (dx != 0 && dy != 0 && (edges[y * width + nx] || edges[ny * width + x]))
                {
                    continue;
                }
                let a = gradients[index];
                let b = gradients[next];
                // Join compatible edge directions and retain the same tall
                // side; adjacent opposite sides of a thin crown stay separate.
                if a[0] * b[0] + a[1] * b[1] >= 0.5 * a[0].hypot(a[1]) * b[0].hypot(b[1]) {
                    neighbors.push(next);
                }
            }
        }
    }

    let mut visited = vec![0_u8; edges.len()];
    let bit = |a: usize, b: usize| {
        let dy = (b / width) as isize - (a / width) as isize;
        let dx = (b % width) as isize - (a % width) as isize;
        let direction = ((dy + 1) * 3 + dx + 1) as u32;
        1_u8 << if direction > 4 {
            direction - 1
        } else {
            direction
        }
    };
    let mut lines = Vec::new();
    // Process endpoints/junctions first and remaining closed rings second.
    // Raster order makes output independent of HashMap iteration order.
    for rings in [false, true] {
        for start in 0..edges.len() {
            let Some(neighbors) = adjacency.get(&start) else {
                continue;
            };
            if !rings && neighbors.len() == 2 {
                continue;
            }
            for &next in neighbors {
                if visited[start] & bit(start, next) != 0 {
                    continue;
                }
                let mut indices = vec![start];
                let (mut previous, mut current) = (start, next);
                loop {
                    visited[previous] |= bit(previous, current);
                    visited[current] |= bit(current, previous);
                    indices.push(current);
                    let options = &adjacency[&current];
                    if current == start || options.len() != 2 {
                        break;
                    }
                    let following = if options[0] == previous {
                        options[1]
                    } else {
                        options[0]
                    };
                    if visited[current] & bit(current, following) != 0 {
                        break;
                    }
                    (previous, current) = (current, following);
                }
                // Orient with the taller canopy on the left. Adjacent tiles
                // can then stitch tip to tail without reversing either line.
                let direction = indices
                    .windows(2)
                    .map(|pair| {
                        let (a, b) = (pair[0], pair[1]);
                        let dx = (b % width) as f32 - (a % width) as f32;
                        let dy = (b / width) as f32 - (a / width) as f32;
                        dy * gradients[a][0] - dx * gradients[a][1]
                    })
                    .sum::<f32>();
                if direction < 0. {
                    indices.reverse();
                }
                lines.push(geo::LineString::new(
                    indices
                        .into_iter()
                        .map(|index| canopy.index2coord(index / width, index % width))
                        .collect(),
                ));
            }
        }
    }
    geo::MultiLineString::new(lines)
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::{map::InternalMap, parameters::Scale, raster::DfmGrid};
    use geo::{Contains, Euclidean, Length};

    fn canopy(cell_size: f64, height_at: impl Fn(f64, f64) -> f32) -> Dfm<HeightAboveGround> {
        let size = (64. / cell_size) as usize + 1;
        let grid = DfmGrid::new(size, size, cell_size, geo::coord! { x: 0., y: 64. }).unwrap();
        let mut canopy = Dfm::new(grid);
        for y in 0..size {
            for x in 0..size {
                let point = canopy.index2coord(y, x);
                canopy[(y, x)] = height_at(point.x, point.y);
            }
        }
        canopy
    }

    fn detect(
        canopy: &Dfm<HeightAboveGround>,
        clip: &geo::Polygon,
        params: &VegetationBoundaryParameters,
    ) -> Vec<geo::LineString> {
        let hull = geo::Rect::new((0., 0.), (64., 64.)).to_polygon();
        let mut map = InternalMap::new(geo::coord! { x: 0., y: 0. }, Scale::S15_000, None);
        let mut density = Dfm::<PointDensity>::new_like(canopy);
        density.field.fill(8.);
        for object in compute_vegetation_boundaries(canopy, &density, &hull, clip, params) {
            assert_eq!(
                object.get_symbol(),
                LineSymbol::ClearVegetationBoundary.into()
            );
            map.add_object(object);
        }
        map.merge_lines(0.5);
        map.filter_vegetation_boundaries_min_length(params.minimum_length_m);
        map.objects
            .remove(&LineSymbol::ClearVegetationBoundary.into())
            .unwrap_or_default()
            .into_iter()
            .map(|object| {
                let MapObject::Line { object, .. } = object else {
                    unreachable!()
                };
                object
            })
            .collect()
    }

    fn full_clip() -> geo::Polygon {
        geo::Rect::new((0., 0.), (64., 64.)).to_polygon()
    }

    #[test]
    fn sharp_edges_are_single_lines_in_ground_units_in_both_directions() {
        for cell_size in [0.5, 1.] {
            for horizontal in [false, true] {
                for reverse in [false, true] {
                    let canopy = canopy(cell_size, |x, y| {
                        if (if horizontal { y } else { x } >= 32.) ^ reverse {
                            14.
                        } else {
                            2.
                        }
                    });
                    let lines = detect(
                        &canopy,
                        &full_clip(),
                        &VegetationBoundaryParameters::default(),
                    );
                    assert_eq!(
                        lines.len(),
                        1,
                        "cell={cell_size} horizontal={horizontal} reverse={reverse}, lines={lines:?}"
                    );
                    assert!(Euclidean.length(&lines[0]) > 50.);
                    assert!(lines[0].0.iter().all(|point| {
                        (if horizontal { point.y } else { point.x } - 32.).abs() <= cell_size
                    }));
                    let delta = *lines[0].0.last().unwrap() - *lines[0].0.first().unwrap();
                    assert!(if horizontal {
                        (delta.x > 0.) ^ reverse
                    } else {
                        (delta.y < 0.) ^ reverse
                    });
                }
            }
        }
    }

    #[test]
    fn diagonal_and_curved_edges_remain_continuous() {
        let diagonal = canopy(0.5, |x, y| if x + y >= 64. { 14. } else { 2. });
        let lines = detect(
            &diagonal,
            &full_clip(),
            &VegetationBoundaryParameters::default(),
        );
        assert_eq!(lines.len(), 1, "lines={lines:?}");
        assert!(Euclidean.length(&lines[0]) > 70.);

        let curved = canopy(0.5, |x, y| {
            if x >= 30. + 4. * (y / 12.).sin() {
                14.
            } else {
                2.
            }
        });
        let lines = detect(
            &curved,
            &full_clip(),
            &VegetationBoundaryParameters::default(),
        );
        assert_eq!(lines.len(), 1, "lines={lines:?}");
        assert!(Euclidean.length(&lines[0]) > 50.);
    }

    #[test]
    fn flat_canopy_gradual_transitions_and_small_height_changes_are_rejected() {
        for raster in [
            canopy(0.5, |_, _| 8.),
            canopy(0.5, |x, _| 2. + (x - 16.).clamp(0., 32.) as f32 * 0.375),
            canopy(0.5, |x, _| if x >= 32. { 3. } else { 2. }),
        ] {
            assert!(
                detect(
                    &raster,
                    &full_clip(),
                    &VegetationBoundaryParameters::default()
                )
                .is_empty()
            );
        }
        // Height-change and sharpness controls constrain different things.
        let raster = canopy(0.5, |x, _| if x >= 32. { 14. } else { 2. });
        let mut params = VegetationBoundaryParameters::default();
        params.minimum_height_change_m = 15.;
        assert!(detect(&raster, &full_clip(), &params).is_empty());
        params.minimum_height_change_m = 3.;
        params.minimum_gradient = 20.;
        assert!(detect(&raster, &full_clip(), &params).is_empty());
    }

    #[test]
    fn clipping_and_minimum_length_apply_to_the_emitted_line() {
        let raster = canopy(0.5, |x, _| if x >= 32. { 14. } else { 2. });
        let clip = geo::Rect::new((24., 20.), (40., 35.)).to_polygon();
        let mut params = VegetationBoundaryParameters::default();
        assert!(detect(&raster, &clip, &params).is_empty());
        params.minimum_length_m = 10.;
        let lines = detect(&raster, &clip, &params);
        assert_eq!(lines.len(), 1);
        assert!((Euclidean.length(&lines[0]) - 15.).abs() < 1e-6);
        assert!(
            lines[0]
                .0
                .iter()
                .all(|point| clip.contains(&geo::Point::from(*point))
                    || point.y == 20.
                    || point.y == 35.)
        );
        params.minimum_length_m = 15.;
        assert!(detect(&raster, &clip, &params).is_empty());
    }

    #[test]
    fn missing_data_and_raster_borders_are_not_boundaries() {
        for missing in [f32::MIN, f32::NAN, f32::INFINITY] {
            let raster = canopy(0.5, |x, _| if x >= 32. { missing } else { 10. });
            assert!(
                detect(
                    &raster,
                    &full_clip(),
                    &VegetationBoundaryParameters::default()
                )
                .is_empty()
            );
        }
        let raster = canopy(0.5, |x, y| {
            if (28. ..=36.).contains(&y) {
                f32::MIN
            } else if x >= 32. {
                14.
            } else {
                2.
            }
        });
        let mut params = VegetationBoundaryParameters::default();
        params.minimum_length_m = 5.;
        let lines = detect(&raster, &full_clip(), &params);
        assert_eq!(lines.len(), 2);
        assert!(
            lines
                .iter()
                .all(|line| line.0.iter().all(|p| p.y < 28.) || line.0.iter().all(|p| p.y > 36.))
        );
    }

    #[test]
    fn adjacent_tile_fragments_join_before_length_filtering() {
        let raster = canopy(0.5, |x, _| if x >= 32. { 14. } else { 2. });
        let params = VegetationBoundaryParameters {
            minimum_length_m: 40.,
            ..Default::default()
        };
        let hull = full_clip();
        let mut map = InternalMap::new(geo::coord! { x: 0., y: 0. }, Scale::S15_000, None);
        let mut density = Dfm::<PointDensity>::new_like(&raster);
        density.field.fill(8.);
        for (index, clip) in [
            geo::Rect::new((0., 0.), (64., 32.)).to_polygon(),
            geo::Rect::new((0., 32.), (64., 64.)).to_polygon(),
        ]
        .into_iter()
        .enumerate()
        {
            let mut tile_canopy = raster.clone();
            let mut tile_density = density.clone();
            // Retiling does not guarantee aligned raster origins.
            tile_canopy.grid.top_left.x += index as f64 * 0.2;
            tile_canopy.grid.top_left.y += index as f64 * 0.1;
            tile_density.grid = tile_canopy.grid.clone();
            for object in
                compute_vegetation_boundaries(&tile_canopy, &tile_density, &hull, &clip, &params)
            {
                map.add_object(object);
            }
        }
        let key = LineSymbol::ClearVegetationBoundary.into();
        assert_eq!(map.objects[&key].len(), 2);
        map.merge_lines(0.5);
        map.filter_vegetation_boundaries_min_length(params.minimum_length_m);
        assert_eq!(map.objects[&key].len(), 1);
    }

    #[test]
    fn disabled_and_invalid_controls_emit_nothing() {
        let raster = canopy(0.5, |x, _| if x >= 32. { 14. } else { 2. });
        for params in [
            VegetationBoundaryParameters {
                enabled: false,
                ..Default::default()
            },
            VegetationBoundaryParameters {
                smoothing_m: f64::NAN,
                ..Default::default()
            },
            VegetationBoundaryParameters {
                minimum_gradient: 0.,
                ..Default::default()
            },
            VegetationBoundaryParameters {
                minimum_height_change_m: f32::NAN,
                ..Default::default()
            },
            VegetationBoundaryParameters {
                minimum_length_m: -1.,
                ..Default::default()
            },
        ] {
            assert!(detect(&raster, &full_clip(), &params).is_empty());
        }
    }

    #[test]
    fn smoothing_can_be_disabled_and_short_crown_edges_are_rejected() {
        let raster = canopy(0.5, |x, _| if x >= 32. { 14. } else { 2. });
        let params = VegetationBoundaryParameters {
            smoothing_m: 0.,
            ..Default::default()
        };
        assert_eq!(detect(&raster, &full_clip(), &params).len(), 1);
        let crown = canopy(0.5, |x, y| {
            if (x - 32.).hypot(y - 32.) <= 2. {
                14.
            } else {
                2.
            }
        });
        assert!(detect(&crown, &full_clip(), &params).is_empty());
    }

    #[test]
    fn disconnected_short_edges_cannot_qualify_by_merging_across_a_gap() {
        let mut map = InternalMap::new(geo::coord! { x: 0., y: 0. }, Scale::S15_000, None);
        for (start, end) in [(0., 15.), (15.25, 30.)] {
            map.add_object(MapObject::Line {
                object: geo::LineString::from(vec![(0., start), (0., end)]),
                symbol: LineSymbol::ClearVegetationBoundary,
                tags: HashMap::new(),
            });
        }
        map.merge_lines(0.5);
        map.filter_vegetation_boundaries_min_length(20.);
        assert!(map.objects[&LineSymbol::ClearVegetationBoundary.into()].is_empty());
    }

    #[test]
    fn unobserved_zero_cells_do_not_look_like_open_ground() {
        let raster = canopy(0.5, |x, _| if x >= 32. { 0. } else { 14. });
        let mut density = Dfm::<PointDensity>::new_like(&raster);
        for y in 0..density.height() {
            for x in 0..density.width() {
                density[(y, x)] = if density.index2coord(y, x).x >= 32. {
                    0.
                } else {
                    8.
                };
            }
        }
        let params = VegetationBoundaryParameters::default();
        assert!(
            compute_vegetation_boundaries(&raster, &density, &full_clip(), &full_clip(), &params)
                .is_empty()
        );
        density.field.fill(8.);
        assert!(
            !compute_vegetation_boundaries(&raster, &density, &full_clip(), &full_clip(), &params)
                .is_empty()
        );
    }

    #[test]
    fn scattered_empty_cells_are_ignored_during_canopy_smoothing() {
        let mut raster = canopy(0.5, |x, _| if x >= 32. { 14. } else { 2. });
        let mut density = Dfm::<PointDensity>::new_like(&raster);
        density.field.fill(8.);
        for y in 0..raster.height() {
            for x in 0..raster.width() {
                if (x + 3 * y) % 5 == 0 {
                    raster[(y, x)] = 0.;
                    density[(y, x)] = 0.;
                }
            }
        }
        let objects = compute_vegetation_boundaries(
            &raster,
            &density,
            &full_clip(),
            &full_clip(),
            &VegetationBoundaryParameters::default(),
        );
        assert!(objects.iter().any(|object| matches!(object, MapObject::Line { object, .. } if Euclidean.length(object) > 50.)));
    }
}
