use crate::{
    CELL_SIZE_METERS, TILE_SIZE_PIXELS,
    geometry::PointCloud,
    parameters::{BufferDirection, BufferRule},
    raster::{
        D8Flow, Dfm, Elevation, FloodFill, HeightAboveGround, HydroCorrected, Intensity, Water,
    },
};

// A 3 m neighborhood is large enough to estimate a plane even where water
// returns are sparse, while still preserving the banks of small ponds.
const WATER_RADIUS_METERS: f64 = 3.;
const MIN_PLANE_RETURNS: f64 = 6.;
const PLANAR_RMSE_METERS: f64 = 0.10;
// Water surfaces are level. A slope of 0.015 is approximately a 0.86 degree
// incline and already receives a strong penalty.
const LEVEL_PLANE_SLOPE: f64 = 0.015;
// Canopy-height values above this scale quickly suppress a water candidate.
const WATER_HAG_SCALE_METERS: f64 = 0.5;
const WATER_MEDIAN_RADIUS_CELLS: usize = 1;

/// Estimate the probability of water in every raster cell.
///
/// Only single returns are used for the plane fit. Low normalized intensity is
/// the primary water signal. Canopy height, plane residual, and plane slope can
/// only reduce that signal, after which a 3x3 median filter removes isolated
/// raster noise.
pub fn compute_water_probability(
    all_point_cloud: &PointCloud,
    dem: &Dfm<Elevation>,
    normalized_intensity: &Dfm<Intensity>,
    canopy_height: &Dfm<HeightAboveGround>,
) -> Dfm<Water> {
    dem.grid
        .ensure_compatible(&normalized_intensity.grid)
        .expect("water probability requires matching intensity and elevation grids");
    dem.grid
        .ensure_compatible(&canopy_height.grid)
        .expect("water probability requires matching canopy-height and elevation grids");

    let side = TILE_SIZE_PIXELS;
    let len = side * side;
    let mut count = vec![0.; len];
    let mut x = vec![0.; len];
    let mut y = vec![0.; len];
    let mut z = vec![0.; len];
    let mut xx = vec![0.; len];
    let mut xy = vec![0.; len];
    let mut yy = vec![0.; len];
    let mut xz = vec![0.; len];
    let mut yz = vec![0.; len];
    let mut zz = vec![0.; len];

    for point in all_point_cloud
        .points
        .iter()
        .filter(|point| point.0.return_number == 1 && point.0.number_of_returns == 1)
    {
        let xi = ((point.x() - dem.grid.top_left.x) / dem.grid.cell_size_m).round() as isize;
        let yi = ((dem.grid.top_left.y - point.y()) / dem.grid.cell_size_m).round() as isize;
        if xi < 0 || yi < 0 || xi >= side as isize || yi >= side as isize {
            continue;
        }

        let index = yi as usize * side + xi as usize;
        // Work in tile-local coordinates to keep the plane-fit covariance
        // numerically stable even when the source CRS has large coordinates.
        let px = point.x() - dem.grid.top_left.x;
        let py = dem.grid.top_left.y - point.y();
        let pz = point.0.z;
        count[index] += 1.;
        x[index] += px;
        y[index] += py;
        z[index] += pz;
        xx[index] += px * px;
        xy[index] += px * py;
        yy[index] += py * py;
        xz[index] += px * pz;
        yz[index] += py * pz;
        zz[index] += pz * pz;
    }

    let fields = [count, x, y, z, xx, xy, yy, xz, yz, zz].map(|field| summed_area_table(&field));
    let radius = (WATER_RADIUS_METERS / CELL_SIZE_METERS).ceil() as usize;
    let mut water = Dfm::<Water>::new_like(dem);

    for yi in 0..side {
        let top = yi.saturating_sub(radius);
        let bottom = (yi + radius + 1).min(side);
        for xi in 0..side {
            let left = xi.saturating_sub(radius);
            let right = (xi + radius + 1).min(side);
            let sums = fields
                .each_ref()
                .map(|field| rectangle_sum(field, top, bottom, left, right));
            let [
                n,
                sx,
                sy,
                sz,
                sxx_raw,
                sxy_raw,
                syy_raw,
                sxz_raw,
                syz_raw,
                szz_raw,
            ] = sums;

            if n < MIN_PLANE_RETURNS {
                water[(yi, xi)] = 0.;
                continue;
            }

            let sxx = sxx_raw - sx * sx / n;
            let sxy = sxy_raw - sx * sy / n;
            let syy = syy_raw - sy * sy / n;
            let sxz = sxz_raw - sx * sz / n;
            let syz = syz_raw - sy * sz / n;
            let szz = (szz_raw - sz * sz / n).max(0.);
            let determinant = sxx * syy - sxy * sxy;
            if determinant <= f64::EPSILON {
                water[(yi, xi)] = 0.;
                continue;
            }

            let plane_x = (sxz * syy - syz * sxy) / determinant;
            let plane_y = (syz * sxx - sxz * sxy) / determinant;
            let residual_sum = (szz - plane_x * sxz - plane_y * syz).max(0.);
            let plane_rmse = (residual_sum / n).sqrt();
            let plane_slope = plane_x.hypot(plane_y);

            water[(yi, xi)] = water_likelihood(
                normalized_intensity[(yi, xi)],
                canopy_height[(yi, xi)],
                plane_rmse,
                plane_slope,
            );
        }
    }

    median_filter(&water, WATER_MEDIAN_RADIUS_CELLS)
}

/// Buffer high-confidence seeds, expand them to their complete level extent,
/// and optionally continue that extent along the existing D8 flow field.
pub fn compute_water_extent(
    water_probability: &Dfm<Water>,
    hydro_corrected: &Dfm<HydroCorrected>,
    flow: &D8Flow,
    seed_threshold: f32,
    elevation_tolerance_m: f32,
    seed_buffer_rules: &[BufferRule],
    allow_downhill_flow: bool,
) -> Dfm<FloodFill> {
    let mut seed_mask = Dfm::<FloodFill>::new_like(water_probability);
    for (seed, probability) in seed_mask.field.iter_mut().zip(&water_probability.field) {
        *seed = if probability.is_finite() && *probability >= seed_threshold {
            1.
        } else {
            0.
        };
    }
    apply_buffer_rules(&mut seed_mask, seed_buffer_rules);

    let generators = seed_mask
        .field
        .iter()
        .enumerate()
        .filter(|(_, seed)| **seed == 1.)
        .map(|(index, _)| seed_mask.index2coord(index / TILE_SIZE_PIXELS, index % TILE_SIZE_PIXELS))
        .collect();

    let mut extent = hydro_corrected.flood_fill(generators, elevation_tolerance_m, false);
    if allow_downhill_flow {
        flow.extend_mask_downstream(&mut extent);
    }
    extent
}

fn water_likelihood(
    normalized_intensity: f32,
    canopy_height_m: f32,
    plane_rmse: f64,
    plane_slope: f64,
) -> f32 {
    let intensity_score = 1. - f64::from(normalized_intensity).clamp(0., 1.);
    let hag_score = (-(f64::from(canopy_height_m).max(0.) / WATER_HAG_SCALE_METERS).powi(2)).exp();
    let planarity_score = (-(plane_rmse / PLANAR_RMSE_METERS).powi(2)).exp();
    let level_score = (-(plane_slope / LEVEL_PLANE_SLOPE).powi(2)).exp();

    // Intensity sets a hard ceiling on confidence. The remaining evidence is
    // deliberately expressed only as a penalty: vegetation, off-plane
    // returns, or a sloping plane can never rescue a bright candidate.
    (intensity_score * (hag_score * planarity_score * level_score).sqrt()).clamp(0., 1.) as f32
}

fn median_filter(source: &Dfm<Water>, radius: usize) -> Dfm<Water> {
    let mut output = Dfm::<Water>::new_like(source);
    let diameter = radius * 2 + 1;
    let mut values = Vec::with_capacity(diameter * diameter);

    for y in 0..source.height() {
        let top = y.saturating_sub(radius);
        let bottom = (y + radius).min(source.height() - 1);
        for x in 0..source.width() {
            let left = x.saturating_sub(radius);
            let right = (x + radius).min(source.width() - 1);
            values.clear();
            for row in top..=bottom {
                for column in left..=right {
                    let value = source[(row, column)];
                    if value.is_finite() {
                        values.push(value);
                    }
                }
            }
            values.sort_unstable_by(f32::total_cmp);
            output[(y, x)] = values.get(values.len() / 2).copied().unwrap_or(0.);
        }
    }

    output
}

fn apply_buffer_rules(mask: &mut Dfm<FloodFill>, rules: &[BufferRule]) {
    for rule in rules {
        if !rule.amount.is_finite() || rule.amount <= 0. {
            continue;
        }

        let grow = rule.direction == BufferDirection::Grow;
        let sources = mask
            .field
            .iter()
            .map(|value| (*value == 1.) == grow)
            .collect::<Vec<_>>();
        let width = mask.width();
        let height = mask.height();
        let distance_squared = squared_distance_transform(&sources, width, height);
        let radius_cells_squared = (rule.amount / mask.grid.cell_size_m).powi(2);

        for (index, value) in mask.field.iter_mut().enumerate() {
            *value = if grow {
                if distance_squared[index] <= radius_cells_squared {
                    1.
                } else {
                    0.
                }
            } else {
                let x = index % width;
                let y = index / width;
                let outside_distance_squared =
                    (x + 1).min(y + 1).min(width - x).min(height - y).pow(2) as f64;
                if *value == 1.
                    && distance_squared[index].min(outside_distance_squared) > radius_cells_squared
                {
                    1.
                } else {
                    0.
                }
            };
        }
    }
}

/// Exact squared Euclidean distance, in cells, to the nearest `true` source.
fn squared_distance_transform(sources: &[bool], width: usize, height: usize) -> Vec<f64> {
    const INF: f64 = 1.0e20;
    let mut horizontal = vec![INF; sources.len()];
    let mut input = vec![INF; width.max(height)];
    let mut output = vec![INF; width.max(height)];

    for y in 0..height {
        for x in 0..width {
            input[x] = if sources[y * width + x] { 0. } else { INF };
        }
        squared_distance_transform_1d(&input[..width], &mut output[..width]);
        horizontal[y * width..(y + 1) * width].copy_from_slice(&output[..width]);
    }

    let mut distances = vec![INF; sources.len()];
    for x in 0..width {
        for y in 0..height {
            input[y] = horizontal[y * width + x];
        }
        squared_distance_transform_1d(&input[..height], &mut output[..height]);
        for y in 0..height {
            distances[y * width + x] = output[y];
        }
    }
    distances
}

fn squared_distance_transform_1d(input: &[f64], output: &mut [f64]) {
    let mut locations = vec![0_usize; input.len()];
    let mut boundaries = vec![0_f64; input.len() + 1];
    let mut envelope = 0_usize;
    locations[0] = 0;
    boundaries[0] = f64::NEG_INFINITY;
    boundaries[1] = f64::INFINITY;

    for q in 1..input.len() {
        let qf = q as f64;
        let mut location = locations[envelope];
        let mut intersection = ((input[q] + qf * qf)
            - (input[location] + (location * location) as f64))
            / (2. * (q - location) as f64);
        while intersection <= boundaries[envelope] {
            envelope -= 1;
            location = locations[envelope];
            intersection = ((input[q] + qf * qf)
                - (input[location] + (location * location) as f64))
                / (2. * (q - location) as f64);
        }
        envelope += 1;
        locations[envelope] = q;
        boundaries[envelope] = intersection;
        boundaries[envelope + 1] = f64::INFINITY;
    }

    envelope = 0;
    for (q, value) in output.iter_mut().enumerate() {
        while boundaries[envelope + 1] < q as f64 {
            envelope += 1;
        }
        let delta = q.abs_diff(locations[envelope]) as f64;
        *value = delta * delta + input[locations[envelope]];
    }
}

fn summed_area_table(values: &[f64]) -> Vec<f64> {
    let stride = TILE_SIZE_PIXELS + 1;
    let mut table = vec![0.; stride * stride];
    for y in 0..TILE_SIZE_PIXELS {
        let mut row_sum = 0.;
        for x in 0..TILE_SIZE_PIXELS {
            row_sum += values[y * TILE_SIZE_PIXELS + x];
            table[(y + 1) * stride + x + 1] = table[y * stride + x + 1] + row_sum;
        }
    }
    table
}

fn rectangle_sum(table: &[f64], top: usize, bottom: usize, left: usize, right: usize) -> f64 {
    let stride = TILE_SIZE_PIXELS + 1;
    table[bottom * stride + right] + table[top * stride + left]
        - table[top * stride + right]
        - table[bottom * stride + left]
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::raster::DfmGrid;

    fn flow_for(corrected: &Dfm<HydroCorrected>) -> D8Flow {
        let mut source = Dfm::<Elevation>::new(corrected.grid.clone());
        source.field.clone_from_slice(&corrected.field);
        source.hydrological_analysis_with_corrected(corrected)
    }

    #[test]
    fn probability_filter_only_selects_seeds_for_the_final_extent() {
        let mut corrected =
            Dfm::<HydroCorrected>::new(crate::raster::DfmGrid::standard(geo::Coord {
                x: 0.,
                y: 100.,
            }));
        corrected.field.fill(10.);
        for y in 20..=22 {
            for x in 30..=32 {
                corrected[(y, x)] = 2.;
            }
        }

        let mut probability = Dfm::<Water>::new_like(&corrected);
        probability.field.fill(0.);
        probability[(21, 31)] = 0.8;

        let flow = flow_for(&corrected);
        let extent = compute_water_extent(&probability, &corrected, &flow, 0.65, 0.05, &[], false);

        assert_eq!(extent.field.iter().filter(|value| **value == 1.).count(), 9);
        assert_eq!(extent[(20, 30)], 1.);
        assert_eq!(probability[(20, 30)], 0.);
    }

    #[test]
    fn intensity_is_primary_and_other_evidence_only_penalizes_it() {
        let ideal = water_likelihood(0.1, 0., 0., 0.);
        let bright = water_likelihood(0.8, 0., 0., 0.);
        let canopy = water_likelihood(0.1, 0.5, 0., 0.);
        let off_plane = water_likelihood(0.1, 0., PLANAR_RMSE_METERS, 0.);
        let sloping = water_likelihood(0.1, 0., 0., LEVEL_PLANE_SLOPE);

        assert!((ideal - 0.9).abs() < 1.0e-6);
        assert!(bright < ideal);
        assert!(canopy < ideal);
        assert!(off_plane < ideal);
        assert!(sloping < ideal);
        assert!(bright <= 0.2 + f32::EPSILON);
    }

    #[test]
    fn median_filter_removes_an_isolated_candidate() {
        let grid = DfmGrid::new(5, 5, 1., geo::coord! { x: 0., y: 5. }).unwrap();
        let mut probability = Dfm::<Water>::new(grid);
        probability.field.fill(0.);
        probability[(2, 2)] = 1.;

        let filtered = median_filter(&probability, 1);

        assert_eq!(filtered[(2, 2)], 0.);
        assert!(filtered.field.iter().all(|value| *value == 0.));
    }

    #[test]
    fn seed_buffers_grow_and_shrink_in_map_units() {
        let grid = DfmGrid::new(7, 7, 1., geo::coord! { x: 0., y: 7. }).unwrap();
        let mut mask = Dfm::<FloodFill>::new(grid);
        mask.field.fill(0.);
        mask[(3, 3)] = 1.;

        apply_buffer_rules(
            &mut mask,
            &[BufferRule {
                direction: BufferDirection::Grow,
                amount: 1.,
            }],
        );
        assert_eq!(mask.field.iter().filter(|value| **value == 1.).count(), 5);

        apply_buffer_rules(
            &mut mask,
            &[BufferRule {
                direction: BufferDirection::Shrink,
                amount: 1.,
            }],
        );
        assert_eq!(mask.field.iter().filter(|value| **value == 1.).count(), 1);
        assert_eq!(mask[(3, 3)], 1.);
    }

    #[test]
    fn downhill_toggle_uses_d8_receivers_without_unrestricted_flooding() {
        let grid = DfmGrid::standard(geo::coord! { x: 0., y: 256. });
        let mut corrected = Dfm::<HydroCorrected>::new(grid);
        for y in 0..corrected.height() {
            for x in 0..corrected.width() {
                corrected[(y, x)] = 1_000. - x as f32;
            }
        }
        let flow = flow_for(&corrected);
        let mut probability = Dfm::<Water>::new_like(&corrected);
        probability.field.fill(0.);
        probability[(20, 20)] = 1.;

        let level = compute_water_extent(&probability, &corrected, &flow, 0.65, 0., &[], false);
        let downhill = compute_water_extent(&probability, &corrected, &flow, 0.65, 0., &[], true);

        assert_eq!(level[(20, 22)], 0.);
        assert_eq!(downhill[(20, 22)], 1.);
        assert_eq!(downhill[(20, 19)], 0.);
    }
}
