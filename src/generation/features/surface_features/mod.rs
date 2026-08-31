use las::point::Classification;

use crate::{
    geometry::{PointCloud, PointLaz},
    raster::{
        Dfm, Elevation, FilteredSurface, GroundRelief2m, GroundRelief5m, HardObjectConfidence,
        HardObjectHeight, RasterMarker, VegetationLikelihood,
    },
};

const SMALL_RELIEF_RADIUS_M: f64 = 2.;
const LARGE_RELIEF_RADIUS_M: f64 = 5.;
const RETURN_NEIGHBORHOOD_RADIUS_M: f64 = 1.;
const LAST_RETURN_EXTENSION_RADIUS_M: f64 = 0.75;
const MINIMUM_OBJECT_HEIGHT_M: f32 = 0.15;
const MAXIMUM_OBJECT_HEIGHT_M: f32 = 3.;
const MAXIMUM_ELEVATED_HEIGHT_M: f32 = 40.;
const SURFACE_SUPPORT_TOLERANCE_M: f32 = 0.25;
const REQUIRED_LOCAL_PULSES: f32 = 4.;
const MINIMUM_HARD_OBJECT_CONFIDENCE: f32 = 0.35;

/// Terrain-only and above-ground surface products used to reveal small map
/// features without allowing penetrable vegetation to dominate the surface.
pub struct SurfaceFeatureRasters {
    pub ground_relief_2m: Dfm<GroundRelief2m>,
    pub ground_relief_5m: Dfm<GroundRelief5m>,
    pub hard_object_height: Dfm<HardObjectHeight>,
    pub hard_object_confidence: Dfm<HardObjectConfidence>,
    pub vegetation_likelihood: Dfm<VegetationLikelihood>,
    pub filtered_surface: Dfm<FilteredSurface>,
}

#[derive(Clone, Copy)]
struct CellReturnStats {
    pulse_count: f32,
    only_pulse_count: f32,
    multi_pulse_count: f32,
    return_count: f32,
    vegetation_return_count: f32,
    candidate_return_count: f32,
    elevated_min: f32,
    elevated_max: f32,
    only_candidate_max: f32,
    last_candidate_max: f32,
}

impl Default for CellReturnStats {
    fn default() -> Self {
        Self {
            pulse_count: 0.,
            only_pulse_count: 0.,
            multi_pulse_count: 0.,
            return_count: 0.,
            vegetation_return_count: 0.,
            candidate_return_count: 0.,
            elevated_min: f32::INFINITY,
            elevated_max: f32::NEG_INFINITY,
            only_candidate_max: f32::NEG_INFINITY,
            last_candidate_max: f32::NEG_INFINITY,
        }
    }
}

pub fn compute_surface_features(cloud: &PointCloud, dem: &Dfm<Elevation>) -> SurfaceFeatureRasters {
    let ground_relief_2m = local_relief(dem, SMALL_RELIEF_RADIUS_M);
    let ground_relief_5m = local_relief(dem, LARGE_RELIEF_RADIUS_M);
    let cell_count = dem.width() * dem.height();
    let mut stats = vec![CellReturnStats::default(); cell_count];

    for point in &cloud.points {
        if is_noise(point.0.classification) {
            continue;
        }
        let Some((cell, height)) = point_cell_and_height(point, dem) else {
            continue;
        };
        let number_of_returns = point.0.number_of_returns;
        let return_number = point.0.return_number;
        if number_of_returns == 0 || return_number == 0 || return_number > number_of_returns {
            continue;
        }

        let cell_stats = &mut stats[cell];
        cell_stats.return_count += 1.;
        if is_vegetation(point.0.classification) {
            cell_stats.vegetation_return_count += 1.;
        }
        if return_number == 1 {
            // Count the first record only so a pulse that produced three
            // returns does not contribute three times the observation weight.
            cell_stats.pulse_count += 1.;
            if number_of_returns == 1 {
                cell_stats.only_pulse_count += 1.;
            } else {
                cell_stats.multi_pulse_count += 1.;
            }
        }

        if (MINIMUM_OBJECT_HEIGHT_M..=MAXIMUM_ELEVATED_HEIGHT_M).contains(&height) {
            cell_stats.elevated_min = cell_stats.elevated_min.min(height);
            cell_stats.elevated_max = cell_stats.elevated_max.max(height);
        }
        if !(MINIMUM_OBJECT_HEIGHT_M..=MAXIMUM_OBJECT_HEIGHT_M).contains(&height) {
            continue;
        }

        cell_stats.candidate_return_count += 1.;
        if return_number == 1 && number_of_returns == 1 {
            cell_stats.only_candidate_max = cell_stats.only_candidate_max.max(height);
        }
        if return_number == number_of_returns {
            cell_stats.last_candidate_max = cell_stats.last_candidate_max.max(height);
        }
    }

    // Only returns seed a surface. A last return from a multi-return pulse may
    // fill a sampling hole, but only where an adjacent only-return seed already
    // supports the same elevation. Low branches cannot create candidates alone.
    let extension_radius = cells_for_radius(dem, LAST_RETURN_EXTENSION_RADIUS_M);
    let only_max = stats
        .iter()
        .map(|cell| cell.only_candidate_max)
        .collect::<Vec<_>>();
    let nearby_only_max = local_extreme(
        &only_max,
        dem.width(),
        dem.height(),
        extension_radius,
        f32::NEG_INFINITY,
        f32::max,
    );
    let mut candidate_height = only_max;
    for (index, height) in candidate_height.iter_mut().enumerate() {
        if height.is_finite() {
            continue;
        }
        let last_height = stats[index].last_candidate_max;
        let seed_height = nearby_only_max[index];
        if last_height.is_finite()
            && seed_height.is_finite()
            && (last_height - seed_height).abs() <= SURFACE_SUPPORT_TOLERANCE_M
        {
            *height = last_height;
        }
    }

    // Surface support is evaluated against the candidate in the point's own
    // cell, then accumulated spatially with the remaining return statistics.
    let mut surface_support = vec![0_f32; cell_count];
    for point in &cloud.points {
        if is_noise(point.0.classification) {
            continue;
        }
        let Some((cell, height)) = point_cell_and_height(point, dem) else {
            continue;
        };
        let candidate = candidate_height[cell];
        if !candidate.is_finite()
            || !(MINIMUM_OBJECT_HEIGHT_M..=MAXIMUM_OBJECT_HEIGHT_M).contains(&height)
            || (height - candidate).abs() > SURFACE_SUPPORT_TOLERANCE_M
        {
            continue;
        }
        if point.0.return_number == 1 && point.0.number_of_returns == 1 {
            surface_support[cell] += 1.;
        } else if point.0.number_of_returns > 0
            && point.0.return_number == point.0.number_of_returns
        {
            // A penetrable pulse is useful supporting evidence once a solid
            // surface exists, but is intentionally weaker than an only return.
            surface_support[cell] += 0.35;
        }
    }

    let neighborhood_radius = cells_for_radius(dem, RETURN_NEIGHBORHOOD_RADIUS_M);
    let pulse_count = local_sum(
        &stats
            .iter()
            .map(|cell| cell.pulse_count)
            .collect::<Vec<_>>(),
        dem.width(),
        dem.height(),
        neighborhood_radius,
    );
    let only_pulse_count = local_sum(
        &stats
            .iter()
            .map(|cell| cell.only_pulse_count)
            .collect::<Vec<_>>(),
        dem.width(),
        dem.height(),
        neighborhood_radius,
    );
    let multi_pulse_count = local_sum(
        &stats
            .iter()
            .map(|cell| cell.multi_pulse_count)
            .collect::<Vec<_>>(),
        dem.width(),
        dem.height(),
        neighborhood_radius,
    );
    let return_count = local_sum(
        &stats
            .iter()
            .map(|cell| cell.return_count)
            .collect::<Vec<_>>(),
        dem.width(),
        dem.height(),
        neighborhood_radius,
    );
    let vegetation_return_count = local_sum(
        &stats
            .iter()
            .map(|cell| cell.vegetation_return_count)
            .collect::<Vec<_>>(),
        dem.width(),
        dem.height(),
        neighborhood_radius,
    );
    let candidate_return_count = local_sum(
        &stats
            .iter()
            .map(|cell| cell.candidate_return_count)
            .collect::<Vec<_>>(),
        dem.width(),
        dem.height(),
        neighborhood_radius,
    );
    let local_surface_support = local_sum(
        &surface_support,
        dem.width(),
        dem.height(),
        neighborhood_radius,
    );
    let elevated_min = local_extreme(
        &stats
            .iter()
            .map(|cell| cell.elevated_min)
            .collect::<Vec<_>>(),
        dem.width(),
        dem.height(),
        neighborhood_radius,
        f32::INFINITY,
        f32::min,
    );
    let elevated_max = local_extreme(
        &stats
            .iter()
            .map(|cell| cell.elevated_max)
            .collect::<Vec<_>>(),
        dem.width(),
        dem.height(),
        neighborhood_radius,
        f32::NEG_INFINITY,
        f32::max,
    );

    let mut hard_object_height = Dfm::<HardObjectHeight>::new_like(dem);
    let mut hard_object_confidence = Dfm::<HardObjectConfidence>::new_like(dem);
    let mut vegetation_likelihood = Dfm::<VegetationLikelihood>::new_like(dem);
    let mut filtered_surface = Dfm::<FilteredSurface>::new_like(dem);

    for index in 0..cell_count {
        let pulses = pulse_count[index];
        let only_fraction = safe_ratio(only_pulse_count[index], pulses);
        let multi_fraction = safe_ratio(multi_pulse_count[index], pulses);
        let vegetation_fraction = safe_ratio(vegetation_return_count[index], return_count[index]);
        let vertical_spread = if elevated_min[index].is_finite() && elevated_max[index].is_finite()
        {
            (elevated_max[index] - elevated_min[index]).max(0.)
        } else {
            0.
        };
        let spread_score = smoothstep((vertical_spread - 0.5) / 2.5);
        let vegetation = 1.
            - (1. - 0.85 * multi_fraction)
                * (1. - 0.95 * vegetation_fraction)
                * (1. - 0.75 * spread_score);
        vegetation_likelihood.field[index] = vegetation.clamp(0., 1.);

        let observation_score = (pulses / REQUIRED_LOCAL_PULSES).clamp(0., 1.);
        let opacity_score = smoothstep((only_fraction - 0.25) / 0.65);
        let coherence_score = smoothstep(
            (safe_ratio(local_surface_support[index], candidate_return_count[index]) - 0.2) / 0.65,
        );
        let candidate_confidence = observation_score
            * (0.55 * opacity_score + 0.45 * coherence_score)
            * (1. - vegetation_likelihood.field[index]);
        let confidence = if candidate_height[index].is_finite() {
            candidate_confidence.clamp(0., 1.)
        } else {
            0.
        };
        hard_object_confidence.field[index] = confidence;
        hard_object_height.field[index] = if confidence >= MINIMUM_HARD_OBJECT_CONFIDENCE {
            candidate_height[index]
        } else {
            0.
        };
        filtered_surface.field[index] = dem.field[index] + hard_object_height.field[index];
    }

    SurfaceFeatureRasters {
        ground_relief_2m,
        ground_relief_5m,
        hard_object_height,
        hard_object_confidence,
        vegetation_likelihood,
        filtered_surface,
    }
}

fn point_cell_and_height(point: &PointLaz, dem: &Dfm<Elevation>) -> Option<(usize, f32)> {
    let x = ((point.x() - dem.grid.top_left.x) / dem.grid.cell_size_m).round() as isize;
    let y = ((dem.grid.top_left.y - point.y()) / dem.grid.cell_size_m).round() as isize;
    if x < 0 || y < 0 || x >= dem.width() as isize || y >= dem.height() as isize {
        return None;
    }
    let index = y as usize * dem.width() + x as usize;
    let ground = dem
        .sample_bilinear(geo::coord! { x: point.x(), y: point.y() })
        .unwrap_or(dem.field[index]);
    let height = (point.0.z - f64::from(ground)) as f32;
    height.is_finite().then_some((index, height))
}

fn is_vegetation(classification: Classification) -> bool {
    matches!(
        classification,
        Classification::LowVegetation
            | Classification::MediumVegetation
            | Classification::HighVegetation
    )
}

fn is_noise(classification: Classification) -> bool {
    matches!(
        classification,
        Classification::LowPoint | Classification::HighNoise
    )
}

fn cells_for_radius<T: RasterMarker>(raster: &Dfm<T>, radius_m: f64) -> usize {
    (radius_m / raster.grid.cell_size_m).ceil() as usize
}

fn safe_ratio(numerator: f32, denominator: f32) -> f32 {
    if denominator > f32::EPSILON {
        (numerator / denominator).clamp(0., 1.)
    } else {
        0.
    }
}

fn smoothstep(value: f32) -> f32 {
    let value = value.clamp(0., 1.);
    value * value * (3. - 2. * value)
}

fn local_sum(values: &[f32], width: usize, height: usize, radius: usize) -> Vec<f32> {
    let stride = width + 1;
    let mut prefix = vec![0_f64; stride * (height + 1)];
    for y in 0..height {
        let mut row_sum = 0_f64;
        for x in 0..width {
            row_sum += f64::from(values[y * width + x]);
            prefix[(y + 1) * stride + x + 1] = prefix[y * stride + x + 1] + row_sum;
        }
    }

    let mut sums = vec![0_f32; width * height];
    for y in 0..height {
        let top = y.saturating_sub(radius);
        let bottom = (y + radius + 1).min(height);
        for x in 0..width {
            let left = x.saturating_sub(radius);
            let right = (x + radius + 1).min(width);
            sums[y * width + x] = (prefix[bottom * stride + right]
                - prefix[top * stride + right]
                - prefix[bottom * stride + left]
                + prefix[top * stride + left]) as f32;
        }
    }
    sums
}

fn local_extreme(
    values: &[f32],
    width: usize,
    height: usize,
    radius: usize,
    initial: f32,
    combine: fn(f32, f32) -> f32,
) -> Vec<f32> {
    let mut result = vec![initial; width * height];
    for y in 0..height {
        let top = y.saturating_sub(radius);
        let bottom = (y + radius).min(height - 1);
        for x in 0..width {
            let left = x.saturating_sub(radius);
            let right = (x + radius).min(width - 1);
            let mut extreme = initial;
            for row in top..=bottom {
                for column in left..=right {
                    extreme = combine(extreme, values[row * width + column]);
                }
            }
            result[y * width + x] = extreme;
        }
    }
    result
}

/// A centred local mean is also the fitted-plane elevation at the centre of a
/// symmetric window. Shrinking the window symmetrically at tile edges keeps a
/// planar slope at zero residual without requiring a per-cell matrix solve.
fn local_relief<T: RasterMarker>(dem: &Dfm<Elevation>, radius_m: f64) -> Dfm<T> {
    let requested_radius = cells_for_radius(dem, radius_m);
    let prefix = local_sum_prefix(&dem.field, dem.width(), dem.height());
    let stride = dem.width() + 1;
    let mut relief = Dfm::<T>::new_like(dem);
    for y in 0..dem.height() {
        let radius_y = requested_radius.min(y).min(dem.height() - 1 - y);
        let top = y - radius_y;
        let bottom = y + radius_y + 1;
        for x in 0..dem.width() {
            let radius_x = requested_radius.min(x).min(dem.width() - 1 - x);
            let left = x - radius_x;
            let right = x + radius_x + 1;
            let sum = prefix[bottom * stride + right]
                - prefix[top * stride + right]
                - prefix[bottom * stride + left]
                + prefix[top * stride + left];
            let count = (bottom - top) * (right - left);
            relief[(y, x)] = dem[(y, x)] - (sum / count as f64) as f32;
        }
    }
    relief
}

fn local_sum_prefix(values: &[f32], width: usize, height: usize) -> Vec<f64> {
    let stride = width + 1;
    let mut prefix = vec![0_f64; stride * (height + 1)];
    for y in 0..height {
        let mut row_sum = 0_f64;
        for x in 0..width {
            row_sum += f64::from(values[y * width + x]);
            prefix[(y + 1) * stride + x + 1] = prefix[y * stride + x + 1] + row_sum;
        }
    }
    prefix
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::{geometry::PointLaz, raster::DfmGrid};
    use las::{Bounds, Vector};

    fn test_dem() -> Dfm<Elevation> {
        let grid = DfmGrid::new(21, 21, 1., geo::coord! { x: 0., y: 20. }).unwrap();
        let mut dem = Dfm::<Elevation>::new(grid);
        dem.field.fill(0.);
        dem
    }

    fn bounds() -> Bounds {
        Bounds {
            min: Vector {
                x: 0.,
                y: 0.,
                z: 0.,
            },
            max: Vector {
                x: 20.,
                y: 20.,
                z: 40.,
            },
        }
    }

    #[test]
    fn local_relief_removes_a_plane_and_retains_a_bump() {
        let mut dem = test_dem();
        for y in 0..dem.height() {
            for x in 0..dem.width() {
                dem[(y, x)] = 0.2 * x as f32 - 0.1 * y as f32;
            }
        }
        let centre = 10 * dem.width() + 10;
        dem.field[centre] += 1.;

        let relief = local_relief::<GroundRelief2m>(&dem, 2.);
        assert!(relief.field[centre] > 0.8);
        assert!(relief[(6, 6)].abs() < 1e-5);
    }

    #[test]
    fn only_return_surface_survives_but_classified_tree_is_rejected() {
        let dem = test_dem();
        let mut points = Vec::new();
        for &(x, y) in &[(5., 15.), (6., 15.), (5., 14.), (6., 14.)] {
            points.push(PointLaz::new(x, y, 1.));
        }
        for &(x, y, z) in &[
            (14., 15., 0.5),
            (15., 15., 1.2),
            (14., 14., 2.1),
            (15., 14., 2.8),
        ] {
            let mut point = PointLaz::new(x, y, z);
            point.0.classification = Classification::HighVegetation;
            points.push(point);
        }

        let features = compute_surface_features(&PointCloud::new(points, bounds()), &dem);
        let rock = 5 * dem.width() + 5;
        let tree = 5 * dem.width() + 14;
        assert!(features.hard_object_height.field[rock] > 0.9);
        assert!(features.hard_object_confidence.field[rock] > 0.5);
        assert_eq!(features.hard_object_height.field[tree], 0.);
        assert!(features.vegetation_likelihood.field[tree] > 0.8);
    }

    #[test]
    fn multi_return_last_echo_cannot_seed_an_object() {
        let dem = test_dem();
        let mut points = Vec::new();
        for return_number in 1..=3 {
            let mut point = PointLaz::new(10., 10., 3.5 - return_number as f64 * 0.5);
            point.0.return_number = return_number;
            point.0.number_of_returns = 3;
            points.push(point);
        }

        let features = compute_surface_features(&PointCloud::new(points, bounds()), &dem);
        let cell = 10 * dem.width() + 10;
        assert_eq!(features.hard_object_height.field[cell], 0.);
        assert_eq!(features.hard_object_confidence.field[cell], 0.);
        assert!(features.vegetation_likelihood.field[cell] > 0.8);
    }
}
