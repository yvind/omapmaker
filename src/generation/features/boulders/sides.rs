use std::collections::HashMap;

use crate::{
    geometry::PointCloud,
    parameters::BoulderParameters,
    raster::{Dfm, Elevation},
};

use super::BoulderEvidence;
use super::evidence::{is_last, object_class};

pub(super) struct GroundedSides {
    pub(super) grounded_fraction: f64,
    pub(super) minimum_slope_degrees: f64,
}

/// Follow radial profiles from the block to observed surrounding ground.
/// No shape is fitted to the object; the top is checked separately.
pub(super) fn grounded_runoffs(
    heights: &HashMap<usize, f64>,
    evidence: &BoulderEvidence,
    cloud: &PointCloud,
    dem: &Dfm<Elevation>,
    center: geo::Coord,
    params: &BoulderParameters,
) -> Option<GroundedSides> {
    const DIRECTIONS: usize = 16;
    let cell_size = dem.grid.cell_size_m;
    let radius = heights
        .keys()
        .map(|cell| {
            let p = dem.index2coord(cell / dem.width(), cell % dem.width());
            (p.x - center.x).hypot(p.y - center.y)
        })
        .fold(0_f64, f64::max);
    let reach = radius + params.maximum_side_runoff_m + cell_size;
    let required_slope = f64::from(params.minimum_side_slope_degrees)
        .to_radians()
        .tan();
    let minimum_drop = f64::from(params.minimum_height_m.max(1.)) * 0.35;
    let mut slopes = Vec::new();
    // Ground samples are not aligned to raster centers or the 16 profiles.
    // Search a narrow spatial neighborhood instead of requiring a return in
    // the exact cell crossed by a profile.
    let ground_search_radius = cell_size.max(1.);
    let ground_search_cells = (ground_search_radius / cell_size).ceil() as usize;
    // Samples confined to one side must not appear to surround the object
    // just because the profile search has a horizontal tolerance.
    let mut ground_angles = Vec::new();
    let center_x = ((center.x - dem.grid.top_left.x) / cell_size).round() as usize;
    let center_y = ((dem.grid.top_left.y - center.y) / cell_size).round() as usize;
    let context_cells = ((reach + ground_search_radius) / cell_size).ceil() as usize;
    for y in
        center_y.saturating_sub(context_cells)..=(center_y + context_cells).min(dem.height() - 1)
    {
        for x in
            center_x.saturating_sub(context_cells)..=(center_x + context_cells).min(dem.width() - 1)
        {
            let cell = y * dem.width() + x;
            if heights.contains_key(&cell) {
                continue;
            }
            for &index in evidence.points_in_cell(cell) {
                let point = &cloud.points[index];
                let (dx, dy) = (point.x() - center.x, point.y() - center.y);
                if is_last(point)
                    && (point.0.classification == las::point::Classification::Ground
                        || object_class(point.0.classification))
                    && f64::from(evidence.heights_above_ground[index]).abs()
                        <= f64::from(params.ground_level_tolerance_m)
                    && dx.hypot(dy) <= reach + ground_search_radius
                {
                    ground_angles.push(dy.atan2(dx));
                }
            }
        }
    }
    ground_angles.sort_unstable_by(f64::total_cmp);
    ground_angles.dedup();
    if ground_angles.len() < 3 {
        return None;
    }
    let wrap_gap =
        ground_angles[0] + std::f64::consts::TAU - ground_angles[ground_angles.len() - 1];
    let maximum_gap = ground_angles
        .windows(2)
        .map(|a| a[1] - a[0])
        .fold(wrap_gap, f64::max);
    if maximum_gap > std::f64::consts::PI {
        return None;
    }
    for direction in 0..DIRECTIONS {
        let angle = direction as f64 * std::f64::consts::TAU / DIRECTIONS as f64;
        let (dy, dx) = angle.sin_cos();
        let mut previous_cell = None;
        let mut surface = Vec::new();
        let steps = (reach / (cell_size * 0.5)).ceil() as usize;
        for step in 0..=steps {
            let distance = step as f64 * cell_size * 0.5;
            let x = ((center.x + dx * distance - dem.grid.top_left.x) / cell_size).round();
            let y = ((dem.grid.top_left.y - center.y - dy * distance) / cell_size).round();
            if x < 0. || y < 0. || x >= dem.width() as f64 || y >= dem.height() as f64 {
                break;
            }
            let cell = y as usize * dem.width() + x as usize;
            if previous_cell == Some(cell) {
                continue;
            }
            previous_cell = Some(cell);
            let coordinate = dem.index2coord(y as usize, x as usize);
            if let Some(&height) = heights.get(&cell) {
                if height.is_finite() {
                    surface.push((coordinate, height));
                }
                continue;
            }
            // An interpolated DEM alone cannot prove that the side reaches
            // ordinary ground. Require a valid observed last return there.
            let mut ground = None;
            let mut closest = f64::INFINITY;
            for row in (y as usize).saturating_sub(ground_search_cells)
                ..=(y as usize + ground_search_cells).min(dem.height() - 1)
            {
                for column in (x as usize).saturating_sub(ground_search_cells)
                    ..=(x as usize + ground_search_cells).min(dem.width() - 1)
                {
                    let neighbor = row * dem.width() + column;
                    for &index in evidence.points_in_cell(neighbor) {
                        let point = &cloud.points[index];
                        if !is_last(point)
                            || !(point.0.classification == las::point::Classification::Ground
                                || object_class(point.0.classification))
                        {
                            continue;
                        }
                        let ground_height = f64::from(evidence.heights_above_ground[index]);
                        let distance = (point.x() - coordinate.x).hypot(point.y() - coordinate.y);
                        let point_angle = (point.y() - center.y).atan2(point.x() - center.x);
                        let angle_difference = (point_angle - angle + std::f64::consts::PI)
                            .rem_euclid(std::f64::consts::TAU)
                            - std::f64::consts::PI;
                        if ground_height.abs() <= f64::from(params.ground_level_tolerance_m)
                            && distance <= ground_search_radius
                            && angle_difference.abs() <= std::f64::consts::TAU / DIRECTIONS as f64
                            && distance < closest
                        {
                            closest = distance;
                            ground = Some((point, ground_height));
                        }
                    }
                }
            }
            let Some((point, ground_height)) = ground else {
                continue;
            };
            let slope = surface
                .iter()
                .filter_map(|(p, height)| {
                    let run = (p.x - point.x()).hypot(p.y - point.y());
                    let drop = height - ground_height;
                    (run > 0. && run <= params.maximum_side_runoff_m && drop >= minimum_drop)
                        .then_some(drop / run)
                })
                .fold(0_f64, f64::max);
            if slope >= required_slope {
                slopes.push(slope.atan().to_degrees());
            }
            // The first observed ground fixes the end of this side profile;
            // a distant low point cannot rescue a gently sloping shoulder.
            break;
        }
    }
    let grounded_fraction = slopes.len() as f64 / DIRECTIONS as f64;
    if slopes.is_empty() || grounded_fraction < f64::from(params.minimum_grounded_side_fraction) {
        return None;
    }
    Some(GroundedSides {
        grounded_fraction,
        minimum_slope_degrees: slopes.into_iter().fold(f64::INFINITY, f64::min),
    })
}
