use geo::{Area, BooleanOps, Simplify, Validation};

use crate::parameters::BuildingParameters;

const ANGLE_EPSILON: f64 = 1e-8;
const COORDINATE_EPSILON: f64 = 1e-7;

#[derive(Clone, Copy)]
struct SourceEdge {
    start: geo::Coord,
    end: geo::Coord,
    angle: f64,
    length: f64,
}

#[derive(Clone, Copy)]
struct SupportingLine {
    start: geo::Coord,
    end: geo::Coord,
    normal: (f64, f64),
    offset: f64,
    weight: f64,
    orientation: Option<u8>,
}

/// Align a building footprint to a length-weighted dominant direction.
///
/// Every accepted result is valid, retains the configured minimum IoU, and
/// stays within the configured boundary-displacement tolerance. Any failed
/// safeguard returns the source geometry unchanged.
pub fn regularize_building_footprint(
    polygon: &geo::Polygon,
    parameters: &BuildingParameters,
) -> geo::Polygon {
    if !parameters.regularize_footprints || !valid_parameters(parameters) {
        return polygon.clone();
    }

    try_regularize_polygon(polygon, parameters).unwrap_or_else(|| polygon.clone())
}

fn valid_parameters(parameters: &BuildingParameters) -> bool {
    parameters
        .regularization_simplification_tolerance_m
        .is_finite()
        && parameters.regularization_simplification_tolerance_m >= 0.
        && parameters.regularization_parallel_threshold_m.is_finite()
        && parameters.regularization_parallel_threshold_m >= 0.
        && parameters
            .regularization_maximum_boundary_displacement_m
            .is_finite()
        && parameters.regularization_maximum_boundary_displacement_m >= 0.
        && parameters
            .regularization_maximum_angle_deviation_degrees
            .is_finite()
        && (0.0..=45.0).contains(&parameters.regularization_maximum_angle_deviation_degrees)
        && (0.0..=1.0).contains(&parameters.regularization_minimum_supported_edge_fraction)
        && (0.0..=1.0).contains(&parameters.regularization_minimum_iou)
        && parameters.regularization_diagonal_bias_degrees.is_finite()
        && (0.0..=22.5).contains(&parameters.regularization_diagonal_bias_degrees)
}

fn try_regularize_polygon(
    polygon: &geo::Polygon,
    parameters: &BuildingParameters,
) -> Option<geo::Polygon> {
    if !polygon.is_valid() || polygon.unsigned_area() <= f64::EPSILON {
        return None;
    }

    let simplified_exterior = simplify_ring(
        polygon.exterior(),
        parameters.regularization_simplification_tolerance_m,
    )?;
    let exterior_edges = source_edges(&simplified_exterior)?;
    let main_direction = dominant_direction(&exterior_edges)?;
    let exterior = regularize_ring(
        polygon.exterior(),
        &simplified_exterior,
        main_direction,
        parameters,
    )?;

    let mut interiors = Vec::with_capacity(polygon.interiors().len());
    for interior in polygon.interiors() {
        let regularized = simplify_ring(
            interior,
            parameters.regularization_simplification_tolerance_m,
        )
        .and_then(|simplified| regularize_ring(interior, &simplified, main_direction, parameters))
        .unwrap_or_else(|| interior.clone());
        interiors.push(regularized);
    }

    let candidate = geo::Polygon::new(exterior, interiors);
    if !candidate.is_valid() || candidate.unsigned_area() <= f64::EPSILON {
        return None;
    }
    if polygon_boundary_displacement(polygon, &candidate)
        > parameters.regularization_maximum_boundary_displacement_m
    {
        return None;
    }

    let intersection_area = polygon.intersection(&candidate).unsigned_area();
    let union_area = polygon.union(&candidate).unsigned_area();
    if union_area <= f64::EPSILON
        || intersection_area / union_area < parameters.regularization_minimum_iou
    {
        return None;
    }
    Some(candidate)
}

fn simplify_ring(ring: &geo::LineString, tolerance: f64) -> Option<geo::LineString> {
    let simplified = ring.simplify(tolerance);
    (unique_ring_coordinates(&simplified)?.len() >= 3).then_some(simplified)
}

fn source_edges(ring: &geo::LineString) -> Option<Vec<SourceEdge>> {
    let points = unique_ring_coordinates(ring)?;
    let mut edges = Vec::with_capacity(points.len());
    for index in 0..points.len() {
        let start = points[index];
        let end = points[(index + 1) % points.len()];
        let dx = end.x - start.x;
        let dy = end.y - start.y;
        let length = dx.hypot(dy);
        if length <= COORDINATE_EPSILON {
            continue;
        }
        edges.push(SourceEdge {
            start,
            end,
            angle: dy.atan2(dx).rem_euclid(std::f64::consts::PI),
            length,
        });
    }
    (edges.len() >= 3).then_some(edges)
}

fn unique_ring_coordinates(ring: &geo::LineString) -> Option<Vec<geo::Coord>> {
    let mut output = Vec::with_capacity(ring.0.len());
    for &coordinate in &ring.0 {
        if !coordinate.x.is_finite() || !coordinate.y.is_finite() {
            return None;
        }
        if output.last().is_none_or(|previous| {
            squared_distance(*previous, coordinate) > COORDINATE_EPSILON.powi(2)
        }) {
            output.push(coordinate);
        }
    }
    if output.len() >= 2
        && squared_distance(output[0], *output.last().expect("ring is not empty"))
            <= COORDINATE_EPSILON.powi(2)
    {
        output.pop();
    }
    (output.len() >= 3).then_some(output)
}

/// Select the orthogonal edge family with the greatest length-weighted
/// agreement. Evaluating observed edge directions avoids histogram-bin edge
/// effects while retaining the reference algorithm's modulo-90° symmetry.
fn dominant_direction(edges: &[SourceEdge]) -> Option<f64> {
    let total_length: f64 = edges.iter().map(|edge| edge.length).sum();
    if total_length <= f64::EPSILON {
        return None;
    }
    let period = std::f64::consts::FRAC_PI_2;
    let mut best_direction = 0.;
    let mut best_score = f64::NEG_INFINITY;
    for candidate_edge in edges {
        let candidate = candidate_edge.angle.rem_euclid(period);
        let score: f64 = edges
            .iter()
            .map(|edge| {
                let difference = signed_family_difference(edge.angle, candidate, period).abs();
                edge.length * (2. * difference).cos().max(0.).powi(4)
            })
            .sum();
        if score > best_score + 1e-10
            || ((score - best_score).abs() <= 1e-10 && candidate < best_direction)
        {
            best_score = score;
            best_direction = candidate;
        }
    }

    // Refine the observed candidate by a weighted circular-local mean.
    let mut adjustment_sum = 0.;
    let mut adjustment_weight = 0.;
    for edge in edges {
        let difference = signed_family_difference(edge.angle, best_direction, period);
        let weight = edge.length * (2. * difference).cos().max(0.).powi(4);
        adjustment_sum += weight * difference;
        adjustment_weight += weight;
    }
    if adjustment_weight > f64::EPSILON {
        best_direction = (best_direction + adjustment_sum / adjustment_weight).rem_euclid(period);
    }
    Some(best_direction)
}

fn regularize_ring(
    original: &geo::LineString,
    simplified: &geo::LineString,
    main_direction: f64,
    parameters: &BuildingParameters,
) -> Option<geo::LineString> {
    let edges = source_edges(simplified)?;
    let maximum_angle = parameters
        .regularization_maximum_angle_deviation_degrees
        .to_radians();
    let mut supported_length = 0.;
    let total_length: f64 = edges.iter().map(|edge| edge.length).sum();
    let mut lines = Vec::with_capacity(edges.len());

    for edge in edges {
        let (target_angle, orientation, difference) = nearest_target_angle(
            edge.angle,
            main_direction,
            parameters.regularization_allow_45_degree_edges,
            parameters.regularization_diagonal_bias_degrees.to_radians(),
        );
        let supported = difference <= maximum_angle;
        if supported {
            supported_length += edge.length;
        }
        let line_angle = if supported { target_angle } else { edge.angle };
        let normal = (-line_angle.sin(), line_angle.cos());
        let midpoint = geo::coord! {
            x: (edge.start.x + edge.end.x) / 2.,
            y: (edge.start.y + edge.end.y) / 2.,
        };
        lines.push(SupportingLine {
            start: edge.start,
            end: edge.end,
            normal,
            offset: normal.0 * midpoint.x + normal.1 * midpoint.y,
            weight: edge.length,
            orientation: supported.then_some(orientation),
        });
    }

    if total_length <= f64::EPSILON
        || supported_length / total_length
            < parameters.regularization_minimum_supported_edge_fraction
    {
        return None;
    }

    coalesce_parallel_lines(&mut lines, parameters.regularization_parallel_threshold_m);
    if lines.len() < 3 {
        return None;
    }

    let mut coordinates = Vec::with_capacity(lines.len() * 2 + 1);
    for index in 0..lines.len() {
        let current = lines[index];
        let next = lines[(index + 1) % lines.len()];
        if let Some(intersection) = line_intersection(current, next) {
            push_unique(&mut coordinates, intersection);
        } else {
            let joint = geo::coord! {
                x: (current.end.x + next.start.x) / 2.,
                y: (current.end.y + next.start.y) / 2.,
            };
            push_unique(&mut coordinates, project_to_line(joint, current));
            push_unique(&mut coordinates, project_to_line(joint, next));
        }
    }
    if coordinates.len() < 3 {
        return None;
    }
    coordinates.push(coordinates[0]);
    let candidate = geo::LineString::new(coordinates);
    (symmetric_ring_displacement(original, &candidate)
        <= parameters.regularization_maximum_boundary_displacement_m)
        .then_some(candidate)
}

fn nearest_target_angle(
    angle: f64,
    main_direction: f64,
    allow_45_degree: bool,
    diagonal_bias: f64,
) -> (f64, u8, f64) {
    let offsets: &[f64] = if allow_45_degree {
        &[
            0.,
            std::f64::consts::FRAC_PI_4,
            std::f64::consts::FRAC_PI_2,
            3. * std::f64::consts::FRAC_PI_4,
        ]
    } else {
        &[0., std::f64::consts::FRAC_PI_2]
    };
    let mut best = (main_direction, 0_u8, f64::INFINITY, f64::INFINITY);
    for (index, offset) in offsets.iter().copied().enumerate() {
        let target = (main_direction + offset).rem_euclid(std::f64::consts::PI);
        let raw_difference = angle_difference_mod_pi(angle, target);
        let selection_difference = raw_difference
            + if allow_45_degree && index % 2 == 1 {
                diagonal_bias
            } else {
                0.
            };
        if selection_difference < best.3 {
            best = (target, index as u8, raw_difference, selection_difference);
        }
    }
    (best.0, best.1, best.2)
}

fn coalesce_parallel_lines(lines: &mut Vec<SupportingLine>, threshold: f64) {
    let mut changed = true;
    while changed && lines.len() > 3 {
        changed = false;
        for index in 0..lines.len() {
            let next = (index + 1) % lines.len();
            if lines[index].orientation.is_none()
                || lines[index].orientation != lines[next].orientation
                || (lines[index].offset - lines[next].offset).abs() > threshold
            {
                continue;
            }
            let total_weight = lines[index].weight + lines[next].weight;
            let merged_offset = (lines[index].offset * lines[index].weight
                + lines[next].offset * lines[next].weight)
                / total_weight.max(f64::EPSILON);
            if next == 0 {
                lines[0].start = lines[index].start;
                lines[0].offset = merged_offset;
                lines[0].weight = total_weight;
                lines.pop();
            } else {
                lines[index].end = lines[next].end;
                lines[index].offset = merged_offset;
                lines[index].weight = total_weight;
                lines.remove(next);
            }
            changed = true;
            break;
        }
    }
}

fn line_intersection(first: SupportingLine, second: SupportingLine) -> Option<geo::Coord> {
    let determinant = first.normal.0 * second.normal.1 - first.normal.1 * second.normal.0;
    if determinant.abs() <= ANGLE_EPSILON {
        return None;
    }
    Some(geo::coord! {
        x: (first.offset * second.normal.1 - first.normal.1 * second.offset) / determinant,
        y: (first.normal.0 * second.offset - first.offset * second.normal.0) / determinant,
    })
}

fn project_to_line(point: geo::Coord, line: SupportingLine) -> geo::Coord {
    let distance = line.normal.0 * point.x + line.normal.1 * point.y - line.offset;
    geo::coord! {
        x: point.x - distance * line.normal.0,
        y: point.y - distance * line.normal.1,
    }
}

fn push_unique(coordinates: &mut Vec<geo::Coord>, coordinate: geo::Coord) {
    if coordinates
        .last()
        .is_none_or(|previous| squared_distance(*previous, coordinate) > COORDINATE_EPSILON.powi(2))
    {
        coordinates.push(coordinate);
    }
}

fn signed_family_difference(angle: f64, family: f64, period: f64) -> f64 {
    (angle - family + period / 2.).rem_euclid(period) - period / 2.
}

fn angle_difference_mod_pi(first: f64, second: f64) -> f64 {
    let difference = (first - second).abs().rem_euclid(std::f64::consts::PI);
    difference.min(std::f64::consts::PI - difference)
}

fn polygon_boundary_displacement(first: &geo::Polygon, second: &geo::Polygon) -> f64 {
    let mut maximum = symmetric_ring_displacement(first.exterior(), second.exterior());
    if first.interiors().len() != second.interiors().len() {
        return f64::INFINITY;
    }
    for (first_ring, second_ring) in first.interiors().iter().zip(second.interiors()) {
        maximum = maximum.max(symmetric_ring_displacement(first_ring, second_ring));
    }
    maximum
}

fn symmetric_ring_displacement(first: &geo::LineString, second: &geo::LineString) -> f64 {
    directed_ring_displacement(first, second).max(directed_ring_displacement(second, first))
}

fn directed_ring_displacement(source: &geo::LineString, target: &geo::LineString) -> f64 {
    source
        .0
        .iter()
        .map(|&point| point_to_ring_distance(point, target))
        .fold(0., f64::max)
}

fn point_to_ring_distance(point: geo::Coord, ring: &geo::LineString) -> f64 {
    ring.0
        .windows(2)
        .map(|segment| point_to_segment_distance(point, segment[0], segment[1]))
        .fold(f64::INFINITY, f64::min)
}

fn point_to_segment_distance(point: geo::Coord, start: geo::Coord, end: geo::Coord) -> f64 {
    let dx = end.x - start.x;
    let dy = end.y - start.y;
    let length_squared = dx * dx + dy * dy;
    if length_squared <= f64::EPSILON {
        return squared_distance(point, start).sqrt();
    }
    let fraction =
        (((point.x - start.x) * dx + (point.y - start.y) * dy) / length_squared).clamp(0., 1.);
    let projected = geo::coord! {
        x: start.x + fraction * dx,
        y: start.y + fraction * dy,
    };
    squared_distance(point, projected).sqrt()
}

fn squared_distance(first: geo::Coord, second: geo::Coord) -> f64 {
    (first.x - second.x).powi(2) + (first.y - second.y).powi(2)
}

#[cfg(test)]
mod tests {
    use super::*;
    use geo::polygon;

    fn noisy_rotated_rectangle() -> geo::Polygon {
        polygon![
            (x: -0.10, y: 0.15),
            (x: 9.55, y: 2.70),
            (x: 8.05, y: 8.35),
            (x: -1.55, y: 5.75),
        ]
    }

    #[test]
    fn noisy_rotated_rectangle_becomes_orthogonal() {
        let source = noisy_rotated_rectangle();
        let parameters = BuildingParameters {
            regularization_simplification_tolerance_m: 0.,
            regularization_maximum_boundary_displacement_m: 1.,
            regularization_minimum_iou: 0.9,
            ..Default::default()
        };
        let regularized = regularize_building_footprint(&source, &parameters);
        let coordinates = unique_ring_coordinates(regularized.exterior()).unwrap();
        assert_eq!(coordinates.len(), 4);
        for index in 0..coordinates.len() {
            let a = coordinates[index];
            let b = coordinates[(index + 1) % coordinates.len()];
            let c = coordinates[(index + 2) % coordinates.len()];
            let first = (b.x - a.x, b.y - a.y);
            let second = (c.x - b.x, c.y - b.y);
            let dot = first.0 * second.0 + first.1 * second.1;
            assert!(dot.abs() <= 1e-8);
        }
        assert!(polygon_boundary_displacement(&source, &regularized) <= 1.);
        let iou = source.intersection(&regularized).unsigned_area()
            / source.union(&regularized).unsigned_area();
        assert!(iou >= 0.9);
    }

    #[test]
    fn hard_displacement_limit_falls_back_to_source() {
        let source = noisy_rotated_rectangle();
        let parameters = BuildingParameters {
            regularization_simplification_tolerance_m: 0.,
            regularization_maximum_boundary_displacement_m: 0.001,
            ..Default::default()
        };
        assert_eq!(regularize_building_footprint(&source, &parameters), source);
    }

    #[test]
    fn non_structural_round_polygon_is_not_forced_into_a_box() {
        let coordinates = (0..16)
            .map(|index| {
                let angle = index as f64 * std::f64::consts::TAU / 16.;
                geo::coord! { x: angle.cos() * 5., y: angle.sin() * 5. }
            })
            .chain(std::iter::once(geo::coord! { x: 5., y: 0. }))
            .collect();
        let source = geo::Polygon::new(geo::LineString::new(coordinates), vec![]);
        let parameters = BuildingParameters {
            regularization_simplification_tolerance_m: 0.,
            ..Default::default()
        };
        assert_eq!(regularize_building_footprint(&source, &parameters), source);
    }
}
