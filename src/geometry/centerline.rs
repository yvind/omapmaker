//! Medial-axis centerline extraction for projected polygons.

use geo::{
    Coord, Densify, Distance, Euclidean, Intersects, Length, Line, LineString, MultiLineString,
    MultiPolygon, Point, Polygon, PreparedGeometry, Relate, TriangulateDelaunayUnconstrained,
};
use spade::{Triangulation, handles::VoronoiVertex::Inner};

use super::graph::Graph;

const ENDPOINT_CONNECTION_TOLERANCE: f64 = 1.0e-6;

/// Builds a polygon's Voronoi medial axis and returns its significant branches.
/// Distances are expressed in the input geometry's units.
pub(crate) fn extract(
    polygon: &Polygon<f64>,
    densify_spacing: f64,
    minimum_branch_length: f64,
) -> Option<MultiLineString<f64>> {
    if polygon.exterior().0.len() < 4
        || !densify_spacing.is_finite()
        || densify_spacing <= 0.0
        || !minimum_branch_length.is_finite()
        || minimum_branch_length < 0.0
    {
        return None;
    }

    let densified = Euclidean.densify(polygon, densify_spacing);
    let triangulation = densified.unconstrained_triangulation_raw().ok()?;
    let prepared = PreparedGeometry::from(polygon);

    let mut graph = Graph::new();
    for edge in triangulation.undirected_voronoi_edges() {
        // Infinite Voronoi rays end outside the polygon and can never pass the
        // full-edge containment test. Building only finite dual edges also
        // avoids clipping degeneracies for cocircular boundary vertices.
        let [Inner(from), Inner(to)] = edge.vertices() else {
            continue;
        };
        let from = from.circumcenter();
        let to = to.circumcenter();
        let edge = Line::new(
            Coord {
                x: from.x,
                y: from.y,
            },
            Coord { x: to.x, y: to.y },
        );
        if prepared.relate(&edge).is_contains() {
            let thickness =
                boundary_distance(polygon, edge.start) + boundary_distance(polygon, edge.end);
            graph.add_line(&edge, thickness);
        }
    }

    let mut branches = graph
        .significant_branches(minimum_branch_length)
        .into_iter()
        .map(LineString::new)
        .collect::<Vec<_>>();
    // The graph orders the principal path first. Tiny polygons deliberately
    // bypass the configured linearity threshold, but that must not also retain
    // the short end-cap Voronoi spokes as extra overlapping lines. A secondary
    // path shorter than the polygon's local width is not independently
    // line-like; genuine arms remain because they extend for several widths.
    if branches.len() > 1 {
        let mut is_main_path = true;
        branches.retain(|branch| {
            if is_main_path {
                is_main_path = false;
                return true;
            }

            Euclidean.length(branch) >= maximum_local_width(polygon, branch)
        });
    }
    (!branches.is_empty()).then(|| MultiLineString::new(branches))
}

/// Converts a polygon to a complete medial-axis line network when its main
/// path is sufficiently long relative to the polygon's local width. This
/// separates the geometric question "can this polygon be represented by a
/// line?" from any later symbol classification.
pub(crate) fn linearize(
    polygon: &Polygon<f64>,
    densify_spacing: f64,
    minimum_length_width_ratio: f64,
) -> Option<MultiLineString<f64>> {
    if !minimum_length_width_ratio.is_finite() || minimum_length_width_ratio < 0. {
        return None;
    }

    let mut centerlines = extract(polygon, densify_spacing, 0.)?;
    let main = centerlines.0.first()?;
    if Euclidean.length(main) < minimum_length_width_ratio * maximum_local_width(polygon, main) {
        return None;
    }

    extend_terminal_branches(&mut centerlines, polygon, &MultiPolygon::empty());
    Some(centerlines)
}

fn maximum_local_width(polygon: &Polygon<f64>, line: &LineString<f64>) -> f64 {
    line.0
        .iter()
        .map(|coordinate| 2. * boundary_distance(polygon, *coordinate))
        .fold(0., f64::max)
}

/// Finite Voronoi edges stop near a polygon's end caps because the remaining
/// medial-axis rays are infinite and deliberately omitted during graph
/// construction. Extend only unconnected branch terminals to the first parent
/// boundary crossing. Endpoints shared with another branch are junctions and
/// must remain at the junction, while endpoints on `protected` retained
/// geometry already mark a width-class transition and must not cross it.
pub(crate) fn extend_terminal_branches(
    branches: &mut MultiLineString<f64>,
    parent: &Polygon<f64>,
    protected: &MultiPolygon<f64>,
) {
    let original = branches.0.clone();
    for (branch_index, branch) in branches.0.iter_mut().enumerate() {
        if branch.0.len() < 2 || branch.is_closed() {
            continue;
        }

        let first = branch.0[0];
        if !endpoint_is_connected(first, branch_index, &original)
            && !protected.intersects(&Point::from(first))
            && let Some(boundary) = terminal_boundary_intersection(first, branch.0[1], parent)
        {
            branch.0[0] = boundary;
        }

        let last_index = branch.0.len() - 1;
        let last = branch.0[last_index];
        if !endpoint_is_connected(last, branch_index, &original)
            && !protected.intersects(&Point::from(last))
            && let Some(boundary) =
                terminal_boundary_intersection(last, branch.0[last_index - 1], parent)
        {
            branch.0[last_index] = boundary;
        }
    }
}

fn endpoint_is_connected(
    endpoint: Coord<f64>,
    branch_index: usize,
    branches: &[LineString<f64>],
) -> bool {
    branches.iter().enumerate().any(|(other_index, branch)| {
        other_index != branch_index
            && branch.0.iter().any(|coordinate| {
                (coordinate.x - endpoint.x).hypot(coordinate.y - endpoint.y)
                    <= ENDPOINT_CONNECTION_TOLERANCE
            })
    })
}

fn terminal_boundary_intersection(
    endpoint: Coord<f64>,
    adjacent: Coord<f64>,
    polygon: &Polygon<f64>,
) -> Option<Coord<f64>> {
    let direction = Coord {
        x: endpoint.x - adjacent.x,
        y: endpoint.y - adjacent.y,
    };
    if direction.x.hypot(direction.y) <= f64::EPSILON {
        return None;
    }

    std::iter::once(polygon.exterior())
        .chain(polygon.interiors())
        .flat_map(LineString::lines)
        .filter_map(|boundary| ray_segment_intersection(endpoint, direction, boundary))
        .min_by(|(first, _), (second, _)| first.total_cmp(second))
        .map(|(_, coordinate)| coordinate)
}

fn ray_segment_intersection(
    origin: Coord<f64>,
    direction: Coord<f64>,
    segment: Line<f64>,
) -> Option<(f64, Coord<f64>)> {
    let boundary_direction = segment.delta();
    let denominator = cross(direction, boundary_direction);
    if denominator.abs() <= f64::EPSILON {
        return None;
    }

    let to_boundary = segment.start - origin;
    let ray_distance = cross(to_boundary, boundary_direction) / denominator;
    let boundary_fraction = cross(to_boundary, direction) / denominator;
    if ray_distance < -ENDPOINT_CONNECTION_TOLERANCE
        || boundary_fraction < -ENDPOINT_CONNECTION_TOLERANCE
        || boundary_fraction > 1. + ENDPOINT_CONNECTION_TOLERANCE
    {
        return None;
    }

    let ray_distance = ray_distance.max(0.);
    Some((
        ray_distance,
        Coord {
            x: origin.x + ray_distance * direction.x,
            y: origin.y + ray_distance * direction.y,
        },
    ))
}

fn cross(first: Coord<f64>, second: Coord<f64>) -> f64 {
    first.x * second.y - first.y * second.x
}

fn boundary_distance(polygon: &Polygon<f64>, coordinate: Coord<f64>) -> f64 {
    let point = Point::from(coordinate);
    std::iter::once(polygon.exterior())
        .chain(polygon.interiors())
        .map(|ring| Euclidean.distance(&point, ring))
        .fold(f64::INFINITY, f64::min)
}

#[cfg(test)]
mod tests {
    use super::*;
    use geo::{Contains, Point, polygon};

    #[test]
    fn extracts_a_spine_inside_a_long_polygon() {
        let polygon = polygon![
            (x: 0.0, y: 0.0),
            (x: 100.0, y: 0.0),
            (x: 100.0, y: 10.0),
            (x: 0.0, y: 10.0),
        ];

        let centerlines = extract(&polygon, 5.0, 12.0).expect("centerline extraction succeeds");

        assert_eq!(centerlines.0.len(), 1);
        assert!(
            centerlines
                .0
                .iter()
                .flat_map(|line| &line.0)
                .all(|coordinate| polygon.contains(&Point::from(*coordinate)))
        );
        let x_span = centerlines.0[0]
            .0
            .iter()
            .map(|coord| coord.x)
            .fold(0.0, f64::max)
            - centerlines.0[0]
                .0
                .iter()
                .map(|coord| coord.x)
                .fold(f64::INFINITY, f64::min);
        assert!(x_span > 80.0, "centerline x span was {x_span}");
    }

    #[test]
    fn rejects_a_compact_polygon() {
        let polygon = polygon![
            (x: 0.0, y: 0.0),
            (x: 10.0, y: 0.0),
            (x: 10.0, y: 10.0),
            (x: 0.0, y: 10.0),
        ];

        assert!(extract(&polygon, 5.0, 12.0).is_none());
    }

    #[test]
    fn a_hole_in_a_linear_polygon_does_not_make_the_spine_loop_back() {
        let exterior = polygon![
            (x: 0.0, y: 0.0),
            (x: 40.0, y: 0.0),
            (x: 40.0, y: 4.0),
            (x: 0.0, y: 4.0),
        ];
        let hole = polygon![
            (x: 18.0, y: 1.0),
            (x: 22.0, y: 1.0),
            (x: 22.0, y: 3.0),
            (x: 18.0, y: 3.0),
        ];
        let polygon = Polygon::new(exterior.exterior().clone(), vec![hole.exterior().clone()]);

        let centerlines = extract(&polygon, 1.0, 2.0).expect("centerline extraction succeeds");

        assert_eq!(centerlines.0.len(), 1, "centerlines={centerlines:?}");
        assert!(!centerlines.0[0].is_closed());
        let mut unique = centerlines.0[0].0.clone();
        unique.sort_by(|a, b| a.x.total_cmp(&b.x).then(a.y.total_cmp(&b.y)));
        unique.dedup();
        assert_eq!(unique.len(), centerlines.0[0].0.len());
    }

    #[test]
    fn linearization_uses_length_relative_to_width() {
        let compact = polygon![
            (x: 0.0, y: 0.0),
            (x: 5.0, y: 0.0),
            (x: 5.0, y: 5.0),
            (x: 0.0, y: 5.0),
        ];
        let linear = polygon![
            (x: 0.0, y: 0.0),
            (x: 50.0, y: 0.0),
            (x: 50.0, y: 5.0),
            (x: 0.0, y: 5.0),
        ];

        assert!(linearize(&compact, 1.0, 2.0).is_none());
        assert!(linearize(&linear, 1.0, 2.0).is_some());
    }
}
