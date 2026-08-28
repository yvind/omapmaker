//! Medial-axis centerline extraction for projected polygons.

use geo::{
    Coord, Densify, Distance, Euclidean, Line, LineString, MultiLineString, Point, Polygon,
    PreparedGeometry, Relate, TriangulateDelaunayUnconstrained,
};
use spade::{Triangulation, handles::VoronoiVertex::Inner};

use super::graph::Graph;

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

    let branches = graph
        .significant_branches(minimum_branch_length)
        .into_iter()
        .map(LineString::new)
        .collect::<Vec<_>>();
    (!branches.is_empty()).then(|| MultiLineString::new(branches))
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
}
