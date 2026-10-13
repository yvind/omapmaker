//! Geometry the *renderer* owns: projecting a paper path into the paint-ready
//! geographic one, triangulating area fills, and cutting a line at its dash
//! points. The space-agnostic path machinery it builds on lives in
//! [`crate::rendering::paper`], which the navigation layer shares.

use eframe::egui::Pos2;
use geo::{LineString, Polygon, TriangulateEarcut};

use super::primitives::Path;
use crate::rendering::geo::GeoTransform;
use crate::rendering::paper::{self, PaperPath};

/// Project a [`PaperPath`] into the paint-ready, geographic-space [`Path`].
pub(super) fn to_path(path: &PaperPath, geo: &GeoTransform) -> Path {
    Path {
        anchors: path.anchors.iter().map(|p| geo.to_mercator(*p)).collect(),
        curves: path
            .curves
            .iter()
            .map(|&(seg, [h1, h2])| (seg, [geo.to_mercator(h1), geo.to_mercator(h2)]))
            .collect(),
        dash_points: path.dash_points.clone(),
    }
}

/// Split `len` vertices into contiguous index runs `(first, last)` — inclusive,
/// and sharing their boundary vertex — cut at each *interior* vertex flagged in
/// `dash_points`. A line's dash pattern and its mid symbols both restart at
/// those OMAP dash points, so both consumers ask for the same runs.
///
/// Index 0 and the last index bound the line already, so a flag there adds no
/// split. Yields a single whole-line run when nothing interior is flagged, or
/// when `dash_points`'s length doesn't match `len` (defensive).
pub fn dash_runs(len: usize, dash_points: &[bool]) -> Vec<(usize, usize)> {
    if len < 2 {
        return Vec::new();
    }
    if dash_points.len() != len {
        return vec![(0, len - 1)];
    }
    let mut bounds = vec![0usize];
    bounds.extend((1..len - 1).filter(|&i| dash_points[i]));
    bounds.push(len - 1);
    bounds.windows(2).map(|w| (w[0], w[1])).collect()
}

/// Triangulate already-flattened polygon rings with [`TriangulateEarcut`].
///
/// Indices address the `exterior ++ interiors` concatenation the painter
/// uploads, *including* each ring's closing duplicate. Ear clipping numbers
/// vertices one shorter per ring, so its output is remapped here — unmapped, an
/// interior ring's triangles land on the preceding ring and paint far more than
/// the object.
///
/// Every ring must already be closed. A ring of fewer than four vertices
/// encloses no area: such an exterior yields nothing, and such an interior is
/// not punched out but keeps its slots, because the painter uploads it anyway.
///
/// Public beyond this crate for [`super::PointMark::Fill`]'s sake: a
/// point-symbol fill is small enough to triangulate fresh every frame, once
/// its curves are flattened at the current zoom, rather than needing a
/// load-time `fill_indices` field the way a whole-map [`super::Kind::Area`]
/// does.
pub fn triangulate_rings(exterior: &[Pos2], interiors: &[Vec<Pos2>]) -> Vec<u32> {
    debug_assert!(
        std::iter::once(exterior)
            .chain(interiors.iter().map(|r| r.as_slice()))
            .all(|r| r.first() == r.last()),
        "triangulate_rings needs closed rings"
    );
    if exterior.len() < 4 {
        return Vec::new();
    }

    // Where each participating ring starts in the painter's vertex list, paired
    // with its length in the ear clipper's numbering.
    let mut rings = Vec::with_capacity(1 + interiors.len());
    rings.push((0, exterior.len() - 1));
    let line_string =
        |ring: &[Pos2]| -> LineString<f64> { ring.iter().copied().map(paper::to_point).collect() };
    let mut holes = Vec::with_capacity(interiors.len());
    let mut base = exterior.len();

    for ring in interiors {
        if ring.len() >= 4 {
            rings.push((base, ring.len() - 1));
            holes.push(line_string(ring));
        }
        base += ring.len();
    }

    // Geo handles flattening the closed rings and locating the holes. Its raw
    // indices omit each closing duplicate, so retain the painter remapping.
    let polygon = Polygon::new(line_string(exterior), holes);
    let clipped = polygon.earcut_triangles_raw().triangle_indices;

    let mut out = Vec::with_capacity(clipped.len());
    for index in clipped {
        // Unreachable for well-formed output; drop the whole mesh rather than
        // emit an index the painter has no vertex for.
        let Some(index) = to_painter_index(index, &rings).and_then(|i| u32::try_from(i).ok())
        else {
            return Vec::new();
        };
        out.push(index);
    }
    out
}

/// Translate one ear-clipper index into the painter's `exterior ++ interiors`
/// numbering. `rings` pairs each participating ring's start in the painter's
/// vertex list with its length in the clipper's, in order.
fn to_painter_index(clipped: usize, rings: &[(usize, usize)]) -> Option<usize> {
    let mut base = 0usize;
    for &(painter_base, len) in rings {
        if clipped < base + len {
            return Some(painter_base + clipped - base);
        }
        base += len;
    }
    None
}

#[cfg(test)]
mod tests {
    use geo::{Area, Triangle};

    use super::*;

    /// Total area of the triangles, read back exactly the way the painter reads
    /// them: index into the `exterior ++ interiors` vertex concatenation.
    fn triangulated_area(exterior: &[Pos2], interiors: &[Vec<Pos2>]) -> f64 {
        let vertices: Vec<Pos2> = exterior
            .iter()
            .chain(interiors.iter().flatten())
            .copied()
            .collect();
        let indices = triangulate_rings(exterior, interiors);
        assert_eq!(indices.len() % 3, 0, "indices must come in triples");
        assert!(
            indices.iter().all(|&i| (i as usize) < vertices.len()),
            "every index must address a painted vertex"
        );
        indices
            .as_chunks::<3>()
            .0
            .iter()
            .map(|t| {
                Triangle::new(
                    paper::to_point(vertices[t[0] as usize]),
                    paper::to_point(vertices[t[1] as usize]),
                    paper::to_point(vertices[t[2] as usize]),
                )
                .unsigned_area()
            })
            .sum()
    }

    fn square(lo: f32, hi: f32) -> Vec<Pos2> {
        vec![
            Pos2::new(lo, lo),
            Pos2::new(hi, lo),
            Pos2::new(hi, hi),
            Pos2::new(lo, hi),
            Pos2::new(lo, lo),
        ]
    }

    /// A ring of three vertices closing on itself — no enclosed area.
    fn sliver(lo: f32, hi: f32) -> Vec<Pos2> {
        vec![Pos2::new(lo, lo), Pos2::new(hi, lo), Pos2::new(lo, lo)]
    }

    #[test]
    fn concave_exterior_fills_only_its_interior() {
        let exterior = vec![
            Pos2::new(0.0, 0.0),
            Pos2::new(4.0, 0.0),
            Pos2::new(4.0, 1.0),
            Pos2::new(1.0, 1.0),
            Pos2::new(1.0, 4.0),
            Pos2::new(0.0, 4.0),
            Pos2::new(0.0, 0.0),
        ];
        let area = triangulated_area(&exterior, &[]);
        assert!(
            (area - 7.0).abs() < 1e-3,
            "the L-shaped fill should cover 7, got {area}"
        );
    }

    #[test]
    fn triangulated_holed_area_covers_the_ring_geometry_exactly() {
        // Regression: the clipper's indices skip each ring's closing duplicate,
        // the painter's vertex list keeps it. Unmapped, the hole's triangles
        // land on exterior vertices and paint a far bigger area than the object.
        let exterior = square(0.0, 10.0);
        let interiors = vec![square(3.0, 7.0)];
        let area = triangulated_area(&exterior, &interiors);
        assert!(
            (area - (100.0 - 16.0)).abs() < 1e-3,
            "10×10 square minus a 4×4 hole should triangulate to 84, got {area}"
        );
    }

    #[test]
    fn several_holes_each_map_back_to_their_own_vertices() {
        let exterior = square(0.0, 10.0);
        let interiors = vec![square(1.0, 3.0), square(6.0, 8.0)];
        let area = triangulated_area(&exterior, &interiors);
        assert!(
            (area - (100.0 - 4.0 - 4.0)).abs() < 1e-3,
            "two 2×2 holes should leave 92, got {area}"
        );
    }

    #[test]
    fn a_hole_enclosing_no_area_is_not_punched_out() {
        let exterior = square(0.0, 10.0);
        let interiors = vec![sliver(3.0, 7.0)];
        let area = triangulated_area(&exterior, &interiors);
        assert!(
            (area - 100.0).abs() < 1e-3,
            "expected a full fill, got {area}"
        );
    }

    #[test]
    fn a_degenerate_hole_does_not_cost_the_others_theirs() {
        // The degenerate ring keeps its slots in the painter's vertex list, so
        // the real hole's indices still land on the real hole.
        let exterior = square(0.0, 10.0);
        let interiors = vec![sliver(1.0, 2.0), square(3.0, 7.0)];
        let area = triangulated_area(&exterior, &interiors);
        assert!(
            (area - (100.0 - 16.0)).abs() < 1e-3,
            "the 4×4 hole should still be punched out, leaving 84, got {area}"
        );
    }

    #[test]
    fn dash_runs_without_interior_flags_is_one_whole_line() {
        assert_eq!(dash_runs(4, &[false; 4]), vec![(0, 3)]);
        // Flags on the endpoints bound the line already.
        assert_eq!(dash_runs(4, &[true, false, false, true]), vec![(0, 3)]);
    }

    #[test]
    fn dash_runs_split_at_interior_flags_sharing_the_boundary_vertex() {
        assert_eq!(
            dash_runs(5, &[false, false, true, false, false]),
            vec![(0, 2), (2, 4)]
        );
        assert_eq!(
            dash_runs(5, &[false, true, false, true, false]),
            vec![(0, 1), (1, 3), (3, 4)]
        );
    }

    #[test]
    fn dash_runs_tolerates_mismatched_or_degenerate_input() {
        assert_eq!(dash_runs(3, &[false]), vec![(0, 2)]);
        assert!(dash_runs(1, &[false]).is_empty());
    }
}
