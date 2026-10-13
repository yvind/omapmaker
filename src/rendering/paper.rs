//! Paths in paper space - the map's own coordinate system (millimetres, y-up =
//! magnetic north), which is what every `omap` geometry accessor returns.
//!
//! The one place that understands the file's own Bézier control points, free of
//! rendering and georeferencing so the renderer and the navigation layer can
//! share it. Anything needing a *target* space lives with its consumer.

use eframe::epaint::{Color32, CubicBezierShape, Pos2, Stroke};
use geo::{BoundingRect, Euclidean, Length, LineString};
#[cfg(test)]
use geo::{Contains, Distance, Line, Polygon};
use omap::geo_types::Coord;
use omap::objects::{AreaObject, BezierPath, BezierSegment, LineObject};

/// Widen an egui point to geometry coordinates, preserving its `f32` precision.
pub fn to_point(p: Pos2) -> geo::Coord {
    Coord::from((p.x as f64, p.y as f64))
}

/// Narrow geometry coordinates for egui, keeping the axes unchanged.
pub fn to_pos2(p: geo::Coord) -> Pos2 {
    Pos2::new(p.x as f32, p.y as f32)
}

/// An area object's rings as paper-space paths, exterior first.
///
/// The file's own Bézier control points, so curvature survives to whatever the
/// caller does next — the renderer strokes outlines from them in screen space,
/// the navigation layer flattens them at [`BEZIER_FLATTEN_ERROR_MM`].
///
/// Every ring `omap` hands back is closed, so these are too — which is what a
/// fill mesh assumes and what a scanline fill is happier with.
pub fn area_paths(area_obj: &AreaObject) -> Vec<PaperPath> {
    let geometry = area_obj.geometry();
    std::iter::once(geometry.exterior())
        .chain(geometry.interiors())
        .map(PaperPath::from_bezier)
        .collect()
}

/// A line object's geometry as a paper-space path, keeping its curves for the
/// same reasons as [`area_paths`].
pub fn line_path(line_obj: &LineObject) -> PaperPath {
    PaperPath::from_bezier(line_obj.geometry())
}

/// Flatten tolerance (on-paper mm) for load-time flattening: area fill meshes,
/// fill patterns, and the arc-length placement of line decorations.
///
/// Stroked geometry does **not** use this — it keeps its curves all the way to
/// `Path::flatten`, which subdivides in screen space. This only has to be finer
/// than a fill edge tucked under its own outline stroke.
pub const BEZIER_FLATTEN_ERROR_MM: f32 = 0.005;

/// Append the points of the cubic `p0`–`h1`–`h2`–`p3` *after* its start, so
/// segments chain without duplicating vertices. The subdivision is epaint's own
/// adaptive flattener, and works in whichever space the caller passes in.
fn push_flattened_cubic(
    p0: Pos2,
    h1: Pos2,
    h2: Pos2,
    p3: Pos2,
    tolerance: f32,
    out: &mut Vec<Pos2>,
) {
    CubicBezierShape::from_points_stroke(
        [p0, h1, h2, p3],
        false,
        Color32::TRANSPARENT,
        Stroke::NONE,
    )
    .for_each_flattened_with_t(tolerance, &mut |p, _t| out.push(p));
}

/// A mixed straight/cubic path in paper space (mm, y-up) — the loader's working
/// form of what becomes a paint-ready `Path`, and the one thing that still
/// knows the file's own Bézier control points.
#[derive(Clone)]
pub struct PaperPath {
    /// Anchor vertices, in order.
    pub anchors: Vec<Pos2>,
    /// Handles of the cubic segments, sparse and ordered: `(i, [h1, h2])`
    /// curves the segment from `anchors[i]` to `anchors[i + 1]`.
    pub curves: Vec<(u32, [Pos2; 2])>,
    /// Per-anchor OMAP dash points, parallel to `anchors`.
    pub dash_points: Vec<bool>,
}

impl PaperPath {
    /// Rebuild the file's own path from `omap`'s [`BezierPath`], keeping each
    /// segment's curvature and every vertex's dash-point flag.
    ///
    /// A path with no segments yields an empty `PaperPath`; callers already
    /// reject anything with fewer than two anchors.
    pub fn from_bezier(path: &BezierPath) -> Self {
        let vertex_is_dash_point = path.vertex_is_dash_point();
        let mut anchors = Vec::with_capacity(path.num_vertices());
        let mut curves = Vec::new();
        let mut dash_points = Vec::with_capacity(path.num_vertices());

        for (i, (segment, starts_dash_point)) in path.segments().enumerate() {
            if i == 0 {
                anchors.push(to_pos2(segment.start()));
                dash_points.push(starts_dash_point);
            }
            if let BezierSegment::Bezier(curve) = segment {
                curves.push((i as u32, [to_pos2(curve.handle1), to_pos2(curve.handle2)]));
            }
            anchors.push(to_pos2(segment.end()));
            dash_points.push(vertex_is_dash_point.get(i + 1).copied().unwrap_or(false));
        }
        Self {
            anchors,
            curves,
            dash_points,
        }
    }

    /// Flatten in paper space, for the load-time consumers listed on
    /// [`BEZIER_FLATTEN_ERROR_MM`]. Returns points and their dash-point flags,
    /// parallel and the same length.
    pub fn flatten(&self, tolerance_mm: f32) -> (Vec<Pos2>, Vec<bool>) {
        flatten_anchors(
            &self.anchors,
            &self.dash_points,
            |i| self.curves.get(i).copied(),
            tolerance_mm,
            // No decimation: these points feed the nav and terrain models,
            // which measure ground rather than pixels.
            0.0,
        )
    }

    /// Bounding box of the control polygon — anchors *and* handles. A cubic
    /// never leaves its control points' convex hull, so this always contains the
    /// drawn curve, which is all viewport culling needs.
    pub fn control_bounds(&self) -> (Pos2, Pos2) {
        let handles = self.curves.iter().flat_map(|(_, h)| h.iter());
        bounds_of(self.anchors.iter().chain(handles).copied())
    }
}

/// Walk `anchors`, subdividing the segments that carry handles, and return the
/// points with their dash-point flags (parallel, same length).
///
/// Shared by the paper- and screen-space flatteners: everything is already in
/// whichever space the caller draws in, so the screen-space one projects handles
/// lazily. `curve_at(n)` yields the `n`-th cubic as `(segment, [h1, h2])`,
/// ordered, and `None` past the end.
///
/// Public beyond this crate for `omap-render`'s own screen-space flattener,
/// which projects handles lazily rather than needing a widened `PaperPath`.
pub fn flatten_anchors(
    anchors: &[Pos2],
    dash_points: &[bool],
    curve_at: impl Fn(usize) -> Option<(u32, [Pos2; 2])>,
    tolerance: f32,
    min_step: f32,
) -> (Vec<Pos2>, Vec<bool>) {
    let flag = |i: usize| dash_points.get(i).copied().unwrap_or(false);
    let Some(&first) = anchors.first() else {
        return (Vec::new(), Vec::new());
    };

    let mut points = vec![first];
    let mut flags = vec![flag(0)];
    let mut cursor = 0usize;
    for (i, &anchor) in anchors.iter().enumerate().skip(1) {
        let segment = (i - 1) as u32;
        while curve_at(cursor).is_some_and(|(s, _)| s < segment) {
            cursor += 1;
        }
        let before = points.len();
        if let Some((s, [h1, h2])) = curve_at(cursor)
            && s == segment
        {
            let start = points[before - 1];
            push_flattened_cubic(start, h1, h2, anchor, tolerance, &mut points);
        }
        match points.len() > before {
            // The flattener finishes at t = 1.0, i.e. on this anchor; pin that
            // last point to the anchor exactly. Ring closure and dash points
            // are compared by equality, so a last-bit difference would matter.
            true => *points.last_mut().expect("just pushed") = anchor,
            // A straight segment, or a curve degenerate enough that epaint
            // emitted nothing for it.
            false => points.push(anchor),
        }
        flags.resize(points.len() - 1, false);
        flags.push(flag(i));
    }
    decimate(points, flags, min_step)
}

/// Drop vertices closer to the one before them than `min_step` pixels.
///
/// A map drawn at 1:4000 and shown whole puts most of a line's vertices inside
/// the same pixel as their neighbour, where they draw nothing the neighbour
/// does not — but a stroked polyline costs triangles per vertex either way.
/// Both ends survive regardless, so nothing shortens and a ring still closes on
/// itself; so does every dash point, because that is the mapper saying the
/// pattern has to land there.
fn decimate(points: Vec<Pos2>, flags: Vec<bool>, min_step: f32) -> (Vec<Pos2>, Vec<bool>) {
    if min_step <= 0.0 || points.len() < 3 {
        return (points, flags);
    }
    let last = points.len() - 1;
    let min_step_sq = min_step * min_step;
    let mut kept = Vec::with_capacity(points.len());
    let mut kept_flags = Vec::with_capacity(flags.len());
    let mut previous = points[0];
    for (i, (&p, &flagged)) in points.iter().zip(&flags).enumerate() {
        if i == 0 || i == last || flagged || previous.distance_sq(p) >= min_step_sq {
            kept.push(p);
            kept_flags.push(flagged);
            previous = p;
        }
    }
    (kept, kept_flags)
}

/// Bounding box of `points`. An empty iterator yields an inverted box
/// (`+inf`..`-inf`), which is the identity for combining boxes and which every
/// `min.x.is_finite()` check downstream reads as "nothing here".
pub fn bounds_of(points: impl Iterator<Item = Pos2>) -> (Pos2, Pos2) {
    LineString(points.map(|p| geo::Coord { x: p.x, y: p.y }).collect())
        .bounding_rect()
        .map(|r| {
            (
                Pos2::new(r.min().x, r.min().y),
                Pos2::new(r.max().x, r.max().y),
            )
        })
        .unwrap_or((
            Pos2::new(f32::INFINITY, f32::INFINITY),
            Pos2::new(f32::NEG_INFINITY, f32::NEG_INFINITY),
        ))
}

/// Distance from `p` to the segment `a`-`b`, ends included.
#[cfg(test)]
pub fn distance_to_segment(p: Pos2, a: Pos2, b: Pos2) -> f32 {
    Euclidean.distance(
        &geo::Point::new(p.x, p.y),
        &Line::new((a.x, a.y), (b.x, b.y)),
    )
}

/// Euclidean length of a flattened paper or screen path.
pub fn path_length(points: &[Pos2]) -> f32 {
    Euclidean.length(&LineString(
        points
            .iter()
            .map(|p| geo::Coord { x: p.x, y: p.y })
            .collect(),
    ))
}

/// Whether `p` is strictly inside the ring, excluding its boundary.
/// Open rings are closed by the polygon constructor.
#[cfg(test)]
pub fn point_in_ring(p: Pos2, ring: &[Pos2]) -> bool {
    Polygon::new(
        LineString(ring.iter().map(|p| geo::Coord { x: p.x, y: p.y }).collect()),
        vec![],
    )
    .contains(&geo::Point::new(p.x, p.y))
}

#[cfg(test)]
mod tests {
    use omap::geo_types::LineString;
    use omap::objects::BezierString;

    use super::*;

    fn coord(x: f64, y: f64) -> Coord {
        Coord { x, y }
    }

    /// A straight-segment fixture path through `points`, with the dash flag of
    /// every vertex.
    fn straight(points: &[(f64, f64)], dash_points: &[bool]) -> BezierPath {
        let line: LineString = points.iter().map(|&(x, y)| coord(x, y)).collect();
        let mut path = BezierPath::from(line);
        for (i, &flag) in dash_points.iter().enumerate() {
            path.set_vertex_is_dash_point(i, flag);
        }
        path
    }

    /// A fixture path over explicit segments, for the cases where curvature is
    /// the subject.
    fn mixed(segments: Vec<BezierSegment>, dash_points: Vec<bool>) -> BezierPath {
        BezierPath::new(BezierString::new(segments), dash_points).expect("valid fixture path")
    }

    fn flatten(path: BezierPath, tolerance: f32) -> (Vec<Pos2>, Vec<bool>) {
        PaperPath::from_bezier(&path).flatten(tolerance)
    }

    #[test]
    fn a_cubic_segment_keeps_its_handles_and_a_straight_one_does_not() {
        let path = PaperPath::from_bezier(&mixed(
            vec![
                BezierSegment::new(coord(0.0, 0.0), None, coord(1.0, 0.0)),
                BezierSegment::new(
                    coord(1.0, 0.0),
                    Some((coord(1.0, 1.0), coord(2.0, 1.0))),
                    coord(2.0, 0.0),
                ),
            ],
            vec![false, false, false],
        ));
        // Three anchors, and only the second segment carries handles — the
        // curvature the painter needs is still there, unflattened.
        assert_eq!(path.anchors.len(), 3);
        assert_eq!(path.curves.len(), 1);
        assert_eq!(path.curves[0].0, 1, "handles belong to the second segment");
        assert_eq!(path.curves[0].1, [Pos2::new(1.0, 1.0), Pos2::new(2.0, 1.0)]);
        // The control polygon bounds the curve, so it bounds the drawn shape.
        let (min, max) = path.control_bounds();
        assert_eq!((min, max), (Pos2::new(0.0, 0.0), Pos2::new(2.0, 1.0)));
    }

    #[test]
    fn a_tighter_tolerance_buys_more_points_on_the_same_curve() {
        let curve = || {
            mixed(
                vec![BezierSegment::new(
                    coord(0.0, 0.0),
                    Some((coord(0.0, 10.0), coord(10.0, 10.0))),
                    coord(10.0, 0.0),
                )],
                vec![false, false],
            )
        };
        let coarse = flatten(curve(), 1.0).0.len();
        let fine = flatten(curve(), 0.001).0.len();
        assert!(
            fine > coarse * 4,
            "flattening should scale with the tolerance ({fine} vs {coarse})"
        );
    }

    #[test]
    fn flatten_all_line_segments_flags_nothing() {
        let path = straight(
            &[(0.0, 0.0), (1.0, 0.0), (2.0, 0.0), (3.0, 0.0)],
            &[false, false, false, false],
        );
        let (points, dash_points) = flatten(path, BEZIER_FLATTEN_ERROR_MM);
        assert_eq!(points.len(), 4);
        assert_eq!(dash_points, vec![false, false, false, false]);
    }

    #[test]
    fn flatten_marks_only_the_flagged_segment_end() {
        let path = mixed(
            vec![BezierSegment::new(
                coord(0.0, 0.0),
                Some((coord(0.0, 10.0), coord(10.0, 10.0))),
                coord(10.0, 0.0),
            )],
            vec![false, true],
        );
        let (points, dash_points) = flatten(path, 0.5);
        assert!(
            points.len() > 2,
            "a curved segment should subdivide into multiple points"
        );
        assert!(!dash_points[0], "this path's start is not flagged");
        assert!(*dash_points.last().unwrap());
        assert!(dash_points[1..dash_points.len() - 1].iter().all(|&d| !d));
    }

    #[test]
    fn an_open_paths_first_vertex_can_be_a_dash_point() {
        // `BezierPath` carries one flag per *vertex*, so an open path's very
        // first one is expressible — no reaching back into the raw coordinates.
        let path = straight(&[(0.0, 0.0), (1.0, 0.0), (2.0, 0.0)], &[true, false, true]);
        let (_points, dash_points) = flatten(path, BEZIER_FLATTEN_ERROR_MM);
        assert_eq!(dash_points, vec![true, false, true]);
    }

    #[test]
    fn flatten_attributes_flags_to_the_right_segment_boundary() {
        // Mirrors the shape of omap-rs's own `dash_flags_align_with_segment_end_vertices`
        // test: Line, Bezier, Line segments with flags [true, true, false].
        let path = mixed(
            vec![
                BezierSegment::new(coord(0.0, 0.0), None, coord(1.0, 0.0)),
                BezierSegment::new(
                    coord(1.0, 0.0),
                    Some((coord(1.0, 1.0), coord(2.0, 1.0))),
                    coord(2.0, 1.0),
                ),
                BezierSegment::new(coord(2.0, 1.0), None, coord(3.0, 0.0)),
            ],
            vec![false, true, true, false],
        );
        let (_points, dash_points) = flatten(path, 0.05);
        // Exactly two `true` entries, landing on the two flagged segment ends,
        // not on any interior curve subdivision or the path's final vertex.
        assert_eq!(dash_points.iter().filter(|&&d| d).count(), 2);
        assert!(!dash_points[0]);
        assert!(!*dash_points.last().unwrap());
    }

    #[test]
    fn flatten_closed_ring_can_flag_the_seam_vertex() {
        // Mirrors omap-rs's own `closed_path_combines_first_and_closing_vertex_dash_flags`:
        // a ring whose seam vertex is flagged, which the crate reports at both
        // ends of the vertex vector since they are the same vertex.
        let path = straight(&[(0.0, 0.0), (1.0, 0.0), (0.0, 0.0)], &[true, false, true]);
        let (points, dash_points) = flatten(path, BEZIER_FLATTEN_ERROR_MM);
        assert_eq!(points.len(), 3);
        assert_eq!(dash_points, vec![true, false, true]);
    }

    #[test]
    fn flatten_empty_path_is_empty() {
        assert_eq!(
            flatten(BezierPath::empty(), BEZIER_FLATTEN_ERROR_MM),
            (Vec::new(), Vec::new())
        );
    }

    fn at(x: f32, y: f32) -> Pos2 {
        Pos2::new(x, y)
    }

    #[test]
    fn a_degenerate_segment_is_a_point() {
        assert!((distance_to_segment(at(3.0, 4.0), at(0.0, 0.0), at(0.0, 0.0)) - 5.0).abs() < 1e-5);
    }

    #[test]
    fn distance_to_a_segment_stops_at_its_ends() {
        let (a, b) = (at(0.0, 0.0), at(10.0, 0.0));
        assert!((distance_to_segment(at(5.0, 3.0), a, b) - 3.0).abs() < 1e-5);
        assert!((distance_to_segment(at(13.0, 4.0), a, b) - 5.0).abs() < 1e-5);
    }

    #[test]
    fn a_point_is_inside_a_ring_by_the_even_odd_rule() {
        let square = [at(0.0, 0.0), at(10.0, 0.0), at(10.0, 10.0), at(0.0, 10.0)];
        assert!(point_in_ring(at(5.0, 5.0), &square));
        assert!(!point_in_ring(at(15.0, 5.0), &square));
        assert!(!point_in_ring(at(-1.0, 5.0), &square));
    }

    #[test]
    fn the_notch_of_a_concave_ring_is_outside_it() {
        let u = [
            at(0.0, 0.0),
            at(9.0, 0.0),
            at(9.0, 9.0),
            at(6.0, 9.0),
            at(6.0, 3.0),
            at(3.0, 3.0),
            at(3.0, 9.0),
            at(0.0, 9.0),
        ];
        assert!(point_in_ring(at(1.0, 5.0), &u));
        assert!(!point_in_ring(at(4.5, 6.0), &u), "the notch of the U");
    }

    #[test]
    fn fewer_than_three_points_enclose_nothing() {
        assert!(!point_in_ring(at(0.0, 0.0), &[]));
        assert!(!point_in_ring(at(0.5, 0.0), &[at(0.0, 0.0), at(1.0, 0.0)]));
    }
}
