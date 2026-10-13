//! The drawable data model produced by [`super::draw_objects`]:
//! paint-ready geometry plus a flat, pre-resolved style, so painting never
//! re-walks the symbol graph or re-runs colour resolution.
//!
//! Every coordinate here is a [`Mercator`] point, resolved once at load
//! time, so painting turns a vertex into a pixel with a multiply and an add.

use crate::rendering::geo::Mercator;
use eframe::egui::{Color32, Pos2};

use crate::rendering::paper::flatten_anchors;

/// z-order for objects whose colour could not be resolved: put them far in the
/// background so real, coloured features paint over them. Shared with
/// `omap-paper`'s own colour-priority fallback, so a face the colour table
/// genuinely has nothing to say about sinks to the same place either way.
pub(super) use crate::rendering::color::UNKNOWN_PRIORITY as Z_UNKNOWN;

/// A single thing to draw, already reduced to paint-space geometry + a flat
/// style.
pub struct Primitive {
    pub kind: Kind,
    /// Draw order key. Lower is painted *later*, i.e. on top — as Mapper treats
    /// colour priority, where the top of the colour list is priority 0.
    pub z: i32,
    /// Geographic bounds, used for cheap viewport culling while panning —
    /// precomputed once at load time rather than reprojected every frame.
    pub min: Mercator,
    pub max: Mercator,
}

pub enum Kind {
    /// A filled area, drawn as a triangle mesh: `convex_polygon` shoots long
    /// anti-aliasing spikes out of reflex vertices and cannot punch holes.
    Area {
        exterior: Vec<Mercator>,
        interiors: Vec<Vec<Mercator>>,
        fill: Color32,
        /// Ear-clipping triangulation of the filled region: indices into the
        /// concatenation `exterior ++ interiors.concat()`. Empty when
        /// triangulation failed (degenerate/self-intersecting input).
        fill_indices: Vec<u32>,
    },
    /// A faint fallback outline for an area whose symbol resolved to nothing
    /// drawable, so it is at least visible.
    FallbackOutline { ring: Vec<Mercator> },
    /// A single area fill pattern (hatching or scattered points), its own
    /// primitive so it layers at its pattern colour's priority.
    Pattern(AreaPattern),
    /// A stroked path (walls, fences, paths, area outlines, ...).
    Line {
        path: Path,
        /// `None` when this component's colour is `NoColor`
        color: Option<Color32>,
        /// Nominal on-paper width in millimetres.
        width_mm: f32,
        /// Optional border rails drawn alongside the main stroke
        border: Option<LineBorder>,
        /// Dash pattern (on-paper mm); `None` for a solid line.
        dash: Option<LineDash>,
    },
    /// One resolved piece of a point symbol, its own primitive at its own colour
    /// priority so a point's parts interleave correctly with other objects.
    PointMark(PointMark),
    /// A point whose symbol resolved to no drawable marks: a small fallback dot
    /// so the object is still visible.
    UnresolvedPoint { at: Mercator },
}

/// A stroked path, kept as the file's own mixed straight/cubic Bézier path
/// rather than flattened once at load time.
///
/// The question is which *space* to flatten in. A tolerance fixed in paper
/// millimetres is a guess about how far the user will zoom. Keeping the control
/// points lets [`Path::flatten`] subdivide in screen space per frame, at the
/// zoom actually being drawn — exact at any zoom, and cheaper at normal ones,
/// since a curve costs four projected points instead of a hundred-odd.
pub struct Path {
    /// Anchor vertices, in order.
    pub anchors: Vec<Mercator>,
    /// Handles of the cubic segments, sparse and ordered: `(i, [h1, h2])`
    /// curves the segment from `anchors[i]` to `anchors[i + 1]`. Unlisted
    /// segments are straight.
    pub curves: Vec<(u32, [Mercator; 2])>,
    /// Per-anchor OMAP dash points (flag 32), parallel to `anchors`: vertices
    /// the mapper placed by hand to say "the pattern must land here". A dashed
    /// line's pattern restarts at each; the line itself is not broken.
    pub dash_points: Vec<bool>,
}

impl Path {
    /// Project to screen space and flatten there, subdividing each cubic until
    /// it sits within `tolerance_px` of the true curve *at this zoom*.
    ///
    /// Returns the points and their dash-point flags — parallel, same length,
    /// with only anchors ever flagged.
    ///
    /// Vertices landing within `min_step_px` of the one before them are
    /// dropped: at a zoom that fits the whole sheet most of a line's vertices
    /// share a pixel with their neighbour, and each one still costs triangles.
    pub fn flatten<F: Fn(Mercator) -> Pos2>(
        &self,
        to_screen: F,
        tolerance_px: f32,
        min_step_px: f32,
    ) -> (Vec<Pos2>, Vec<bool>) {
        let anchors: Vec<Pos2> = self.anchors.iter().map(|p| to_screen(*p)).collect();
        flatten_anchors(
            &anchors,
            &self.dash_points,
            |i| {
                self.curves
                    .get(i)
                    .map(|&(seg, [h1, h2])| (seg, [to_screen(h1), to_screen(h2)]))
            },
            tolerance_px,
            min_step_px,
        )
    }
}

/// One drawable piece of a point symbol, fully resolved to absolute geographic
/// positions (element offsets already combined with the point's own location).
pub enum PointMark {
    Disc {
        center: Mercator,
        radius_mm: f32,
        color: Color32,
    },
    Ring {
        center: Mercator,
        radius_mm: f32,
        width_mm: f32,
        color: Color32,
    },
    /// A line element's outline (an arc, a hook, ...). Kept as a curve-capable
    /// [`Path`] rather than points flattened once at load time, so it stays
    /// exact at any zoom the same way a line object's own stroke does.
    Stroke {
        path: Path,
        width_mm: f32,
        color: Color32,
    },
    /// An area element's fill (e.g. a boulder's triangle). Same reasoning as
    /// `Stroke` for keeping `path` curve-capable — but unlike `Stroke`, a
    /// filled point-symbol shape is not always convex (a frame-like outline
    /// built as one ring is a real example), so the painter has to triangulate
    /// `Path::flatten`'s screen-space points — see [`super::triangulate_rings`]
    /// — rather than fan-filling them as if they were.
    Fill { path: Path, color: Color32 },
}

/// A line's borders
pub struct LineBorder {
    pub left: Option<BorderSide>,
    pub right: Option<BorderSide>,
}

/// One border rail: a stroke running parallel to the main line's centreline,
/// offset outward from the main line's own edge (not the centreline itself).
pub struct BorderSide {
    pub color: Color32,
    /// On-paper width in millimetres.
    pub width_mm: f32,
    /// On-paper gap in millimetres between the main line's edge and this
    /// border's near edge
    pub shift_mm: f32,
    /// The border's own dash pattern (`BorderDash`), independent of the main
    /// line's. `None` for a solid rail.
    pub dash: Option<LineDash>,
}

/// A dash/gap pattern for a line, in on-paper millimetres. `dash_lengths_mm`
/// and `gap_lengths_mm` are parallel and cycle together, one dash+gap per
/// step — a plain dash is a single-element pattern; a dash *group* (several
/// short dashes separated by a small in-group gap, then one larger gap before
/// the next group) is one element per dash, with the group's larger break as
/// the final gap and the smaller in-group break as the others.
#[derive(Clone)]
pub struct LineDash {
    pub dash_lengths_mm: Vec<f32>,
    pub gap_lengths_mm: Vec<f32>,
}

/// A resolved area fill pattern, its geometry already clipped to the polygon
/// and projected to geographic positions, so painting is just project + draw.
pub enum AreaPattern {
    /// Parallel hatch lines.
    Hatch {
        segments: Vec<[Mercator; 2]>,
        color: Color32,
        width_mm: f32,
        /// On-paper millimetres between lines, carried through from the
        /// symbol's own spacing so paint time can thin sub-pixel-dense
        /// hatching without re-deriving it from the segment geometry.
        spacing_mm: f32,
    },
    /// Every mark of the pattern's point symbol (base circle plus any element
    /// sub-shapes, e.g. a boulder-field pattern's triangles), fully resolved
    /// at each scattered position. A multi-mark symbol becomes several marks
    /// per position, so each still layers at its own colour priority —
    /// mirrors `Kind::PointMark` for point objects.
    Dots {
        marks: Vec<PointMark>,
        /// On-paper millimetres between scattered positions — the tighter of
        /// the pattern's row/along spacing, so paint time can thin
        /// sub-pixel-dense scatter without re-deriving it from the marks.
        pitch_mm: f32,
    },
}

#[cfg(test)]
mod tests {

    use super::*;
    use crate::rendering::paper::distance_to_segment;

    /// A quarter-ish arc from (0,0) to (10,0), plus a straight run out to
    /// (20,0) whose end anchor is a dash point.
    fn curved_path() -> Path {
        let at = |x, y| Mercator { x, y };
        Path {
            anchors: vec![at(0.0, 0.0), at(10.0, 0.0), at(20.0, 0.0)],
            curves: vec![(0, [at(0.0, 10.0), at(10.0, 10.0)])],
            dash_points: vec![false, false, true],
        }
    }

    /// Screen projection at `scale` pixels per unit — stands in for zoom.
    fn at_zoom(scale: f32) -> impl Fn(Mercator) -> Pos2 {
        move |p| Pos2::new(p.x as f32 * scale, p.y as f32 * scale)
    }

    #[test]
    fn flattening_follows_the_zoom_it_is_drawn_at() {
        // The point of keeping control points: the same path spends vertices
        // in proportion to how big it is on screen, instead of committing to
        // one on-paper tolerance at load time.
        let path = curved_path();
        let far = path.flatten(at_zoom(1.0), 0.1, 0.0).0.len();
        let near = path.flatten(at_zoom(100.0), 0.1, 0.0).0.len();
        assert!(
            near > far * 5,
            "zooming in should refine the curve ({near} vs {far} points)"
        );
    }

    #[test]
    fn flattening_stays_within_tolerance_of_the_true_curve() {
        let path = curved_path();
        let tolerance = 0.1;
        let (points, _) = path.flatten(at_zoom(50.0), tolerance, 0.0);

        // Sample the true cubic densely and check every sample is close to the
        // flattened polyline. Control points, in the same screen space:
        let p = |i: usize| at_zoom(50.0)(path.anchors[i]);
        let h = |i: usize| at_zoom(50.0)(path.curves[0].1[i]);
        let (p0, h1, h2, p3) = (p(0), h(0), h(1), p(1));
        for step in 0..=200 {
            let t = step as f32 / 200.0;
            let (u, tt) = (1.0 - t, t);
            let exact = Pos2::new(
                u * u * u * p0.x
                    + 3.0 * u * u * tt * h1.x
                    + 3.0 * u * tt * tt * h2.x
                    + tt * tt * tt * p3.x,
                u * u * u * p0.y
                    + 3.0 * u * u * tt * h1.y
                    + 3.0 * u * tt * tt * h2.y
                    + tt * tt * tt * p3.y,
            );
            let nearest = points
                .windows(2)
                .map(|w| distance_to_segment(exact, w[0], w[1]))
                .fold(f32::INFINITY, f32::min);
            assert!(
                nearest <= tolerance,
                "curve sample at t={t} is {nearest}px off the flattened path"
            );
        }
    }

    #[test]
    fn dash_points_stay_on_their_anchors_through_flattening() {
        let path = curved_path();
        let (points, dash_points) = path.flatten(at_zoom(30.0), 0.1, 0.0);
        assert_eq!(points.len(), dash_points.len());
        // Only the final anchor is flagged, and the curve's own subdivisions
        // must not have shifted the flag off it.
        assert_eq!(dash_points.iter().filter(|&&d| d).count(), 1);
        assert!(*dash_points.last().expect("non-empty"));
        assert_eq!(
            *points.last().expect("non-empty"),
            at_zoom(30.0)(path.anchors[2])
        );
    }
}
