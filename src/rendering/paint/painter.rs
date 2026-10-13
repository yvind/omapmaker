//! Converts [`Primitive`]s into egui [`Shape`]s on the screen.

use eframe::egui::{self, Color32, Pos2, Shape, Stroke, Vec2};

use super::tuning::RenderTuning;
use super::view::View;
use crate::rendering::geo::Mercator;
use crate::rendering::render::{
    AreaPattern, BorderSide, Kind, LineDash, PointMark, Primitive, dash_runs, triangulate_rings,
};

/// Draw all primitives as `view` says: skipping those outside its bounds,
/// mapping geographic positions to the screen and scaling on-paper millimetre
/// widths/radii to pixels.
pub fn paint_primitives<F: Fn(Mercator) -> Pos2>(
    painter: &egui::Painter,
    prims: &[Primitive],
    view: &View<F>,
) {
    let (view_min, view_max, mm_to_px, tuning) = (view.min, view.max, view.mm_to_px, view.tuning);
    let to_screen = &view.to_screen;
    for prim in prims {
        if !prim_visible(prim, view_min, view_max) {
            continue;
        }
        if !matches!(prim.kind, Kind::UnresolvedPoint { .. })
            && !prim_significant(prim, to_screen, tuning.min_visible_px)
        {
            continue;
        }
        match &prim.kind {
            Kind::Area {
                exterior,
                interiors,
                fill,
                fill_indices,
            } => {
                // Building a mesh (rather than `Shape::convex_polygon`) is what
                // makes concave outlines render correctly and lets interior rings
                // punch real holes — `convex_polygon` both mis-fills concave
                // shapes and shoots long AA spikes out of reflex vertices.
                if !fill_indices.is_empty() {
                    let mut mesh = egui::Mesh::default();
                    for p in exterior.iter().chain(interiors.iter().flatten()) {
                        mesh.colored_vertex(to_screen(*p), *fill);
                    }
                    for tri in fill_indices.as_chunks::<3>().0 {
                        mesh.add_triangle(tri[0], tri[1], tri[2]);
                    }
                    painter.add(Shape::mesh(mesh));
                } else {
                    // Degenerate triangulation: fall back to the exterior outline.
                    let pts: Vec<Pos2> = exterior.iter().map(|p| to_screen(*p)).collect();
                    painter.add(Shape::closed_line(pts, Stroke::new(0.75, *fill)));
                }
            }
            Kind::Pattern(pattern) => match pattern {
                AreaPattern::Hatch {
                    segments,
                    color,
                    width_mm,
                    spacing_mm,
                } => {
                    let w = (width_mm * mm_to_px).max(0.5);
                    let stride =
                        stride_for_pitch(spacing_mm * mm_to_px, tuning.min_pattern_pitch_px);
                    for seg in segments.iter().step_by(stride) {
                        painter.add(Shape::line_segment(
                            [to_screen(seg[0]), to_screen(seg[1])],
                            Stroke::new(w, *color),
                        ));
                    }
                }
                AreaPattern::Dots { marks, pitch_mm } => {
                    let stride = stride_for_pitch(pitch_mm * mm_to_px, tuning.min_pattern_pitch_px);
                    for mark in marks.iter().step_by(stride) {
                        paint_mark(painter, mark, to_screen, mm_to_px, tuning);
                    }
                }
            },
            Kind::FallbackOutline { ring } => {
                let pts: Vec<Pos2> = ring.iter().map(|p| to_screen(*p)).collect();
                painter.add(Shape::closed_line(
                    pts,
                    Stroke::new(0.75, Color32::from_gray(120)),
                ));
            }
            Kind::Line {
                path,
                color,
                width_mm,
                border,
                dash,
            } => {
                // Curves are flattened here, in screen space, at the zoom being
                // drawn — not at load time against a guessed on-paper tolerance.
                let (pts, dash_points) = path.flatten(
                    to_screen,
                    tuning.curve_tolerance_px,
                    tuning.min_vertex_step_px,
                );
                // Main fill stroke
                if let Some(color) = color {
                    let line = Polyline {
                        pts: &pts,
                        width_px: (width_mm * mm_to_px).max(0.75),
                        color: *color,
                        dash,
                        dash_points: &dash_points,
                    };
                    paint_polyline(painter, line, mm_to_px);
                }
                // Border
                if let Some(b) = border {
                    if let Some(side) = &b.left {
                        paint_border_rail(painter, &pts, *width_mm, side, 1.0, mm_to_px);
                    }
                    if let Some(side) = &b.right {
                        paint_border_rail(painter, &pts, *width_mm, side, -1.0, mm_to_px);
                    }
                }
            }
            Kind::UnresolvedPoint { at } => {
                // Element-less / unresolved symbol: a small dot so it shows.
                painter.circle_filled(to_screen(*at), 1.5, Color32::from_gray(60));
            }
            Kind::PointMark(mark) => paint_mark(painter, mark, to_screen, mm_to_px, tuning),
        }
    }
}

/// A flattened line and how it is stroked. `dash_points` is parallel to `pts`
/// and marks the vertices a dashed pattern restarts at.
#[derive(Clone, Copy)]
pub(super) struct Polyline<'a> {
    pub pts: &'a [Pos2],
    pub width_px: f32,
    pub color: Color32,
    pub dash: &'a Option<LineDash>,
    pub dash_points: &'a [bool],
}

/// Draw a polyline, solid or dashed. When dashed, the line is cut into runs at
/// `dash_points` and each run's pattern is rescaled to land on both its ends.
/// The drawn line stays unbroken; only the pattern restarts.
fn paint_polyline(painter: &egui::Painter, line: Polyline, mm_to_px: f32) {
    let Polyline {
        pts,
        width_px,
        color,
        dash,
        dash_points,
    } = line;
    let stroke = Stroke::new(width_px, color);
    match dash {
        Some(d) => {
            let dashes: Vec<f32> = d
                .dash_lengths_mm
                .iter()
                .map(|l| (l * mm_to_px).max(0.5))
                .collect();
            let gaps: Vec<f32> = d
                .gap_lengths_mm
                .iter()
                .map(|l| (l * mm_to_px).max(0.5))
                .collect();
            for (first, last) in dash_runs(pts.len(), dash_points) {
                let run = &pts[first..=last];
                let (run_dashes, run_gaps) = rescale_dash_pattern(run, &dashes, &gaps);
                painter.add(Shape::dashed_line_with_offset(
                    run,
                    stroke,
                    &run_dashes,
                    &run_gaps,
                    0.0,
                ));
            }
        }
        None => {
            painter.add(Shape::line(pts.to_vec(), stroke));
        }
    }
}

/// Uniformly scale `dashes`/`gaps` (screen px) so `run` holds a whole number of
/// periods *ending on a dash*, as Mapper does. The last trailing gap is never
/// drawn, so `n` periods fill `n · period − last gap`. Degenerate runs fall
/// back to the unscaled pattern.
pub(super) fn rescale_dash_pattern(
    run: &[Pos2],
    dashes: &[f32],
    gaps: &[f32],
) -> (Vec<f32>, Vec<f32>) {
    let period: f32 = dashes.iter().sum::<f32>() + gaps.iter().sum::<f32>();
    let last_gap = gaps.last().copied().unwrap_or(0.0);
    let run_length = crate::rendering::paper::path_length(run);
    if period <= 0.0 || run_length <= 0.0 {
        return (dashes.to_vec(), gaps.to_vec());
    }
    let periods = ((run_length + last_gap) / period).round().max(1.0);
    let fitted = periods * period - last_gap;
    if fitted <= 0.0 {
        return (dashes.to_vec(), gaps.to_vec());
    }
    let scale = run_length / fitted;
    (
        dashes.iter().map(|d| d * scale).collect(),
        gaps.iter().map(|g| g * scale).collect(),
    )
}

fn paint_border_rail(
    painter: &egui::Painter,
    pts: &[Pos2],
    main_width_mm: f32,
    side: &BorderSide,
    sign: f32,
    mm_to_px: f32,
) {
    let offset_mm = main_width_mm / 2.0 + side.shift_mm;
    let offset_px = offset_mm * mm_to_px * sign;
    let width_px = (side.width_mm * mm_to_px).max(0.75);
    let rail = offset_polyline(pts, offset_px);
    let stroke = Stroke::new(width_px, side.color);
    match &side.dash {
        Some(d) => {
            // The border's own dash (`BorderDash`) is always a single
            // dash+gap step — no dash groups, no OMAP dash-point runs like the
            // main line has — so the whole rail rescales as one run.
            let dashes: Vec<f32> = d
                .dash_lengths_mm
                .iter()
                .map(|l| (l * mm_to_px).max(0.5))
                .collect();
            let gaps: Vec<f32> = d
                .gap_lengths_mm
                .iter()
                .map(|l| (l * mm_to_px).max(0.5))
                .collect();
            let (run_dashes, run_gaps) = rescale_dash_pattern(&rail, &dashes, &gaps);
            painter.add(Shape::dashed_line_with_offset(
                &rail,
                stroke,
                &run_dashes,
                &run_gaps,
                0.0,
            ));
        }
        None => {
            painter.add(Shape::line(rail, stroke));
        }
    }
}

/// Offset a polyline perpendicular to its own direction by `offset_px`
/// (screen pixels), using each vertex's averaged neighbouring-segment normal.
/// A simple miter approximation — good enough for the thin border rails this
/// is used for, though it can spike on very sharp reversals.
pub(super) fn offset_polyline(pts: &[Pos2], offset_px: f32) -> Vec<Pos2> {
    if pts.len() < 2 || offset_px == 0.0 {
        return pts.to_vec();
    }
    let normal = |a: Pos2, b: Pos2| -> Vec2 {
        let d = b - a;
        let len = d.length();
        if len > 0.0 {
            Vec2::new(-d.y, d.x) / len
        } else {
            Vec2::ZERO
        }
    };
    let n = pts.len();
    (0..n)
        .map(|i| {
            let normal_at = if i == 0 {
                normal(pts[0], pts[1])
            } else if i == n - 1 {
                normal(pts[n - 2], pts[n - 1])
            } else {
                let avg = normal(pts[i - 1], pts[i]) + normal(pts[i], pts[i + 1]);
                if avg.length_sq() > 0.0 {
                    avg.normalized()
                } else {
                    normal(pts[i - 1], pts[i])
                }
            };
            pts[i] + normal_at * offset_px
        })
        .collect()
}

/// Draw one point-symbol mark, used both for a point object's own marks and
/// for each instance of a scattered area-pattern mark. Its geometry is
/// already an absolute geographic position (resolved once at load time), so
/// painting is just project + draw.
fn paint_mark<F: Fn(Mercator) -> Pos2>(
    painter: &egui::Painter,
    mark: &PointMark,
    to_screen: &F,
    mm_to_px: f32,
    tuning: &RenderTuning,
) {
    match mark {
        PointMark::Disc {
            center,
            radius_mm,
            color,
        } => {
            painter.circle_filled(to_screen(*center), (radius_mm * mm_to_px).max(0.5), *color);
        }
        PointMark::Ring {
            center,
            radius_mm,
            width_mm,
            color,
        } => {
            painter.circle_stroke(
                to_screen(*center),
                (radius_mm * mm_to_px).max(0.5),
                Stroke::new((width_mm * mm_to_px).max(0.5), *color),
            );
        }
        PointMark::Stroke {
            path,
            width_mm,
            color,
        } => {
            // Flattened here, in screen space, at the zoom being drawn — same
            // as a line object's own stroke (`Kind::Line`), so an element's
            // curves (an arc, a knoll's hook) stay exact at any zoom instead
            // of being pinned to a load-time tolerance.
            let (pts, _) = path.flatten(
                to_screen,
                tuning.curve_tolerance_px,
                tuning.min_vertex_step_px,
            );
            painter.add(Shape::line(
                pts,
                Stroke::new((width_mm * mm_to_px).max(0.5), *color),
            ));
        }
        PointMark::Fill { path, color } => {
            let (pts, _) = path.flatten(
                to_screen,
                tuning.curve_tolerance_px,
                tuning.min_vertex_step_px,
            );
            paint_point_fill(painter, &pts, *color);
        }
    }
}

/// A point-symbol fill, triangulated fresh each frame from its just-flattened
/// screen-space points. Not always convex — a frame-like outline built as one
/// ring is a real example, drawn by a boulder-field or out-of-bounds symbol —
/// so this earns the same mesh treatment [`Kind::Area`] gets above rather than
/// `Shape::convex_polygon`, which mis-fills exactly that shape.
fn paint_point_fill(painter: &egui::Painter, pts: &[Pos2], color: Color32) {
    if pts.len() < 3 {
        return;
    }
    let mut closed = pts.to_vec();
    if closed.first() != closed.last() {
        closed.push(pts[0]);
    }
    let indices = triangulate_rings(&closed, &[]);
    if indices.is_empty() {
        // Degenerate triangulation: fall back to the outline, the same as
        // `Kind::Area` does.
        painter.add(Shape::closed_line(pts.to_vec(), Stroke::new(0.75, color)));
        return;
    }
    let mut mesh = egui::Mesh::default();
    for p in &closed {
        mesh.colored_vertex(*p, color);
    }
    for tri in indices.as_chunks::<3>().0 {
        mesh.add_triangle(tri[0], tri[1], tri[2]);
    }
    painter.add(Shape::mesh(mesh));
}

/// Cheap cull: compare the primitive's precomputed geographic bounds against
/// the current viewport's, both known ahead of time — no projection needed.
pub(super) fn prim_visible(prim: &Primitive, view_min: Mercator, view_max: Mercator) -> bool {
    prim.max.x >= view_min.x
        && prim.min.x <= view_max.x
        && prim.max.y >= view_min.y
        && prim.min.y <= view_max.y
}

/// Whether a primitive is big enough on screen to be worth drawing. Two
/// Mercator calls are negligible next to meshing or flattening something under a
/// pixel across. The metric is bbox *diagonal*, so a long thin line is never
/// caught — only primitives tiny in both dimensions.
pub(super) fn prim_significant<F: Fn(Mercator) -> Pos2>(
    prim: &Primitive,
    to_screen: &F,
    min_visible_px: f32,
) -> bool {
    (to_screen(prim.max) - to_screen(prim.min)).length() >= min_visible_px
}

/// How many scattered pattern items (hatch lines, dots) to skip between the
/// ones actually drawn, given their on-screen spacing. A whole area can be
/// large on screen while it's densely scattered with individually sub-pixel
/// items (a forest polygon full of tiny tree dots), which `prim_significant`
/// alone cannot catch since it only sees the containing primitive's bbox.
pub(super) fn stride_for_pitch(pitch_px: f32, min_pattern_pitch_px: f32) -> usize {
    if pitch_px <= 0.0 {
        return 1;
    }
    ((min_pattern_pitch_px / pitch_px).ceil() as usize).max(1)
}

#[cfg(test)]
mod tests {

    use super::*;

    fn line(xs: &[f32]) -> Vec<Pos2> {
        xs.iter().map(|&x| Pos2::new(x, 0.0)).collect()
    }

    /// A screen projection at `scale` pixels per degree — stands in for zoom,
    /// same idea as `render::primitives`'s own `at_zoom` test helper.
    fn at_zoom(scale: f32) -> impl Fn(Mercator) -> Pos2 {
        move |p| Pos2::new(p.x as f32 * scale, p.y as f32 * scale)
    }

    fn prim_with_bbox(min: Mercator, max: Mercator) -> Primitive {
        Primitive {
            kind: Kind::FallbackOutline { ring: Vec::new() },
            z: 0,
            min,
            max,
        }
    }

    #[test]
    fn a_tiny_primitive_is_culled_when_zoomed_out_and_kept_when_zoomed_in() {
        // A 1e-6-degree-square primitive: negligible at a coarse zoom, but a
        // real building at a close one.
        let prim = prim_with_bbox(Mercator { x: 0.0, y: 0.0 }, Mercator { x: 1e-6, y: 1e-6 });
        let min_visible_px = RenderTuning::default().min_visible_px;

        assert!(
            !prim_significant(&prim, &at_zoom(1.0), min_visible_px),
            "a sub-pixel primitive should be culled"
        );
        assert!(
            prim_significant(&prim, &at_zoom(5_000_000.0), min_visible_px),
            "the same primitive should be kept once it's actually on screen"
        );
    }

    #[test]
    fn a_degenerate_zero_size_bbox_is_never_significant() {
        // `Kind::UnresolvedPoint` is built with `min == max` by construction
        // — the caller must exempt it explicitly rather than rely on this
        // function, which correctly treats a zero-size box as insignificant
        // at any zoom.
        let prim = prim_with_bbox(Mercator { x: 5.0, y: 5.0 }, Mercator { x: 5.0, y: 5.0 });
        assert!(!prim_significant(
            &prim,
            &at_zoom(1_000_000.0),
            RenderTuning::default().min_visible_px
        ));
    }

    #[test]
    fn stride_for_pitch_is_one_above_the_threshold() {
        let min_pattern_pitch_px = RenderTuning::default().min_pattern_pitch_px;
        assert_eq!(
            stride_for_pitch(min_pattern_pitch_px, min_pattern_pitch_px),
            1
        );
        assert_eq!(
            stride_for_pitch(min_pattern_pitch_px * 10.0, min_pattern_pitch_px),
            1
        );
    }

    #[test]
    fn stride_for_pitch_thins_sub_pixel_spacing() {
        // A pitch a tenth of the threshold should skip roughly nine out of
        // every ten items, landing the *effective* spacing back at or above
        // the threshold.
        let min_pattern_pitch_px = RenderTuning::default().min_pattern_pitch_px;
        let pitch = min_pattern_pitch_px / 10.0;
        let stride = stride_for_pitch(pitch, min_pattern_pitch_px);
        assert!(stride >= 10);
        assert!(pitch * stride as f32 >= min_pattern_pitch_px);
    }

    #[test]
    fn stride_for_pitch_never_zero() {
        let min_pattern_pitch_px = RenderTuning::default().min_pattern_pitch_px;
        assert_eq!(stride_for_pitch(0.0, min_pattern_pitch_px), 1);
        assert_eq!(stride_for_pitch(-1.0, min_pattern_pitch_px), 1);
    }

    #[test]
    fn rescale_fits_whole_dashes_and_ends_on_a_dash() {
        let pts = line(&[0.0, 10.0]); // 10px run
        let dashes = [3.0_f32];
        let gaps = [1.0_f32]; // nominal period 4px, doesn't divide 10 evenly
        let (run_dashes, run_gaps) = rescale_dash_pattern(&pts, &dashes, &gaps);
        let period: f32 = run_dashes.iter().sum::<f32>() + run_gaps.iter().sum::<f32>();
        // A run of `n` periods draws n dashes and only n-1 gaps: the trailing
        // gap falls off the end, so the run must finish flush with a dash.
        let periods = ((10.0 + run_gaps[0]) / period).round();
        assert!((periods * period - run_gaps[0] - 10.0).abs() < 1e-4);
        // Proportions between dash and gap are preserved.
        assert!((run_dashes[0] / run_gaps[0] - dashes[0] / gaps[0]).abs() < 1e-4);
    }

    #[test]
    fn rescale_degenerate_run_or_pattern_returns_input_unchanged() {
        let dashes = [3.0_f32];
        let gaps = [1.0_f32];

        let zero_length = line(&[5.0, 5.0]);
        let (d, g) = rescale_dash_pattern(&zero_length, &dashes, &gaps);
        assert_eq!(d, dashes);
        assert_eq!(g, gaps);

        let pts = line(&[0.0, 10.0]);
        let (d, g) = rescale_dash_pattern(&pts, &[0.0], &[0.0]);
        assert_eq!(d, vec![0.0]);
        assert_eq!(g, vec![0.0]);
        assert!(d[0].is_finite() && g[0].is_finite());
    }
}
