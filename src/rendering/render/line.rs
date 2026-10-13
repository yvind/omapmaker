use eframe::egui::{Color32, Pos2, Vec2};
use geo::{Distance, Euclidean, InterpolatePoint};
use omap::{
    colors::ColorSet,
    symbols::{
        BorderDash, BorderStyle, DashStyle, DashSymbol, GroupDashes, LineSymbol, LineSymbolBorder,
        MidSymbol, PointSymbol, PublicOrPrivateSymbol, Symbol, SymbolId,
    },
};

use super::color::{min_prio, resolve_color};
use super::context::Context;
use super::geometry::dash_runs;
use super::point::{LocalMark, build_point_marks, mark_extent, resolve_mark, rotate_marks};
use super::primitives::{BorderSide, Kind, LineBorder, LineDash, Primitive};

pub(super) struct LineStroke {
    pub color: Option<Color32>,
    pub z: i32,
    pub width_mm: f32,
    pub border: Option<LineBorder>,
    pub dash: Option<LineDash>,
}

/// Resolve every stroke a line symbol contributes: one for a plain line
/// symbol, one per private component for a combined line symbol (public
/// components are followed transparently). A component is skipped only if
/// neither its own colour nor its border's colour resolves.
pub(super) fn line_style(symbol: &Symbol, ctx: Context) -> Vec<LineStroke> {
    let mut out = Vec::new();
    collect_line_style(symbol, ctx, 0, &mut out);
    out
}

fn collect_line_style(symbol: &Symbol, ctx: Context, depth: u8, out: &mut Vec<LineStroke>) {
    let Context {
        symbol_set, colors, ..
    } = ctx;
    if depth > 8 {
        return;
    }
    match symbol {
        Symbol::Line(l) => {
            if let Some(stroke) = line_symbol_stroke(l, colors) {
                out.push(stroke);
            }
        }
        Symbol::CombinedLine(s) => {
            for c in s.components() {
                match c {
                    PublicOrPrivateSymbol::Private(l) => {
                        if let Some(stroke) = line_symbol_stroke(l, colors) {
                            out.push(stroke);
                        }
                    }
                    PublicOrPrivateSymbol::Public(id) => {
                        if let Some(sym) = symbol_set.get(SymbolId::from(*id)) {
                            collect_line_style(sym.symbol(), ctx, depth + 1, out);
                        }
                    }
                }
            }
        }
        _ => {}
    }
}

fn line_symbol_stroke(l: &LineSymbol, colors: &ColorSet) -> Option<LineStroke> {
    let border = l.border.as_ref().and_then(|b| resolve_border(b, colors));
    let main = resolve_color(&l.color, colors);
    let z = main
        .map(|(_, z)| z)
        .or_else(|| border.as_ref().map(|(_, z)| *z))?;
    Some(LineStroke {
        color: main.map(|(c, _)| c),
        z,
        width_mm: l.line_width.get() as f32,
        border: border.map(|(b, _)| b),
        dash: dash_from_style(&l.dash_style),
    })
}

fn resolve_border(style: &BorderStyle, colors: &ColorSet) -> Option<(LineBorder, i32)> {
    let (left_style, right_style) = match style {
        BorderStyle::SymmetricBorder { both } => (both, both),
        BorderStyle::AsymmetricBorder { left, right } => (left, right),
    };
    let left = resolve_border_side(left_style, colors);
    let right = resolve_border_side(right_style, colors);
    let (_, z) = min_prio(
        left.as_ref().map(|(s, z)| (s.color, *z)),
        right.as_ref().map(|(s, z)| (s.color, *z)),
    )?;
    Some((
        LineBorder {
            left: left.map(|(s, _)| s),
            right: right.map(|(s, _)| s),
        },
        z,
    ))
}

fn resolve_border_side(b: &LineSymbolBorder, colors: &ColorSet) -> Option<(BorderSide, i32)> {
    let (color, z) = resolve_color(&b.color, colors)?;
    Some((
        BorderSide {
            color,
            width_mm: b.width.get() as f32,
            shift_mm: b.shift.get() as f32,
            dash: dash_from_border(&b.dashed),
        },
        z,
    ))
}

/// A border's own dash pattern (`dashed`/`dash_length`/`break_length` on the
/// `<border>` element) — always a single dash+gap step, unlike a main line's
/// dash groups.
fn dash_from_border(dashed: &Option<BorderDash>) -> Option<LineDash> {
    let d = dashed.as_ref()?;
    let dash_mm = d.dash_length.get() as f32;
    let gap_mm = d.break_length.get() as f32;
    if dash_mm <= 0.0 || gap_mm <= 0.0 {
        return None;
    }
    Some(LineDash {
        dash_lengths_mm: vec![dash_mm],
        gap_lengths_mm: vec![gap_mm],
    })
}

pub(super) fn dash_from_style(style: &DashStyle) -> Option<LineDash> {
    match style {
        DashStyle::Dashed {
            dash_length,
            break_length,
            dash_group,
        } => {
            let dash_mm = dash_length.get() as f32;
            let gap_mm = break_length.get() as f32;
            if dash_mm <= 0.0 || gap_mm <= 0.0 {
                return None;
            }
            Some(match dash_group {
                // A group of `n` dashes with a small in-group gap between
                // them, then the (larger) `break_length` gap before the next
                // group starts.
                GroupDashes::Grouped {
                    dashes_in_group,
                    in_group_break_length,
                } => {
                    let n = (*dashes_in_group).max(1) as usize;
                    let in_gap_mm = in_group_break_length.get() as f32;
                    let mut gap_lengths_mm = vec![in_gap_mm; n];
                    *gap_lengths_mm.last_mut().unwrap() = gap_mm;
                    LineDash {
                        dash_lengths_mm: vec![dash_mm; n],
                        gap_lengths_mm,
                    }
                }
                GroupDashes::UnGrouped { .. } => LineDash {
                    dash_lengths_mm: vec![dash_mm],
                    gap_lengths_mm: vec![gap_mm],
                },
            })
        }
        DashStyle::NotDashed { .. } => None,
    }
}

// --- line decorations (start/end/mid/dash point symbols) -------------------

/// Cap on decorations generated for one line, to bound long dashed lines.
const MAX_LINE_DECORATIONS: usize = 5_000;

/// Emit the point-symbol decorations of a line symbol (start/end/mid/dash),
/// each oriented along the local line direction. `dash_points` is parallel to
/// `points` and marks the OMAP dash points the mid and dash symbols key off.
/// Combined lines contribute their private line components' decorations too.
pub(super) fn place_line_decorations(
    symbol: &Symbol,
    ctx: Context,
    points: &[Pos2],
    dash_points: &[bool],
    out: &mut Vec<Primitive>,
) {
    match symbol {
        Symbol::Line(ls) => line_decorations(ls, points, dash_points, ctx, out),
        Symbol::CombinedLine(s) => {
            for c in s.components() {
                match c {
                    PublicOrPrivateSymbol::Private(l) => {
                        line_decorations(l, points, dash_points, ctx, out)
                    }
                    PublicOrPrivateSymbol::Public(id) => {
                        if let Some(sym) = ctx.symbol_set.get(SymbolId::from(*id)) {
                            place_line_decorations(sym.symbol(), ctx, points, dash_points, out);
                        }
                    }
                }
            }
        }
        _ => {}
    }
}

fn line_decorations(
    ls: &LineSymbol,
    points: &[Pos2],
    dash_points: &[bool],
    ctx: Context,
    out: &mut Vec<Primitive>,
) {
    let total = crate::rendering::paper::path_length(points);
    if total <= 0.0 {
        return;
    }

    if let Some(ps) = &ls.start_symbol
        && let Some((pos, angle)) = point_at_arc(points, 0.0)
    {
        push_decoration(ps, pos, angle, ctx, out);
    }
    if let Some(ps) = &ls.end_symbol
        && let Some((pos, angle)) = point_at_arc(points, total)
    {
        push_decoration(ps, pos, angle, ctx, out);
    }

    let closed = points.first() == points.last();

    // Mid symbols run between dash points, not straight through them: each run
    // gets its own spots, so e.g. a course line's legs each keep `end_length`
    // clear around the control circles instead of running through them.
    if let Some(mid) = &ls.mid_symbol {
        let (spacing, margin) = mid_spot_layout(&ls.dash_style);
        let minimum = (if closed {
            mid.minimum_mid_symbol_count_when_closed
        } else {
            mid.minimum_mid_symbol_count
        } as usize)
            .max(usize::from(mid.show_at_least_one_mid_symbol));
        let layout = SpotLayout {
            spacing,
            margin,
            minimum,
        };
        for (first, last) in dash_runs(points.len(), dash_points) {
            place_mid_symbols(mid, &points[first..=last], layout, ctx, out);
        }
    }

    if let Some(dash) = &ls.dash_symbol {
        place_dash_symbols(dash, points, dash_points, ctx, out);
    }
}

/// Where along a run the mid symbols' spots fall: every `spacing` mm, keeping
/// `margin` mm clear at each end, and at least `minimum` of them.
#[derive(Clone, Copy)]
struct SpotLayout {
    spacing: f32,
    margin: f32,
    minimum: usize,
}

/// Mid symbols along one run: one spot per dash (dashed) or every
/// `segment_length` inset by `end_length` (undashed). `mid_symbol_distance`
/// only spaces the `mid_symbols_per_spot` symbols WITHIN a spot — it is NOT
/// the spot spacing.
fn place_mid_symbols(
    mid: &MidSymbol,
    run: &[Pos2],
    layout: SpotLayout,
    ctx: Context,
    out: &mut Vec<Primitive>,
) {
    let total = crate::rendering::paper::path_length(run);
    if total <= 0.0 {
        return;
    }
    let mut spots = even_spots(total, layout.spacing, layout.margin);
    if spots.len() < layout.minimum {
        spots = evenly_spaced(total, layout.minimum);
    }
    let per_spot = i64::from(mid.mid_symbols_per_spot.max(1));
    let within = mid.mid_symbol_distance.get() as f32;
    for spot in spots {
        for i in 0..per_spot {
            let offset = (i as f32 - (per_spot as f32 - 1.0) * 0.5) * within;
            let arc = (spot + offset).clamp(0.0, total);
            if let Some((pos, angle)) = point_at_arc(run, arc) {
                push_decoration(&mid.mid_symbol, pos, angle, ctx, out);
            }
        }
    }
}

/// Emit the dash symbol at every vertex OMAP flagged as a dash point, oriented
/// along the line through it.
///
/// A dash symbol sits on the line's *dash points*, not on the dashes of a dash
/// pattern — the line symbol does not even have to be dashed to carry one. That
/// is what puts a control circle on each control of a course line (symbol 799
/// "Simple Orienteering Course", an undashed line whose dash symbol is the
/// circle); keying it off the dash pattern instead draws nothing at all there.
fn place_dash_symbols(
    dash: &DashSymbol,
    points: &[Pos2],
    dash_points: &[bool],
    ctx: Context,
    out: &mut Vec<Primitive>,
) {
    if points.len() < 2 || dash_points.len() != points.len() {
        return;
    }
    let last = points.len() - 1;
    let closed = points[0] == points[last];
    let mut placed = 0usize;
    for i in 0..points.len() {
        // A closed line's first and last vertices are one and the same: the
        // flag may sit on either, and the symbol is drawn once.
        let flagged = match i {
            _ if !closed => dash_points[i],
            0 => dash_points[0] || dash_points[last],
            _ if i == last => continue,
            _ => dash_points[i],
        };
        // Only an open line has ends to suppress the symbol at.
        if !flagged || (!closed && (i == 0 || i == last) && dash.suppress_dash_symbol_at_ends) {
            continue;
        }
        push_decoration(
            &dash.dash_symbol,
            points[i],
            vertex_direction(points, i, closed),
            ctx,
            out,
        );
        placed += 1;
        if placed >= MAX_LINE_DECORATIONS {
            return;
        }
    }
}

/// Direction (radians, y-up) of the line *through* vertex `i` — the chord
/// between its neighbours, so a dash symbol straddling a corner is not aligned
/// to just one of the two segments.
fn vertex_direction(points: &[Pos2], i: usize, closed: bool) -> f32 {
    let n = points.len();
    let before = match i {
        0 if closed => Some(points[n - 2]),
        0 => None,
        _ => Some(points[i - 1]),
    };
    let after = match i {
        _ if i + 1 < n => Some(points[i + 1]),
        _ if closed => Some(points[1]),
        _ => None,
    };
    let d = match (before, after) {
        (Some(a), Some(b)) => b - a,
        (Some(a), None) => points[i] - a,
        (None, Some(b)) => b - points[i],
        (None, None) => Vec2::X,
    };
    if d.length_sq() > 0.0 {
        d.y.atan2(d.x)
    } else {
        0.0
    }
}

/// Spot spacing and end margin (mm) for mid symbols, from the dash style: one
/// spot per dash centre for dashed lines, or every `segment_length` inset by
/// `end_length` for undashed lines.
fn mid_spot_layout(dash: &DashStyle) -> (f32, f32) {
    match dash {
        DashStyle::Dashed {
            dash_length,
            break_length,
            ..
        } => (
            dash_length.get() as f32 + break_length.get() as f32,
            dash_length.get() as f32 * 0.5,
        ),
        DashStyle::NotDashed {
            segment_length,
            end_length,
        } => (segment_length.get() as f32, end_length.get() as f32),
    }
}

/// Arc positions every `spacing`, keeping `margin` clear at each end. Capped.
fn even_spots(total: f32, spacing: f32, margin: f32) -> Vec<f32> {
    let mut spots = Vec::new();
    if spacing <= 0.05 {
        return spots;
    }
    let mut arc = margin;
    let limit = total - margin;
    while arc <= limit + 0.01 && spots.len() < MAX_LINE_DECORATIONS {
        spots.push(arc);
        arc += spacing;
    }
    spots
}

/// `count` positions spread evenly along the line (centres of equal segments).
fn evenly_spaced(total: f32, count: usize) -> Vec<f32> {
    (0..count)
        .map(|k| total * (k as f32 + 0.5) / count as f32)
        .collect()
}

/// Build a point symbol's marks, orient them along `angle` (line tangent) and
/// emit them as a point primitive at `pos`.
fn push_decoration(
    ps: &PointSymbol,
    pos: Pos2,
    angle: f32,
    ctx: Context,
    out: &mut Vec<Primitive>,
) {
    let Context { colors, geo, .. } = ctx;
    let mut marks: Vec<(LocalMark, i32)> = Vec::new();
    build_point_marks(ps, colors, Vec2::ZERO, 0, &mut marks);
    if marks.is_empty() {
        return;
    }
    let (sin, cos) = angle.sin_cos();
    rotate_marks(&mut marks, cos, sin); // align local +x with the tangent

    // Each mark is its own primitive at its own colour priority, so decoration
    // parts layer correctly against other objects instead of all sitting at one
    // shared z (mirrors the point-object handling in `render::map`).
    for (mark, z) in &marks {
        let (lo, hi) = mark_extent(mark);
        let (min, max) = geo.mercator_bounds(pos + lo, pos + hi);
        out.push(Primitive {
            kind: Kind::PointMark(resolve_mark(mark, pos, geo)),
            z: *z,
            min,
            max,
        });
    }
}

/// Position and tangent angle (radians, y-up) at arc length `target` along a
/// polyline. Clamps to the ends.
fn point_at_arc(points: &[Pos2], target: f32) -> Option<(Pos2, f32)> {
    if points.len() < 2 {
        return None;
    }
    let mut acc = 0.0;
    for w in points.windows(2) {
        let seg = w[1] - w[0];
        let a = geo::Point::new(w[0].x, w[0].y);
        let b = geo::Point::new(w[1].x, w[1].y);
        let len = Euclidean.distance(a, b);
        if len <= 0.0 {
            continue;
        }
        if acc + len >= target {
            let t = ((target - acc) / len).clamp(0.0, 1.0);
            let at = Euclidean.point_at_ratio_between(a, b, t);
            return Some((Pos2::new(at.x(), at.y()), seg.y.atan2(seg.x)));
        }
        acc += len;
    }
    let n = points.len();
    let seg = points[n - 1] - points[n - 2];
    Some((points[n - 1], seg.y.atan2(seg.x)))
}

#[cfg(test)]
mod tests {
    use omap::NonNegativeF64;

    use super::*;

    #[test]
    fn ungrouped_dash_is_a_single_dash_gap_step() {
        let style = DashStyle::Dashed {
            dash_length: NonNegativeF64::clamped_from(1.0),
            break_length: NonNegativeF64::clamped_from(0.8),
            dash_group: GroupDashes::UnGrouped {
                half_outer_dashes: false,
            },
        };
        let dash = dash_from_style(&style).expect("dashed style should produce a pattern");
        assert_eq!(dash.dash_lengths_mm, vec![1.0]);
        assert_eq!(dash.gap_lengths_mm, vec![0.8]);
    }

    #[test]
    fn grouped_dash_repeats_the_dash_length_with_a_small_in_group_gap_and_a_larger_group_gap() {
        // Matches the real symbol in sample-maps/sample.omap: dash_length=1000,
        // break_length=800, dashes_in_group=2, in_group_break_length=250 (0.1mm
        // file units).
        let style = DashStyle::Dashed {
            dash_length: NonNegativeF64::clamped_from(1.0),
            break_length: NonNegativeF64::clamped_from(0.8),
            dash_group: GroupDashes::Grouped {
                dashes_in_group: 2,
                in_group_break_length: NonNegativeF64::clamped_from(0.25),
            },
        };
        let dash = dash_from_style(&style).expect("dashed style should produce a pattern");
        // Two dashes per group: a small in-group gap after the first, then the
        // full break_length after the second (before the next group starts).
        assert_eq!(dash.dash_lengths_mm, vec![1.0, 1.0]);
        assert_eq!(dash.gap_lengths_mm, vec![0.25, 0.8]);
    }

    #[test]
    fn zero_dash_or_break_length_disables_dashing() {
        let style = DashStyle::Dashed {
            dash_length: NonNegativeF64::clamped_from(0.0),
            break_length: NonNegativeF64::clamped_from(0.8),
            dash_group: GroupDashes::UnGrouped {
                half_outer_dashes: false,
            },
        };
        assert!(dash_from_style(&style).is_none());
    }

    #[test]
    fn border_dash_builds_a_single_dash_gap_step() {
        let dashed = Some(BorderDash {
            dash_length: NonNegativeF64::clamped_from(1.2),
            break_length: NonNegativeF64::clamped_from(0.4),
        });
        let dash = dash_from_border(&dashed).expect("dashed border should produce a pattern");
        assert_eq!(dash.dash_lengths_mm, vec![1.2]);
        assert_eq!(dash.gap_lengths_mm, vec![0.4]);
    }

    #[test]
    fn undashed_or_degenerate_border_has_no_pattern() {
        assert!(dash_from_border(&None).is_none());
        assert!(
            dash_from_border(&Some(BorderDash {
                dash_length: NonNegativeF64::clamped_from(0.0),
                break_length: NonNegativeF64::clamped_from(0.4),
            }))
            .is_none()
        );
    }
}
