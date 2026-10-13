use eframe::egui::{Color32, Pos2, Vec2};
use geo::{BooleanOps, Contains, LineString, LinesIter, MultiLineString, Polygon};
use omap::{
    colors::{ColorSet, SymbolColor},
    symbols::{
        AreaOrLineSymbol, ClippingOption, DashStyle, FillPattern, PublicOrPrivateSymbol, Symbol,
        SymbolId,
    },
};

use super::color::resolve_color;
use super::context::Context;
use super::line::dash_from_style;
use super::point::{LocalMark, build_point_marks, mark_extent, resolve_mark, rotate_marks};
use super::primitives::{AreaPattern, LineDash, Z_UNKNOWN};

/// A stroke applied along an area's rings — the border/outline contributed by a
/// line component of a combined area symbol. Emitted as its own `Kind::Line`
/// primitive at the border colour's own priority, so it layers correctly.
pub(super) struct OutlineStroke {
    pub color: Color32,
    pub width_mm: f32,
    pub dash: Option<LineDash>,
    pub z: i32,
}

/// Resolve an area symbol into `(fill colour, z-order, outline strokes)`.
///
/// A plain area symbol contributes only its fill. A combined area symbol (e.g.
/// a building = filled polygon + black border) contributes the first area
/// component's fill and a stroke for each line component.
pub(super) fn area_style(
    symbol: &Symbol,
    ctx: Context,
) -> (Option<Color32>, i32, Vec<OutlineStroke>) {
    let Context {
        symbol_set, colors, ..
    } = ctx;
    match symbol {
        Symbol::Area(a) => match resolve_color(&a.color, colors) {
            Some((c, p)) => (Some(c), p, Vec::new()),
            None => (None, Z_UNKNOWN, Vec::new()),
        },
        Symbol::CombinedArea(s) => {
            let mut fill = None;
            let mut z = Z_UNKNOWN;
            let mut outlines = Vec::new();
            for c in s.components() {
                match c {
                    PublicOrPrivateSymbol::Private(AreaOrLineSymbol::Area(a)) => {
                        if fill.is_none()
                            && let Some((col, prio)) = resolve_color(&a.color, colors)
                        {
                            fill = Some(col);
                            z = prio;
                        }
                    }
                    PublicOrPrivateSymbol::Private(AreaOrLineSymbol::Line(l)) => {
                        if let Some(stroke) =
                            line_symbol_stroke(&l.color, l.line_width.get(), &l.dash_style, colors)
                        {
                            outlines.push(stroke);
                        }
                    }
                    PublicOrPrivateSymbol::Public(id) => {
                        if let Some(sym) = symbol_set.get(SymbolId::from(*id)) {
                            let (f, zz, mut o) = area_style(sym.symbol(), ctx);
                            if fill.is_none() && f.is_some() {
                                fill = f;
                                z = zz;
                            }
                            outlines.append(&mut o);
                            if let Some(stroke) = symbol_line_stroke(sym.symbol(), colors) {
                                outlines.push(stroke);
                            }
                        }
                    }
                }
            }
            (fill, z, outlines)
        }
        _ => (None, Z_UNKNOWN, Vec::new()),
    }
}

fn line_symbol_stroke(
    color: &SymbolColor,
    line_width: f64,
    dash_style: &DashStyle,
    colors: &ColorSet,
) -> Option<OutlineStroke> {
    let (color, z) = resolve_color(color, colors)?;
    Some(OutlineStroke {
        color,
        width_mm: line_width as f32,
        dash: dash_from_style(dash_style),
        z,
    })
}

/// Outline stroke for a (public) line symbol used as a combined-area component.
fn symbol_line_stroke(symbol: &Symbol, colors: &ColorSet) -> Option<OutlineStroke> {
    match symbol {
        Symbol::Line(l) => line_symbol_stroke(&l.color, l.line_width.get(), &l.dash_style, colors),
        _ => None,
    }
}

// --- area fill patterns ----------------------------------------------------

/// Cap on generated hatch lines / scatter rows to keep dense patterns over big
/// areas from blowing up load time.
const MAX_PATTERN_LINES: usize = 5_000;
/// Cap on generated scatter points per pattern.
const MAX_PATTERN_POINTS: usize = 150_000;

/// A polygon's rings: the exterior, and the holes cut out of it.
#[derive(Clone, Copy)]
pub(super) struct Rings<'a> {
    pub exterior: &'a [Pos2],
    pub interiors: &'a [Vec<Pos2>],
}

impl Rings<'_> {
    fn polygon(self) -> Polygon<f32> {
        let ring = |points: &[Pos2]| {
            LineString(
                points
                    .iter()
                    .map(|p| geo::Coord { x: p.x, y: p.y })
                    .collect(),
            )
        };
        Polygon::new(
            ring(self.exterior),
            self.interiors.iter().map(|r| ring(r)).collect(),
        )
    }
}

/// How a point pattern is laid out over a polygon: the rotation of its rows,
/// the spacing between rows and between points along a row, and the offset of
/// each.
#[derive(Clone, Copy)]
struct Grid {
    angle: f32,
    row_spacing: f32,
    along_spacing: f32,
    row_offset: f32,
    along_offset: f32,
}

/// How a symbol straddling a polygon's boundary is treated: `radius` is the
/// symbol's extent from its centre, and `option` how that is held against the
/// boundary (see [`ClippingOption`]).
#[derive(Clone, Copy)]
struct Clip {
    radius: f32,
    option: ClippingOption,
}

/// Build the (geometry-resolved) fill patterns for an area symbol. `extra_angle`
/// is the object's own pattern rotation, added to each pattern's symbol angle.
pub(super) fn build_area_patterns(
    symbol: &Symbol,
    ctx: Context,
    rings: Rings,
    extra_angle: f32,
) -> Vec<(AreaPattern, i32)> {
    let mut out = Vec::new();
    collect_patterns(symbol, ctx, rings, extra_angle, 0, &mut out);
    out
}

fn collect_patterns(
    symbol: &Symbol,
    ctx: Context,
    rings: Rings,
    extra_angle: f32,
    depth: u8,
    out: &mut Vec<(AreaPattern, i32)>,
) {
    if depth > 8 {
        return;
    }
    match symbol {
        Symbol::Area(a) => {
            for p in &a.patterns {
                resolve_pattern(p, ctx, rings, extra_angle, out);
            }
        }
        Symbol::CombinedArea(s) => {
            for c in s.components() {
                match c {
                    PublicOrPrivateSymbol::Private(AreaOrLineSymbol::Area(a)) => {
                        for p in &a.patterns {
                            resolve_pattern(p, ctx, rings, extra_angle, out);
                        }
                    }
                    PublicOrPrivateSymbol::Private(AreaOrLineSymbol::Line(_)) => {}
                    PublicOrPrivateSymbol::Public(id) => {
                        if let Some(sym) = ctx.symbol_set.get(SymbolId::from(*id)) {
                            collect_patterns(sym.symbol(), ctx, rings, extra_angle, depth + 1, out);
                        }
                    }
                }
            }
        }
        _ => {}
    }
}

/// Resolve one fill pattern into drawable pieces, pushed into `out` with their
/// colour priorities. A point pattern's symbol may have several marks (e.g. a
/// boulder's dot + triangle); each becomes its own `AreaPattern::Dots` at the
/// same scattered positions, so it layers at its own priority instead of
/// collapsing to one representative disc.
fn resolve_pattern(
    p: &FillPattern,
    ctx: Context,
    rings: Rings,
    extra_angle: f32,
    out: &mut Vec<(AreaPattern, i32)>,
) {
    let Context { colors, geo, .. } = ctx;
    match p {
        FillPattern::LinePattern {
            angle,
            line_spacing,
            line_offset,
            line_color,
            line_width,
            ..
        } => {
            let Some((color, z)) = resolve_color(line_color, colors) else {
                return;
            };
            let spacing = line_spacing.get() as f32;
            if spacing <= 0.01 {
                return;
            }
            let segments = hatch_segments(
                rings,
                *angle as f32 + extra_angle,
                spacing,
                line_offset.get() as f32,
            );
            if !segments.is_empty() {
                out.push((
                    AreaPattern::Hatch {
                        segments: segments
                            .iter()
                            .map(|[a, b]| [geo.to_mercator(*a), geo.to_mercator(*b)])
                            .collect(),
                        color,
                        width_mm: line_width.get() as f32,
                        spacing_mm: spacing,
                    },
                    z,
                ));
            }
        }
        FillPattern::PointPattern {
            clip_options,
            angle,
            line_spacing,
            line_offset,
            offset_along_line,
            point_distance,
            point,
            ..
        } => {
            let mut marks: Vec<(LocalMark, i32)> = Vec::new();
            build_point_marks(point, colors, Vec2::ZERO, 0, &mut marks);
            if marks.is_empty() {
                return;
            }

            let pattern_angle = *angle as f32 + extra_angle;
            // Rotatable pattern symbols follow the pattern's own rotation, same
            // as a rotatable point object follows its object rotation.
            if point.is_rotatable {
                let (sin, cos) = pattern_angle.sin_cos();
                rotate_marks(&mut marks, cos, sin);
            }

            let row = line_spacing.get() as f32;
            let along = point_distance.get() as f32;
            if row <= 0.01 || along <= 0.01 {
                return;
            }

            // Boundary-clip radius: the symbol's overall local extent from its
            // centre, standing in for the single-disc radius the clip test used
            // to use.
            let radius = marks.iter().fold(0.0_f32, |r, (m, _)| {
                let (lo, hi) = mark_extent(m);
                r.max(lo.length()).max(hi.length())
            });

            let grid = Grid {
                angle: pattern_angle,
                row_spacing: row,
                along_spacing: along,
                row_offset: line_offset.get() as f32,
                along_offset: offset_along_line.get() as f32,
            };
            let clip = Clip {
                radius,
                option: *clip_options,
            };
            let positions = scatter_points(rings, grid, clip);
            if positions.is_empty() {
                return;
            }

            // Each mark is resolved (offset + anchor projected through `geo`)
            // at every scattered position, once here at load time — painting
            // then only has to project the already-geographic marks.
            let pitch_mm = row.min(along);
            for (mark, z) in &marks {
                out.push((
                    AreaPattern::Dots {
                        marks: positions
                            .iter()
                            .map(|p| resolve_mark(mark, *p, geo))
                            .collect(),
                        pitch_mm,
                    },
                    *z,
                ));
            }
        }
    }
}

/// Rotated basis for a pattern: `u` runs along the pattern lines, `n` is the
/// perpendicular along which rows/lines are spaced.
fn pattern_basis(angle: f32) -> (Vec2, Vec2) {
    let (s, c) = angle.sin_cos();
    (Vec2::new(c, s), Vec2::new(-s, c))
}

/// Extent of the polygon's vertices projected onto a direction.
fn projected_extent(exterior: &[Pos2], dir: Vec2) -> (f32, f32) {
    let mut lo = f32::INFINITY;
    let mut hi = f32::NEG_INFINITY;
    for p in exterior {
        let d = p.to_vec2().dot(dir);
        lo = lo.min(d);
        hi = hi.max(d);
    }
    (lo, hi)
}

/// Parallel hatch lines clipped by geo to the exterior and holes.
fn hatch_segments(rings: Rings, angle: f32, spacing: f32, offset: f32) -> Vec<[Pos2; 2]> {
    let (u, n) = pattern_basis(angle);
    let (n_lo, n_hi) = projected_extent(rings.exterior, n);
    if !n_lo.is_finite() || (n_hi - n_lo) / spacing > MAX_PATTERN_LINES as f32 {
        return Vec::new();
    }
    let (u_lo, u_hi) = projected_extent(rings.exterior, u);
    let margin = spacing.max(1.0);
    let k0 = ((n_lo - offset) / spacing).ceil() as i64;
    let k1 = ((n_hi - offset) / spacing).floor() as i64;
    let lines = MultiLineString(
        (k0..=k1)
            .map(|k| {
                let d = offset + k as f32 * spacing;
                let a = u * (u_lo - margin) + n * d;
                let b = u * (u_hi + margin) + n * d;
                LineString::from(vec![(a.x, a.y), (b.x, b.y)])
            })
            .collect(),
    );
    rings
        .polygon()
        .clip(&lines, false)
        .lines_iter()
        .map(|line| {
            [
                Pos2::new(line.start.x, line.start.y),
                Pos2::new(line.end.x, line.end.y),
            ]
        })
        .collect()
}

/// A grid of point positions clipped to the polygon. `clip` decides how a
/// symbol straddling the boundary is treated.
fn scatter_points(rings: Rings, grid: Grid, clip: Clip) -> Vec<Pos2> {
    let polygon = rings.polygon();
    let Rings { exterior, .. } = rings;
    let Grid {
        angle,
        row_spacing,
        along_spacing,
        row_offset,
        along_offset,
    } = grid;
    let (u, n) = pattern_basis(angle);
    let (n_lo, n_hi) = projected_extent(exterior, n);
    let (u_lo, u_hi) = projected_extent(exterior, u);
    if !n_lo.is_finite() || !u_lo.is_finite() {
        return Vec::new();
    }
    let rows = (n_hi - n_lo) / row_spacing;
    let cols = (u_hi - u_lo) / along_spacing;
    if rows * cols > MAX_PATTERN_POINTS as f32 {
        return Vec::new();
    }

    let i0 = ((n_lo - row_offset) / row_spacing).ceil() as i64;
    let i1 = ((n_hi - row_offset) / row_spacing).floor() as i64;
    let mut positions = Vec::new();
    for i in i0..=i1 {
        let d = row_offset + i as f32 * row_spacing;
        // Stagger alternate rows by the along-offset (brick-like layout).
        let phase = if i.rem_euclid(2) == 1 {
            along_offset
        } else {
            0.0
        };
        let j0 = ((u_lo - phase) / along_spacing).ceil() as i64;
        let j1 = ((u_hi - phase) / along_spacing).floor() as i64;
        for j in j0..=j1 {
            let s = phase + j as f32 * along_spacing;
            let p = (u * s + n * d).to_pos2();
            if symbol_passes_clip(p, &polygon, clip) {
                positions.push(p);
            }
        }
    }
    positions
}

/// Whether a pattern symbol at `center` (extent `radius`) should be kept, given
/// the area's clip option. We cannot clip the individual symbol shape, so the
/// modes are approximated by testing the centre and four cardinal edge points:
/// `CompletelyInside` needs all inside, `PartiallyInside` needs any inside, and
/// the default (`ClipElementsAtBoundary`) / `CenterInside` test the centre.
fn symbol_passes_clip(center: Pos2, polygon: &Polygon<f32>, clip: Clip) -> bool {
    let Clip { radius, option } = clip;
    let inside = |p: Pos2| polygon.contains(&geo::Coord { x: p.x, y: p.y });
    match option {
        ClippingOption::NoClippingIfCompletelyInside => {
            let r = radius.max(0.01);
            inside(center)
                && inside(center + Vec2::new(r, 0.0))
                && inside(center + Vec2::new(-r, 0.0))
                && inside(center + Vec2::new(0.0, r))
                && inside(center + Vec2::new(0.0, -r))
        }
        ClippingOption::NoClippingIfPartiallyInside => {
            let r = radius.max(0.01);
            inside(center)
                || inside(center + Vec2::new(r, 0.0))
                || inside(center + Vec2::new(-r, 0.0))
                || inside(center + Vec2::new(0.0, r))
                || inside(center + Vec2::new(0.0, -r))
        }
        // ClipElementsAtBoundary (default) and NoClippingIfCenterInside: since
        // we can't clip the shape itself, both fall back to a centre test.
        _ => inside(center),
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn fill_patterns_respect_holes() {
        // 10×10 square with a 4×4 hole in the middle.
        let ext = vec![
            Pos2::new(0.0, 0.0),
            Pos2::new(10.0, 0.0),
            Pos2::new(10.0, 10.0),
            Pos2::new(0.0, 10.0),
        ];
        let interiors = vec![vec![
            Pos2::new(3.0, 3.0),
            Pos2::new(7.0, 3.0),
            Pos2::new(7.0, 7.0),
            Pos2::new(3.0, 7.0),
        ]];

        let rings = Rings {
            exterior: &ext,
            interiors: &interiors,
        };
        assert!(rings.polygon().contains(&geo::Point::new(1.0, 1.0)));
        assert!(!rings.polygon().contains(&geo::Point::new(5.0, 5.0))); // in the hole
        assert!(!rings.polygon().contains(&geo::Point::new(-1.0, 5.0))); // outside

        // Horizontal hatch: the line crossing the hole splits into two spans.
        let segs = hatch_segments(rings, 0.0, 1.0, 0.0);
        let at5 = segs
            .iter()
            .filter(|s| (s[0].y - 5.0).abs() < 1e-3 && (s[1].y - 5.0).abs() < 1e-3)
            .count();
        assert_eq!(
            at5, 2,
            "the hole must split the y=5 hatch line into two spans"
        );

        // Scatter: no point lands strictly inside the hole.
        let grid = Grid {
            angle: 0.0,
            row_spacing: 1.0,
            along_spacing: 1.0,
            row_offset: 0.0,
            along_offset: 0.0,
        };
        let clip = Clip {
            radius: 0.1,
            option: ClippingOption::NoClippingIfCenterInside,
        };
        let pts = scatter_points(rings, grid, clip);
        assert!(!pts.is_empty());
        assert!(
            pts.iter()
                .all(|p| !(p.x > 3.0 && p.x < 7.0 && p.y > 3.0 && p.y < 7.0)),
            "no scatter point should fall inside the hole"
        );
    }
}
