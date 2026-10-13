use eframe::egui::{Color32, Pos2, Vec2};
use omap::{
    colors::ColorSet,
    objects::{AreaObject, LineObject},
    symbols::{Element, PointSymbol},
};

use super::color::resolve_color;
use super::primitives::{Path, PointMark};
use crate::rendering::geo::GeoTransform;
use crate::rendering::paper::PaperPath;

/// One drawable piece of a point symbol *before* it is anchored to a real
/// position: geometry is a local offset (paper mm) from the point's centre.
/// [`resolve_mark`] combines an offset with a centre and projects it through
/// [`GeoTransform`] into the paint-ready [`PointMark`].
pub(super) enum LocalMark {
    Disc {
        center: Vec2,
        radius_mm: f32,
        color: Color32,
    },
    Ring {
        center: Vec2,
        radius_mm: f32,
        width_mm: f32,
        color: Color32,
    },
    Stroke {
        path: LocalPath,
        width_mm: f32,
        color: Color32,
    },
    Fill {
        path: LocalPath,
        color: Color32,
    },
}

/// A point-symbol element's local geometry — paper-mm offset from the point's
/// own centre, kept as anchors plus sparse cubic handles (mirrors
/// [`crate::rendering::paper::PaperPath`], just in offset space rather than absolute
/// paper coordinates). Reconstructed from the file's own Bézier control
/// points where the element's object still carries them, the same way
/// `render::map` handles top-level line/area objects — so a curved element
/// (an arc, a knoll's hook) stays a true curve all the way to paint time
/// instead of being flattened once at load into `omap`'s fixed tolerance.
pub(super) struct LocalPath {
    anchors: Vec<Vec2>,
    curves: Vec<(u32, [Vec2; 2])>,
}

/// Translate a paper-space path into local offset space, applying `offset` to
/// every anchor *and* handle — valid because an affine map (translation here,
/// rotation later in [`rotate_marks`]) applied to a Bézier curve's control
/// points transforms the curve exactly, with no need to re-flatten or refit.
fn paper_path_to_local(p: &PaperPath, offset: Vec2) -> LocalPath {
    LocalPath {
        anchors: p.anchors.iter().map(|a| offset + a.to_vec2()).collect(),
        curves: p
            .curves
            .iter()
            .map(|&(i, [h1, h2])| (i, [offset + h1.to_vec2(), offset + h2.to_vec2()]))
            .collect(),
    }
}

/// A line element's local path, keeping the object's own curves.
fn local_line_path(object: &LineObject, offset: Vec2) -> LocalPath {
    paper_path_to_local(&PaperPath::from_bezier(object.geometry()), offset)
}

/// An area element's exterior ring as a local path — same curve handling as
/// [`local_line_path`]. Interior rings are not needed: point-symbol area
/// elements (a boulder's triangle, ...) are simple single-ring shapes.
fn local_area_exterior_path(object: &AreaObject, offset: Vec2) -> LocalPath {
    paper_path_to_local(
        &PaperPath::from_bezier(object.geometry().exterior()),
        offset,
    )
}

/// Collect the drawable marks of a point symbol — its base circle plus every
/// element sub-shape — each with its colour priority. `offset` accumulates the
/// local (paper-mm) position of nested elements relative to the point centre.
pub(super) fn build_point_marks(
    ps: &PointSymbol,
    colors: &ColorSet,
    offset: Vec2,
    depth: u8,
    out: &mut Vec<(LocalMark, i32)>,
) {
    if depth > 6 {
        return;
    }
    // Base circle: inner disc, then outer ring.
    if let Some((color, z)) = resolve_color(&ps.inner_color, colors) {
        let radius_mm = ps.inner_radius.get() as f32;
        if radius_mm > 0.0 {
            out.push((
                LocalMark::Disc {
                    center: offset,
                    radius_mm,
                    color,
                },
                z,
            ));
        }
    }
    if let Some((color, z)) = resolve_color(&ps.outer_color, colors) {
        let width_mm = ps.outer_width.get() as f32;
        if width_mm > 0.0 {
            out.push((
                LocalMark::Ring {
                    center: offset,
                    radius_mm: ps.inner_radius.get() as f32,
                    width_mm,
                    color,
                },
                z,
            ));
        }
    }
    // Element sub-shapes, positioned by their object geometry.
    for element in &ps.elements {
        match element {
            Element::Point { symbol, object } => {
                let c = object.geometry().0;
                build_point_marks(
                    symbol,
                    colors,
                    offset + Vec2::new(c.x as f32, c.y as f32),
                    depth + 1,
                    out,
                );
            }
            Element::Line { symbol, object } => {
                if let Some((color, z)) = resolve_color(&symbol.color, colors) {
                    let path = local_line_path(object, offset);
                    if path.anchors.len() >= 2 {
                        out.push((
                            LocalMark::Stroke {
                                path,
                                width_mm: symbol.line_width.get() as f32,
                                color,
                            },
                            z,
                        ));
                    }
                }
            }
            Element::Area { symbol, object } => {
                if let Some((color, z)) = resolve_color(&symbol.color, colors) {
                    let path = local_area_exterior_path(object, offset);
                    if path.anchors.len() >= 3 {
                        out.push((LocalMark::Fill { path, color }, z));
                    }
                }
            }
        }
    }
}

/// Local-space bounding offsets of a point mark, for culling.
pub(super) fn mark_extent(m: &LocalMark) -> (Vec2, Vec2) {
    match m {
        LocalMark::Disc {
            center, radius_mm, ..
        } => (
            *center - Vec2::splat(*radius_mm),
            *center + Vec2::splat(*radius_mm),
        ),
        LocalMark::Ring {
            center,
            radius_mm,
            width_mm,
            ..
        } => {
            let r = Vec2::splat(radius_mm + width_mm * 0.5);
            (*center - r, *center + r)
        }
        LocalMark::Stroke { path, .. } | LocalMark::Fill { path, .. } => {
            // The control polygon (anchors + handles) always contains the
            // curve, same bound `PaperPath::control_bounds` uses.
            let mut lo = Vec2::splat(f32::INFINITY);
            let mut hi = Vec2::splat(f32::NEG_INFINITY);
            for p in path
                .anchors
                .iter()
                .chain(path.curves.iter().flat_map(|(_, h)| h.iter()))
            {
                lo = Vec2::new(lo.x.min(p.x), lo.y.min(p.y));
                hi = Vec2::new(hi.x.max(p.x), hi.y.max(p.y));
            }
            if lo.x.is_finite() {
                (lo, hi)
            } else {
                (Vec2::ZERO, Vec2::ZERO)
            }
        }
    }
}

/// Rotate every mark's local offsets in place by the rotation whose cosine and
/// sine are `cos`/`sin` (standard CCW: `x' = x·cos − y·sin`, `y' = x·sin + y·cos`).
/// A curve's handles rotate the same as its anchors — affine maps applied to
/// Bézier control points transform the curve exactly.
pub(super) fn rotate_marks(marks: &mut [(LocalMark, i32)], cos: f32, sin: f32) {
    let rot = |v: Vec2| Vec2::new(v.x * cos - v.y * sin, v.x * sin + v.y * cos);
    for (m, _) in marks {
        match m {
            LocalMark::Disc { center, .. } | LocalMark::Ring { center, .. } => {
                *center = rot(*center)
            }
            LocalMark::Stroke { path, .. } | LocalMark::Fill { path, .. } => {
                for p in path.anchors.iter_mut() {
                    *p = rot(*p);
                }
                for (_, h) in path.curves.iter_mut() {
                    h[0] = rot(h[0]);
                    h[1] = rot(h[1]);
                }
            }
        }
    }
}

/// Anchor a local mark's offsets at `at` (paper mm) and project the result
/// through `geo` into the paint-ready, absolute [`PointMark`]. The expensive
/// paper-mm → WGS84 step happens here, once at load time, rather than every
/// frame.
pub(super) fn resolve_mark(m: &LocalMark, at: Pos2, geo: &GeoTransform) -> PointMark {
    let abs = |v: Vec2| geo.to_mercator(at + v);
    match m {
        LocalMark::Disc {
            center,
            radius_mm,
            color,
        } => PointMark::Disc {
            center: abs(*center),
            radius_mm: *radius_mm,
            color: *color,
        },
        LocalMark::Ring {
            center,
            radius_mm,
            width_mm,
            color,
        } => PointMark::Ring {
            center: abs(*center),
            radius_mm: *radius_mm,
            width_mm: *width_mm,
            color: *color,
        },
        LocalMark::Stroke {
            path,
            width_mm,
            color,
        } => PointMark::Stroke {
            path: resolve_path(path, at, geo),
            width_mm: *width_mm,
            color: *color,
        },
        LocalMark::Fill { path, color } => PointMark::Fill {
            path: resolve_path(path, at, geo),
            color: *color,
        },
    }
}

/// Anchor a local element path at `at` and project it into a paint-ready
/// [`Path`], keeping its curve handles — [`PointMark::Stroke`]/`Fill` flatten
/// it adaptively at paint time, the same way a line object's own stroke does.
/// Dash points are irrelevant to an element shape, so every vertex is `false`.
fn resolve_path(path: &LocalPath, at: Pos2, geo: &GeoTransform) -> Path {
    let abs = |v: Vec2| geo.to_mercator(at + v);
    Path {
        anchors: path.anchors.iter().map(|p| abs(*p)).collect(),
        curves: path
            .curves
            .iter()
            .map(|&(i, [h1, h2])| (i, [abs(h1), abs(h2)]))
            .collect(),
        dash_points: vec![false; path.anchors.len()],
    }
}
