//! Converts OMAP objects into primitives with resolved geometry and symbol styles.

use eframe::egui::Pos2;
use omap::{
    Omap,
    objects::{AreaObject, MapObject},
    symbols::{Symbol, SymbolRef},
};

use super::context::Context;
use super::primitives::{Kind, Primitive, Z_UNKNOWN};
use super::{area, geometry, line, point};
use crate::rendering::geo::GeoTransform;
use crate::rendering::paper::{self, PaperPath};

/// Resolve a map's objects into primitives in colour-table drawing order.
pub fn draw_objects(map: &Omap, geo: &GeoTransform) -> Vec<Primitive> {
    let ctx = Context {
        symbol_set: &map.symbols,
        colors: &map.colors,
        geo,
    };
    let mut primitives = Vec::new();
    for object in map.iter_all_objects() {
        append_primitives(object, ctx, &mut primitives);
    }
    primitives.sort_by_key(|p| std::cmp::Reverse(p.z));
    primitives
}

/// An area object's rings, in both forms the renderer needs.
struct AreaRings {
    /// Exterior ring, flattened at load time for the fill mesh, the fill
    /// patterns and the object's bounds.
    exterior: Vec<Pos2>,
    /// Interior rings, likewise.
    interiors: Vec<Vec<Pos2>>,
    /// The same rings as curve paths, aligned with `exterior ++ interiors`, for
    /// the outline strokes — which flatten in screen space instead.
    paths: Vec<PaperPath>,
}

/// [`paper::area_paths`] split into the two forms above. Resolving the rings
/// themselves is `paper`'s job, because the navigation layer needs exactly the
/// same rings and must not read them differently.
fn area_rings(area_obj: &AreaObject) -> AreaRings {
    let paths = paper::area_paths(area_obj);
    let flattened: Vec<Vec<Pos2>> = paths
        .iter()
        .map(|p| p.flatten(paper::BEZIER_FLATTEN_ERROR_MM).0)
        .collect();
    let mut flattened = flattened.into_iter();
    AreaRings {
        exterior: flattened.next().unwrap_or_default(),
        interiors: flattened.collect(),
        paths,
    }
}

/// Append every drawable primitive an object contributes, and return its
/// map-space bounds — an empty box for an object with no geometry. An area may
/// yield a fill plus one border primitive per outline stroke, each at its own
/// colour priority.
fn append_primitives(object: &MapObject, ctx: Context, out: &mut Vec<Primitive>) -> (Pos2, Pos2) {
    let Context {
        symbol_set,
        colors,
        geo,
    } = ctx;
    let symbol = object
        .symbol()
        .and_then(|id| symbol_set.get(id))
        .map(SymbolRef::symbol);
    match object {
        MapObject::Area(area_obj) => {
            let (fill, fill_z, outlines) =
                symbol
                    .map(|s| area::area_style(s, ctx))
                    .unwrap_or((None, Z_UNKNOWN, Vec::new()));

            let AreaRings {
                exterior,
                interiors,
                paths,
            } = area_rings(area_obj);
            let (min, max) = paper::bounds_of(exterior.iter().copied());
            if exterior.len() < 2 {
                return (min, max);
            }
            let (geo_min, geo_max) = geo.mercator_bounds(min, max);

            let mut emitted = false;

            // Border strokes become their own line primitives, one per ring, so
            // each draws at its border colour's priority.
            for outline in &outlines {
                for ring in &paths {
                    if let Some(prim) = outline_primitive(ring, outline, geo) {
                        out.push(prim);
                        emitted = true;
                    }
                }
            }

            // Each pattern is its own primitive at its own colour's priority, so
            // a purple out-of-bounds hatch does not sink to Z_UNKNOWN. The object
            // may rotate it on top of the symbol's own pattern angle.
            if let Some(sym) = symbol {
                let obj_rot = area_obj.pattern_rotation.rotation as f32;
                let rings = area::Rings {
                    exterior: &exterior,
                    interiors: &interiors,
                };
                for (pattern, z) in area::build_area_patterns(sym, ctx, rings, obj_rot) {
                    out.push(Primitive {
                        kind: Kind::Pattern(pattern),
                        z,
                        min: geo_min,
                        max: geo_max,
                    });
                    emitted = true;
                }
            }

            // Triangulated from these exact rings, not the crate's own
            // `Polygon`, so vertices and earcut indices stay consistent.
            if let Some(fill) = fill {
                out.push(Primitive {
                    kind: Kind::Area {
                        fill_indices: geometry::triangulate_rings(&exterior, &interiors),
                        exterior: exterior.iter().map(|p| geo.to_mercator(*p)).collect(),
                        interiors: interiors
                            .iter()
                            .map(|r| r.iter().map(|p| geo.to_mercator(*p)).collect())
                            .collect(),
                        fill,
                    },
                    z: fill_z,
                    min: geo_min,
                    max: geo_max,
                });
            } else if !emitted {
                // Nothing drawable resolved: a faint outline so the object is
                // still visible.
                out.push(Primitive {
                    kind: Kind::FallbackOutline {
                        ring: exterior.iter().map(|p| geo.to_mercator(*p)).collect(),
                    },
                    z: Z_UNKNOWN,
                    min: geo_min,
                    max: geo_max,
                });
            }
            (min, max)
        }
        MapObject::Line(line_obj) => {
            let path = paper::line_path(line_obj);
            // Bounds come from the control polygon, which always contains the
            // curve the painter will actually draw.
            let (min, max) = path.control_bounds();
            if path.anchors.len() < 2 {
                return (min, max);
            }
            let (geo_min, geo_max) = geo.mercator_bounds(min, max);

            // Start/end/mid/dash point-symbol decorations along the line. These
            // are placed by arc length, once, so they need a flattened form.
            if let Some(s) = symbol {
                let (points, dash_points) = path.flatten(paper::BEZIER_FLATTEN_ERROR_MM);
                line::place_line_decorations(s, ctx, &points, &dash_points, out);
            }

            let strokes = symbol.map(|s| line::line_style(s, ctx)).unwrap_or_default();

            if strokes.is_empty() {
                out.push(Primitive {
                    kind: Kind::Line {
                        path: geometry::to_path(&path, geo),
                        color: Some(eframe::egui::Color32::from_gray(90)),
                        width_mm: 0.0,
                        border: None,
                        dash: None,
                    },
                    z: Z_UNKNOWN,
                    min: geo_min,
                    max: geo_max,
                });
            } else {
                for stroke in strokes {
                    out.push(Primitive {
                        kind: Kind::Line {
                            path: geometry::to_path(&path, geo),
                            color: stroke.color,
                            width_mm: stroke.width_mm,
                            border: stroke.border,
                            dash: stroke.dash,
                        },
                        z: stroke.z,
                        min: geo_min,
                        max: geo_max,
                    });
                }
            }
            (min, max)
        }
        MapObject::Point(point_obj) => {
            let at = paper::to_pos2(point_obj.geometry().0);

            // Resolve the symbol into marks (base circle + element sub-shapes),
            // each with its own colour priority.
            let mut marks: Vec<(point::LocalMark, i32)> = Vec::new();
            if let Some(Symbol::Point(ps)) = symbol {
                point::build_point_marks(ps, colors, eframe::egui::Vec2::ZERO, 0, &mut marks);
                // Rotatable symbols honour the object's rotation. Positive
                // rotation is CCW in map (y-up) space — the SAME convention as
                // area fill patterns (`extra_angle`); keep them in sync.
                if ps.is_rotatable && point_obj.rotation != 0.0 {
                    let (sin, cos) = (point_obj.rotation as f32).sin_cos();
                    point::rotate_marks(&mut marks, cos, sin);
                }
            }

            if marks.is_empty() {
                let z = symbol
                    .and_then(|s| crate::rendering::color::ground_priority(s, symbol_set, colors))
                    .unwrap_or(Z_UNKNOWN);
                let at_pos = geo.to_mercator(at);
                out.push(Primitive {
                    kind: Kind::UnresolvedPoint { at: at_pos },
                    z,
                    min: at_pos,
                    max: at_pos,
                });
                return (at, at);
            }

            // Each mark is its own primitive at its own colour priority (like
            // area fills/borders/patterns), so a point's parts layer correctly
            // against other objects instead of all sitting at one shared z.
            for (mark, z) in &marks {
                let (lo, hi) = point::mark_extent(mark);
                let (min, max) = geo.mercator_bounds(at + lo, at + hi);
                out.push(Primitive {
                    kind: Kind::PointMark(point::resolve_mark(mark, at, geo)),
                    z: *z,
                    min,
                    max,
                });
            }
            (at, at)
        }
        // Generated maps currently contain no text objects.
        MapObject::Text(_) => paper::bounds_of(std::iter::empty()),
    }
}

/// Build a border line primitive for one ring of an area outline. The ring is
/// already closed — `omap` closes every Bézier ring, as `Polygon` always was.
fn outline_primitive(
    ring: &PaperPath,
    outline: &area::OutlineStroke,
    geo: &GeoTransform,
) -> Option<Primitive> {
    if ring.anchors.len() < 2 {
        return None;
    }
    let (min, max) = ring.control_bounds();
    let (geo_min, geo_max) = geo.mercator_bounds(min, max);
    Some(Primitive {
        kind: Kind::Line {
            path: geometry::to_path(ring, geo),
            color: Some(outline.color),
            width_mm: outline.width_mm,
            border: None,
            dash: outline.dash.clone(),
        },
        z: outline.z,
        min: geo_min,
        max: geo_max,
    })
}
