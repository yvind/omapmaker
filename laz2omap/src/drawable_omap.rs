use std::collections::{hash_map::Keys, HashMap};

use eframe::{
    egui::{self, Color32, Stroke},
    emath,
    epaint::CubicBezierShape,
};
use geo::{Coord, LineString, TriangulateEarcut};
use polyline2bezier::BezierString;
use proj4rs::{transform::transform, Proj};

use omap::{MapObject, Omap, Symbol};
use strum::IntoEnumIterator;

const PURPLE: Color32 = Color32::from_rgba_premultiplied(190, 60, 255, 255);
const ROUGH_YELLOW: Color32 = Color32::from_rgba_premultiplied(255, 220, 155, 255);
const BROWN: Color32 = Color32::from_rgba_premultiplied(180, 50, 0, 255);
const MEDIUM_BROWN: Color32 = Color32::from_rgba_premultiplied(200, 80, 0, 255);
const LIGHT_BROWN: Color32 = Color32::from_rgba_premultiplied(220, 110, 0, 255);

trait DrawableSymbol {
    /// what fill to use for drawing symbol equals the stroke of a line or
    /// color of a polygon or color and radius of point
    fn stroke(&self, pixels_per_meter: f32) -> (bool, Stroke);
}

impl DrawableSymbol for Symbol {
    fn stroke(&self, pixels_per_meter: f32) -> (bool, Stroke) {
        let scale_factor = 0.25 * pixels_per_meter;

        match self {
            Symbol::Contour => (false, Stroke::new(3. * scale_factor, BROWN)),
            Symbol::BasemapContour => (false, Stroke::new(1. * scale_factor, LIGHT_BROWN)),
            Symbol::NegBasemapContour => (false, Stroke::new(1. * scale_factor, PURPLE)),
            Symbol::IndexContour => (false, Stroke::new(5. * scale_factor, BROWN)),
            Symbol::Formline => (true, Stroke::new(2. * scale_factor, MEDIUM_BROWN)),
            Symbol::SlopelineContour => (false, Stroke::new(3. * scale_factor, BROWN)),
            Symbol::SlopelineFormline => (false, Stroke::new(2. * scale_factor, BROWN)),
            Symbol::DotKnoll => (false, Stroke::new(8. * scale_factor, BROWN)),
            Symbol::ElongatedDotKnoll => (true, Stroke::new(8. * scale_factor, BROWN)),
            Symbol::UDepression => (false, Stroke::new(8. * scale_factor, PURPLE)),
            Symbol::SmallBoulder => (false, Stroke::new(8. * scale_factor, Color32::BLACK)),
            Symbol::LargeBoulder => (false, Stroke::new(12. * scale_factor, Color32::BLACK)),
            Symbol::GiganticBoulder => (false, Stroke::new(0. * scale_factor, Color32::BLACK)),
            Symbol::SandyGround => (false, Stroke::new(0. * scale_factor, Color32::YELLOW)),
            Symbol::BareRock => (false, Stroke::new(0. * scale_factor, Color32::GRAY)),
            Symbol::RoughOpenLand => (false, Stroke::new(0. * scale_factor, ROUGH_YELLOW)),
            Symbol::LightGreen => (false, Stroke::new(0. * scale_factor, Color32::LIGHT_GREEN)),
            Symbol::MediumGreen => (false, Stroke::new(0. * scale_factor, Color32::GREEN)),
            Symbol::DarkGreen => (false, Stroke::new(0. * scale_factor, Color32::DARK_GREEN)),
            Symbol::Building => (false, Stroke::new(0. * scale_factor, Color32::BLACK)),
            Symbol::Water => (false, Stroke::new(0. * scale_factor, Color32::BLUE)),
            Symbol::PavedArea => (false, Stroke::new(0. * scale_factor, LIGHT_BROWN)),
        }
    }
}

trait Drawable {
    /// converting a symbol to something drawable to screen
    /// needs to know what crs to project to lat/lon
    fn into_drawable_geometry(
        self,
        ref_point: Coord,
        crs: Option<u16>,
        bezier_error: Option<f64>,
    ) -> DrawableGeometry;
}

impl Drawable for MapObject {
    fn into_drawable_geometry(
        self,
        ref_point: Coord,
        crs: Option<u16>,
        bezier_error: Option<f64>,
    ) -> DrawableGeometry {
        match self {
            MapObject::LineObject(line_object) => DrawableGeometry::Line(LineObject::from_geo(
                line_object.line,
                ref_point,
                crs,
                bezier_error,
            )),
            MapObject::PointObject(point_object) => DrawableGeometry::Point(PointObject::from_geo(
                point_object.point,
                point_object.rotation,
                ref_point,
                crs,
            )),
            MapObject::AreaObject(area_object) => DrawableGeometry::Polygon(
                PolygonObject::from_geo(area_object.polygon, ref_point, crs, bezier_error),
            ),
        }
    }
}
pub struct DrawableOmap {
    hull: Vec<walkers::Position>,
    map_objects: HashMap<Symbol, Vec<DrawableGeometry>>,
}

impl DrawableOmap {
    pub fn keys(&self) -> Keys<'_, Symbol, Vec<DrawableGeometry>> {
        self.map_objects.keys()
    }

    pub fn from_omap(omap: Omap, hull: geo::LineString, bezier_error: Option<f64>) -> Self {
        let ref_point = omap.get_ref_point();
        let crs = omap.get_crs();

        let global_hull = if let Some(epsg) = crs {
            let wgs = Proj::from_epsg_code(4326).unwrap();
            let local = Proj::from_epsg_code(epsg).unwrap();

            let mut points: Vec<(f64, f64)> = hull
                .0
                .into_iter()
                .map(|c| (c.x + ref_point.x, c.y + ref_point.y))
                .collect();

            transform(&local, &wgs, points.as_mut_slice()).unwrap();

            points
                .into_iter()
                .map(|t| walkers::pos_from_lon_lat(t.0.to_degrees(), t.1.to_degrees()))
                .collect()
        } else {
            hull.0
                .into_iter()
                .map(|c| walkers::pos_from_lon_lat(c.x + ref_point.x, c.y + ref_point.y))
                .collect()
        };

        // draw-order is determined by the order in the vec
        // should be in reverse order of ISOM color appendix,
        // ie yellow first and so on
        // this must be revised when multithreading is added as
        // the order may vary then
        DrawableOmap {
            hull: global_hull,
            map_objects: Self::into_drawable(omap.objects, ref_point, crs, bezier_error),
        }
    }

    fn into_drawable(
        mut omap_objs: HashMap<Symbol, Vec<MapObject>>,
        ref_point: Coord,
        crs: Option<u16>,
        bezier_error: Option<f64>,
    ) -> HashMap<Symbol, Vec<DrawableGeometry>> {
        let mut drawable_objs = HashMap::with_capacity(omap_objs.len());
        for (symbol, objs) in omap_objs.drain() {
            let bezier = match symbol {
                // basemap should never be converted to beziers
                Symbol::BasemapContour | Symbol::NegBasemapContour => None,
                _ => bezier_error,
            };

            drawable_objs.insert(
                symbol,
                objs.into_iter()
                    .map(|o| o.into_drawable_geometry(ref_point, crs, bezier))
                    .collect(),
            );
        }

        drawable_objs
    }

    pub fn update(&mut self, mut other: Self) {
        // assumes that the omap used for any update and only differs in the contained map_objects

        for (key, objs) in other.map_objects.drain() {
            let _ = self.map_objects.insert(key, objs);
        }
    }

    pub fn draw(
        &self,
        ui: &mut egui::Ui,
        projector: &walkers::Projector,
        visabilities: &HashMap<Symbol, bool>,
        opacity: f32,
    ) {
        // project the hull:
        let points = self
            .hull
            .clone()
            .into_iter()
            .map(|p| projector.project(p))
            .collect();

        // not necessarily a convex polygon, but close
        ui.painter().add(egui::Shape::convex_polygon(
            points,
            Color32::WHITE.gamma_multiply(0.8).gamma_multiply(opacity),
            Stroke::new(2., Color32::RED),
        ));

        for symbol in Symbol::iter() {
            let vis = visabilities.get(&symbol);
            if let Some(vis) = vis {
                if !vis {
                    continue;
                }
            } else {
                continue;
            }

            let (special, stroke) = symbol.stroke(
                projector.scale_pixel_per_meter(projector.unproject(emath::Pos2::new(0.5, 0.5))),
            );
            let stroke = Stroke {
                width: stroke.width,
                color: stroke.color.gamma_multiply(opacity),
            };

            if let Some(objs) = self.map_objects.get(&symbol) {
                for obj in objs {
                    obj.draw(ui, projector, stroke, special);
                }
            }
        }
    }
}

impl DrawableGeometry {
    fn draw(
        &self,
        ui: &mut egui::Ui,
        projector: &walkers::Projector,
        stroke: Stroke,
        special: bool,
    ) {
        match &self {
            DrawableGeometry::Polygon(poly) => {
                poly.draw(ui, projector, &stroke.color, special);
            }
            DrawableGeometry::Line(line) => {
                line.draw(ui, projector, &stroke, special);
            }
            DrawableGeometry::Point(point) => {
                point.draw(ui, projector, &stroke, special);
            }
        }
    }
}

#[derive(Clone)]
pub enum DrawableGeometry {
    Polygon(PolygonObject),
    Line(LineObject),
    Point(PointObject),
}

#[derive(Clone)]
pub struct PolygonObject(Triangulation);

impl PolygonObject {
    fn draw(
        &self,
        ui: &mut egui::Ui,
        projector: &walkers::Projector,
        color: &Color32,
        special: bool,
    ) {
        self.0.draw(ui, projector, color, special);
    }

    fn from_geo(
        poly: geo::Polygon,
        ref_point: Coord,
        crs: Option<u16>,
        _bezier_error: Option<f64>,
    ) -> Self {
        let tri = poly.earcut_triangles_raw();

        let mut verts: Vec<(f64, f64)> = tri
            .vertices
            .chunks(2)
            .map(|c| (c[0] + ref_point.x, c[1] + ref_point.y))
            .collect();
        let obj = if let Some(epsg) = crs {
            let geo_proj = Proj::from_epsg_code(4326).unwrap();
            let local_proj = Proj::from_epsg_code(epsg).unwrap();

            let _ = transform(&local_proj, &geo_proj, verts.as_mut_slice());
            verts
                .iter()
                .map(|c| walkers::pos_from_lon_lat(c.0.to_degrees(), c.1.to_degrees()))
                .collect()
        } else {
            verts
                .iter()
                .map(|c| walkers::pos_from_lon_lat(c.0, c.1))
                .collect()
        };

        let triangulation = Triangulation {
            indices: tri.triangle_indices.iter().map(|t| *t as u32).collect(),
            vertices: obj,
        };

        PolygonObject(triangulation)
    }
}

#[derive(Clone)]
pub struct Triangulation {
    indices: Vec<u32>,
    vertices: Vec<walkers::Position>,
}

impl Triangulation {
    fn draw(
        &self,
        ui: &mut egui::Ui,
        projector: &walkers::Projector,
        color: &Color32,
        special: bool,
    ) {
        let pos: Vec<egui::Pos2> = self
            .vertices
            .iter()
            .map(|p| projector.project(*p))
            .collect();

        let points: Vec<egui::epaint::Vertex> = pos
            .iter()
            .map(|p| egui::epaint::Vertex {
                pos: *p,
                uv: egui::epaint::WHITE_UV,
                color: *color,
            })
            .collect();

        let mesh = egui::Mesh {
            indices: self.indices.clone(),
            vertices: points,
            texture_id: egui::TextureId::Managed(0),
        };

        ui.painter().add(egui::Shape::Mesh(mesh.into()));

        // bounding line
        if special {
            ui.painter()
                .line(pos, egui::Stroke::new(3., egui::Color32::BLACK));
        }
    }
}

#[derive(Clone)]
pub struct LineObject(Vec<walkers::Position>, bool);

impl LineObject {
    fn draw(
        &self,
        ui: &mut egui::Ui,
        projector: &walkers::Projector,
        stroke: &Stroke,
        dashed: bool,
    ) {
        let points = self
            .0
            .iter()
            .map(|p| projector.project(*p))
            .collect::<Vec<_>>();

        // dashed bezier not supported yet
        if self.1 {
            for bezier in points.chunks_exact(4) {
                let bezier = [bezier[0], bezier[1], bezier[2], bezier[3]];
                let bezier_shape = CubicBezierShape::from_points_stroke(
                    bezier,
                    false,
                    Color32::TRANSPARENT,
                    *stroke,
                );

                ui.painter().add(egui::Shape::CubicBezier(bezier_shape));
            }
        } else if dashed {
            ui.painter().add(egui::Shape::dashed_line(
                &points,
                *stroke,
                20. * stroke.width,
                2. * stroke.width,
            ));
        } else {
            ui.painter().line(points, *stroke);
        }
    }

    fn from_geo(
        line: geo::LineString,
        ref_point: Coord,
        crs: Option<u16>,
        bezier_error: Option<f64>,
    ) -> Self {
        let line = if let Some(bezier_error) = bezier_error {
            let mut vec = Vec::with_capacity(line.0.len());
            let bezier_string = BezierString::from_polyline(line, bezier_error);

            for segment in bezier_string.0 {
                if segment.is_bezier_segment() {
                    vec.push(segment.0 .0);
                    vec.push(segment.0 .1.unwrap());
                    vec.push(segment.0 .2.unwrap());
                    vec.push(segment.0 .3);
                } else {
                    let a = segment.0 .0 + (segment.0 .3 - segment.0 .0) / 3.;
                    let b = segment.0 .0 + (segment.0 .3 - segment.0 .0) * 2. / 3.;
                    vec.push(segment.0 .0);
                    vec.push(a);
                    vec.push(b);
                    vec.push(segment.0 .3);
                }
            }
            LineString::new(vec)
        } else {
            line
        };

        let obj = if let Some(epsg) = crs {
            let geo_proj = Proj::from_epsg_code(4326).unwrap();
            let local_proj = Proj::from_epsg_code(epsg).unwrap();

            let mut line: Vec<(f64, f64)> = line
                .0
                .iter()
                .map(|c| (c.x + ref_point.x, c.y + ref_point.y))
                .collect();

            let _ = transform(&local_proj, &geo_proj, line.as_mut_slice());

            line.iter()
                .map(|c| walkers::pos_from_lon_lat(c.0.to_degrees(), c.1.to_degrees()))
                .collect()
        } else {
            line.coords()
                .map(|c| walkers::pos_from_lon_lat(c.x + ref_point.x, c.y + ref_point.y))
                .collect()
        };

        LineObject(obj, bezier_error.is_some())
    }
}

#[derive(Clone)]
pub struct PointObject(walkers::Position, f32);

impl PointObject {
    fn draw(
        &self,
        ui: &mut egui::Ui,
        projector: &walkers::Projector,
        stroke: &Stroke,
        special: bool,
    ) {
        let screen_point = projector.project(self.0);

        if special {
            let radius = if self.1.abs() > std::f32::consts::FRAC_PI_4 {
                egui::Vec2::new(stroke.width, 1.5 * stroke.width)
            } else {
                egui::Vec2::new(1.5 * stroke.width, stroke.width)
            };

            ui.painter().add(egui::Shape::ellipse_filled(
                screen_point,
                radius,
                stroke.color,
            ));
        } else {
            ui.painter()
                .circle_filled(screen_point, stroke.width, stroke.color);
        }
    }

    fn from_geo(point: geo::Point, rot: f64, ref_point: Coord, crs: Option<u16>) -> Self {
        let pos = if let Some(epsg) = crs {
            let geo_proj = Proj::from_epsg_code(4326).unwrap();
            let local_proj = Proj::from_epsg_code(epsg).unwrap();

            let mut p = (point.x() + ref_point.x, point.y() + ref_point.y);
            let _ = transform(&local_proj, &geo_proj, &mut p);

            walkers::pos_from_lon_lat(p.0.to_degrees(), p.1.to_degrees())
        } else {
            walkers::pos_from_lon_lat(point.x() + ref_point.x, point.y() + ref_point.y)
        };

        PointObject(pos, rot as f32)
    }
}
