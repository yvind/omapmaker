use std::collections::HashMap;

use eframe::egui::{self, Color32, Stroke};
use geo::{Coord, TriangulateEarcut};
use proj4rs::{transform::transform, Proj};

use omap::{MapObject, Omap, Symbol};
use strum::IntoEnumIterator;

const PURPLE: Color32 = Color32::from_rgba_premultiplied(190, 60, 255, 255);
const ROUGH_YELLOW: Color32 = Color32::from_rgba_premultiplied(255, 220, 155, 255);

trait DrawableSymbol {
    /// what fill to use for drawing symbol equals the stroke of a line or
    /// color of a polygon or color and radius of point
    fn stroke(&self) -> Stroke;
}

impl DrawableSymbol for Symbol {
    fn stroke(&self) -> Stroke {
        match self {
            Symbol::Contour => Stroke::new(3., Color32::BROWN),
            Symbol::BasemapContour => Stroke::new(1., Color32::BROWN.gamma_multiply(0.5)),
            Symbol::NegBasemapContour => Stroke::new(1., PURPLE),
            Symbol::IndexContour => Stroke::new(5., Color32::BROWN),
            Symbol::Formline => Stroke::new(1.5, Color32::BROWN),
            Symbol::SlopelineContour => Stroke::new(3., Color32::BROWN),
            Symbol::SlopelineFormline => Stroke::new(1.5, Color32::BROWN),
            Symbol::DotKnoll => Stroke::new(6., Color32::BROWN),
            Symbol::ElongatedDotKnoll => Stroke::new(6., Color32::BROWN),
            Symbol::UDepression => Stroke::new(6., Color32::RED),
            Symbol::SmallBoulder => Stroke::new(6., Color32::BLACK),
            Symbol::LargeBoulder => Stroke::new(10., Color32::BLACK),
            Symbol::GiganticBoulder => Stroke::new(0., Color32::BLACK),
            Symbol::SandyGround => Stroke::new(0., Color32::YELLOW),
            Symbol::BareRock => Stroke::new(0., Color32::GRAY),
            Symbol::RoughOpenLand => Stroke::new(0., ROUGH_YELLOW),
            Symbol::LightGreen => Stroke::new(0., Color32::LIGHT_GREEN),
            Symbol::MediumGreen => Stroke::new(0., Color32::GREEN),
            Symbol::DarkGreen => Stroke::new(0., Color32::DARK_GREEN),
            Symbol::Building => Stroke::new(0., Color32::BLACK),
            Symbol::Water => Stroke::new(0., Color32::BLUE),
        }
    }
}

trait Drawable {
    /// converting a symbol to something drawable to screen
    /// needs to know what crs to project to lat/lon
    fn into_drawable_geometry(self, ref_point: Coord, crs: Option<u16>) -> DrawableGeometry;
}

impl Drawable for MapObject {
    fn into_drawable_geometry(self, ref_point: Coord, crs: Option<u16>) -> DrawableGeometry {
        match self {
            MapObject::LineObject(line_object) => {
                DrawableGeometry::Line(LineObject::from_geo(line_object.line, ref_point, crs))
            }
            MapObject::PointObject(point_object) => {
                DrawableGeometry::Point(PointObject::from_geo(point_object.point, ref_point, crs))
            }
            MapObject::AreaObject(area_object) => DrawableGeometry::Polygon(
                PolygonObject::from_geo(area_object.polygon, ref_point, crs),
            ),
        }
    }
}
pub struct DrawableOmap {
    convex_hull: Vec<walkers::Position>,
    map_objects: HashMap<Symbol, Vec<DrawableGeometry>>,
}

impl DrawableOmap {
    pub fn from_omap(omap: Omap, hull: geo::LineString) -> Self {
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
            convex_hull: global_hull,
            map_objects: Self::into_drawable(omap.objects, ref_point, crs),
        }
    }

    fn into_drawable(
        mut omap_objs: HashMap<Symbol, Vec<MapObject>>,
        ref_point: Coord,
        crs: Option<u16>,
    ) -> HashMap<Symbol, Vec<DrawableGeometry>> {
        let mut drawable_objs = HashMap::with_capacity(omap_objs.len());
        for (symbol, objs) in omap_objs.drain() {
            drawable_objs.insert(
                symbol,
                objs.into_iter()
                    .map(|o| o.into_drawable_geometry(ref_point, crs))
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

    pub fn draw(&self, ui: &mut egui::Ui, projector: &walkers::Projector) {
        // project the hull:

        let points = self
            .convex_hull
            .clone()
            .into_iter()
            .map(|p| projector.project(p))
            .collect();

        ui.painter().add(egui::Shape::convex_polygon(
            points,
            Color32::WHITE.gamma_multiply(0.8),
            Stroke::new(2., Color32::RED),
        ));

        for symbol in Symbol::iter() {
            let stroke = symbol.stroke();
            if let Some(objs) = self.map_objects.get(&symbol) {
                for obj in objs {
                    obj.draw(ui, projector, stroke);
                }
            }
        }
    }
}

impl DrawableGeometry {
    fn draw(&self, ui: &mut egui::Ui, projector: &walkers::Projector, stroke: Stroke) {
        match &self {
            DrawableGeometry::Polygon(poly) => {
                poly.draw(ui, projector, &stroke.color);
            }
            DrawableGeometry::Line(line) => {
                line.draw(ui, projector, &stroke);
            }
            DrawableGeometry::Point(point) => {
                point.draw(ui, projector, &stroke);
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
    fn draw(&self, ui: &mut egui::Ui, projector: &walkers::Projector, color: &Color32) {
        self.0.draw(ui, projector, color);
    }

    fn from_geo(poly: geo::Polygon, ref_point: Coord, crs: Option<u16>) -> Self {
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
    fn draw(&self, ui: &mut egui::Ui, projector: &walkers::Projector, color: &Color32) {
        let points: Vec<egui::epaint::Vertex> = self
            .vertices
            .iter()
            .map(|p| egui::epaint::Vertex {
                pos: projector.project(*p),
                uv: egui::epaint::WHITE_UV,
                color: *color,
            })
            .collect();

        let mesh = egui::Mesh {
            indices: self.indices.clone(),
            vertices: points,
            texture_id: egui::TextureId::Managed(0),
        };

        ui.painter().add(egui::Shape::Mesh(mesh));
    }
}
#[derive(Clone)]
pub struct LineObject(Vec<walkers::Position>);

impl LineObject {
    fn draw(&self, ui: &mut egui::Ui, projector: &walkers::Projector, stroke: &Stroke) {
        let points = self.0.iter().map(|p| projector.project(*p)).collect();

        ui.painter().line(points, *stroke);
    }

    fn from_geo(line: geo::LineString, ref_point: Coord, crs: Option<u16>) -> Self {
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

        LineObject(obj)
    }
}

#[derive(Clone)]
pub struct PointObject(walkers::Position);

impl PointObject {
    fn draw(&self, ui: &mut egui::Ui, projector: &walkers::Projector, stroke: &Stroke) {
        let screen_point = projector.project(self.0);

        ui.painter()
            .circle_filled(screen_point, stroke.width, stroke.color);
    }

    fn from_geo(point: geo::Point, ref_point: Coord, crs: Option<u16>) -> Self {
        let pos = if let Some(epsg) = crs {
            let geo_proj = Proj::from_epsg_code(4326).unwrap();
            let local_proj = Proj::from_epsg_code(epsg).unwrap();

            let mut p = (point.x() + ref_point.x, point.y() + ref_point.y);
            let _ = transform(&local_proj, &geo_proj, &mut p);

            walkers::pos_from_lon_lat(p.0.to_degrees(), p.1.to_degrees())
        } else {
            walkers::pos_from_lon_lat(point.x() + ref_point.x, point.y() + ref_point.y)
        };

        PointObject(pos)
    }
}
