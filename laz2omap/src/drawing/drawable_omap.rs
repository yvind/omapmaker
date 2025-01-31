use eframe::egui;
use geo::TriangulateEarcut;
use proj4rs::{transform::transform, Proj};

use omap::Symbol;

pub trait Drawable {
    /// what fill to use for drawing symbol equals the stroke of a line or
    /// color of a polygon or color and radius of point
    fn fill(&self) -> egui::Stroke;

    /// converting a symbol to something drawable to screen
    /// needs to know what crs to project to lat/lon
    fn into_drawable_geometry(self, crs: Option<u16>) -> DrawableGeometry;
}

impl Drawable for Symbol {
    fn fill(&self) -> egui::Stroke {
        match self {
            Symbol::Contour(_) => egui::Stroke::new(3., egui::Color32::BROWN),
            Symbol::SlopelineContour(_, _) => egui::Stroke::new(3., egui::Color32::BROWN),
            Symbol::BasemapContour(_) => {
                egui::Stroke::new(1., egui::Color32::BROWN.gamma_multiply(0.5))
            }
            Symbol::IndexContour(_) => egui::Stroke::new(5., egui::Color32::BROWN),
            Symbol::Formline(_) => egui::Stroke::new(1.5, egui::Color32::BROWN),
            Symbol::SlopelineFormline(_, _) => egui::Stroke::new(1.5, egui::Color32::BROWN),
            Symbol::SmallBoulder(_) => egui::Stroke::new(6., egui::Color32::BLACK),
            Symbol::LargeBoulder(_) => egui::Stroke::new(10., egui::Color32::BLACK),
            Symbol::GiganticBoulder(_) => egui::Stroke::new(0., egui::Color32::BLACK),
            Symbol::SandyGround(_) => egui::Stroke::new(0., egui::Color32::YELLOW),
            Symbol::BareRock(_) => egui::Stroke::new(0., egui::Color32::GRAY),
            Symbol::RoughOpenLand(_) => egui::Stroke::new(0., egui::Color32::LIGHT_YELLOW),
            Symbol::LightGreen(_) => egui::Stroke::new(0., egui::Color32::LIGHT_GREEN),
            Symbol::MediumGreen(_) => egui::Stroke::new(0., egui::Color32::GREEN),
            Symbol::DarkGreen(_) => egui::Stroke::new(0., egui::Color32::DARK_GREEN),
            Symbol::Building(_) => egui::Stroke::new(0., egui::Color32::BLACK),
        }
    }

    fn into_drawable_geometry(self, crs: Option<u16>) -> DrawableGeometry {
        let stroke = self.fill();

        match self {
            Symbol::Contour(line_string) => {
                DrawableGeometry::Line(stroke, LineObject::from_geo(line_string, crs))
            }
            Symbol::SlopelineContour(point, _) => {
                DrawableGeometry::Point(stroke, PointObject::from_geo(point, crs))
            }
            Symbol::BasemapContour(line_string) => {
                DrawableGeometry::Line(stroke, LineObject::from_geo(line_string, crs))
            }
            Symbol::IndexContour(line_string) => {
                DrawableGeometry::Line(stroke, LineObject::from_geo(line_string, crs))
            }
            Symbol::Formline(line_string) => {
                DrawableGeometry::Line(stroke, LineObject::from_geo(line_string, crs))
            }
            Symbol::SlopelineFormline(point, _) => {
                DrawableGeometry::Point(stroke, PointObject::from_geo(point, crs))
            }
            Symbol::SmallBoulder(point) => {
                DrawableGeometry::Point(stroke, PointObject::from_geo(point, crs))
            }
            Symbol::LargeBoulder(point) => {
                DrawableGeometry::Point(stroke, PointObject::from_geo(point, crs))
            }
            Symbol::GiganticBoulder(polygon) => {
                DrawableGeometry::Polygon(stroke.color, PolygonObject::from_geo(polygon, crs))
            }
            Symbol::SandyGround(polygon) => {
                DrawableGeometry::Polygon(stroke.color, PolygonObject::from_geo(polygon, crs))
            }
            Symbol::BareRock(polygon) => {
                DrawableGeometry::Polygon(stroke.color, PolygonObject::from_geo(polygon, crs))
            }
            Symbol::RoughOpenLand(polygon) => {
                DrawableGeometry::Polygon(stroke.color, PolygonObject::from_geo(polygon, crs))
            }
            Symbol::LightGreen(polygon) => {
                DrawableGeometry::Polygon(stroke.color, PolygonObject::from_geo(polygon, crs))
            }
            Symbol::MediumGreen(polygon) => {
                DrawableGeometry::Polygon(stroke.color, PolygonObject::from_geo(polygon, crs))
            }
            Symbol::DarkGreen(polygon) => {
                DrawableGeometry::Polygon(stroke.color, PolygonObject::from_geo(polygon, crs))
            }
            Symbol::Building(polygon) => {
                DrawableGeometry::Polygon(stroke.color, PolygonObject::from_geo(polygon, crs))
            }
        }
    }
}

#[derive(Clone)]
pub struct DrawableOmap(Vec<DrawableGeometry>);

impl DrawableOmap {
    pub fn from_symbols(syms: impl IntoIterator<Item = Symbol>, crs: Option<u16>) -> Self {
        DrawableOmap(
            syms.into_iter()
                .map(|s| s.into_drawable_geometry(crs))
                .collect(),
        )
    }

    pub fn draw(&self, ui: &mut egui::Ui, projector: &walkers::Projector) {
        // order is determined by the order in the vec
        // should be in reverse order of ISOM color appendix,
        // ie yellow first and so on
        for ms in self.0.iter() {
            ms.draw(ui, projector);
        }
    }
}

impl DrawableGeometry {
    fn draw(&self, ui: &mut egui::Ui, projector: &walkers::Projector) {
        match &self {
            DrawableGeometry::Polygon(color, poly) => {
                poly.draw(ui, projector, color);
            }
            DrawableGeometry::Line(stroke, line) => {
                line.draw(ui, projector, &stroke);
            }
            DrawableGeometry::Point(stroke, point) => {
                point.draw(ui, projector, &stroke);
            }
        }
    }
}

#[derive(Clone)]
pub enum DrawableGeometry {
    Polygon(egui::Color32, PolygonObject),
    Line(egui::Stroke, LineObject),
    Point(egui::Stroke, PointObject),
}

#[derive(Clone)]
pub struct PolygonObject(Triangulation);

impl PolygonObject {
    fn draw(&self, ui: &mut egui::Ui, projector: &walkers::Projector, color: &egui::Color32) {
        self.0.draw(ui, projector, color);
    }

    fn from_geo(poly: geo::Polygon, crs: Option<u16>) -> Self {
        let tri = poly.earcut_triangles_raw();

        let mut verts: Vec<(f64, f64)> = tri.vertices.chunks(2).map(|c| (c[0], c[1])).collect();
        if let Some(epsg) = crs {
            let geo_proj = Proj::from_epsg_code(4326).unwrap();
            let local_proj = Proj::from_epsg_code(epsg).unwrap();

            transform(&local_proj, &geo_proj, verts.as_mut_slice());
        }
        let obj = verts
            .iter()
            .map(|c| walkers::Position::new(c.0, c.1))
            .collect();

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
    fn draw(&self, ui: &mut egui::Ui, projector: &walkers::Projector, color: &egui::Color32) {
        let painter = ui.painter();

        let points = self
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

        painter.add(egui::Shape::Mesh(mesh));
    }
}
#[derive(Clone)]
pub struct LineObject(Vec<walkers::Position>);

impl LineObject {
    fn draw(&self, ui: &mut egui::Ui, projector: &walkers::Projector, stroke: &egui::Stroke) {
        let painter = ui.painter();

        let points = self.0.iter().map(|p| projector.project(*p)).collect();

        painter.line(points, *stroke);
    }

    fn from_geo(line: geo::LineString, crs: Option<u16>) -> Self {
        let obj = if let Some(epsg) = crs {
            let geo_proj = Proj::from_epsg_code(4326).unwrap();
            let local_proj = Proj::from_epsg_code(epsg).unwrap();

            let mut line: Vec<(f64, f64)> = line.0.iter().map(|c| (c.x, c.y)).collect();

            transform(&local_proj, &geo_proj, line.as_mut_slice());

            line.iter()
                .map(|c| walkers::Position::new(c.0, c.1))
                .collect()
        } else {
            line.coords()
                .map(|c| walkers::Position::new(c.x, c.y))
                .collect()
        };

        LineObject(obj)
    }
}

#[derive(Clone)]
pub struct PointObject(walkers::Position);

impl PointObject {
    fn draw(&self, ui: &mut egui::Ui, projector: &walkers::Projector, stroke: &egui::Stroke) {
        let painter = ui.painter();
        let screen_point = projector.project(self.0);

        painter.circle_filled(screen_point, stroke.width, stroke.color);
    }

    fn from_geo(point: geo::Point, crs: Option<u16>) -> Self {
        let pos = if let Some(epsg) = crs {
            let geo_proj = Proj::from_epsg_code(4326).unwrap();
            let local_proj = Proj::from_epsg_code(epsg).unwrap();

            let mut p = (point.x(), point.y());
            transform(&local_proj, &geo_proj, &mut p);

            walkers::Position::new(p.0, p.1)
        } else {
            walkers::Position::new(point.x(), point.y())
        };

        PointObject(pos)
    }
}
