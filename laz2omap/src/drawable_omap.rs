use eframe::egui;
use geo::{Coord, TriangulateEarcut};
use proj4rs::{transform::transform, Proj};

use omap::{MapObject, Omap};

trait Drawable {
    /// what fill to use for drawing symbol equals the stroke of a line or
    /// color of a polygon or color and radius of point
    fn fill(&self) -> egui::Stroke;

    /// converting a symbol to something drawable to screen
    /// needs to know what crs to project to lat/lon
    fn into_drawable_geometry(self, ref_point: Coord, crs: Option<u16>) -> DrawableGeometry;
}

impl Drawable for MapObject {
    fn fill(&self) -> egui::Stroke {
        match self {
            MapObject::LineObject(line_object) => match line_object.symbol {
                omap::LineSymbol::Contour => egui::Stroke::new(3., egui::Color32::BROWN),
                omap::LineSymbol::BasemapContour => {
                    egui::Stroke::new(1., egui::Color32::BROWN.gamma_multiply(0.5))
                }
                omap::LineSymbol::IndexContour => egui::Stroke::new(5., egui::Color32::BROWN),
                omap::LineSymbol::Formline => egui::Stroke::new(1.5, egui::Color32::BROWN),
            },
            MapObject::PointObject(point_object) => match point_object.symbol {
                omap::PointSymbol::SlopelineContour => egui::Stroke::new(3., egui::Color32::BROWN),
                omap::PointSymbol::SlopelineFormline => {
                    egui::Stroke::new(1.5, egui::Color32::BROWN)
                }
                omap::PointSymbol::DotKnoll => egui::Stroke::new(6., egui::Color32::BROWN),
                omap::PointSymbol::ElongatedDotKnoll => egui::Stroke::new(6., egui::Color32::BROWN),
                omap::PointSymbol::UDepression => egui::Stroke::new(6., egui::Color32::RED),
                omap::PointSymbol::SmallBoulder => egui::Stroke::new(6., egui::Color32::BLACK),
                omap::PointSymbol::LargeBoulder => egui::Stroke::new(10., egui::Color32::BLACK),
            },
            MapObject::AreaObject(area_object) => match area_object.symbol {
                omap::AreaSymbol::GiganticBoulder => egui::Stroke::new(0., egui::Color32::BLACK),
                omap::AreaSymbol::SandyGround => egui::Stroke::new(0., egui::Color32::YELLOW),
                omap::AreaSymbol::BareRock => egui::Stroke::new(0., egui::Color32::GRAY),
                omap::AreaSymbol::RoughOpenLand => {
                    egui::Stroke::new(0., egui::Color32::LIGHT_YELLOW)
                }
                omap::AreaSymbol::LightGreen => egui::Stroke::new(0., egui::Color32::LIGHT_GREEN),
                omap::AreaSymbol::MediumGreen => egui::Stroke::new(0., egui::Color32::GREEN),
                omap::AreaSymbol::DarkGreen => egui::Stroke::new(0., egui::Color32::DARK_GREEN),
                omap::AreaSymbol::Building => egui::Stroke::new(0., egui::Color32::BLACK),
                omap::AreaSymbol::Water => egui::Stroke::new(0., egui::Color32::BLUE),
            },
        }
    }

    fn into_drawable_geometry(self, ref_point: Coord, crs: Option<u16>) -> DrawableGeometry {
        let stroke = self.fill();

        match self {
            MapObject::LineObject(line_object) => DrawableGeometry::Line(
                stroke,
                LineObject::from_geo(line_object.line, ref_point, crs),
            ),
            MapObject::PointObject(point_object) => DrawableGeometry::Point(
                stroke,
                PointObject::from_geo(point_object.point, ref_point, crs),
            ),
            MapObject::AreaObject(area_object) => DrawableGeometry::Polygon(
                stroke.color,
                PolygonObject::from_geo(area_object.polygon, ref_point, crs),
            ),
        }
    }
}

#[derive(Clone)]
pub struct DrawableOmap {
    convex_hull: Vec<walkers::Position>,
    map_objects: Vec<DrawableGeometry>,
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

        DrawableOmap {
            convex_hull: global_hull,
            map_objects: omap
                .into_objects()
                .map(|o| o.into_drawable_geometry(ref_point, crs))
                .collect(),
        }
    }

    pub fn draw(&self, ui: &mut egui::Ui, projector: &walkers::Projector) {
        // order is determined by the order in the vec
        // should be in reverse order of ISOM color appendix,
        // ie yellow first and so on
        for ms in self.map_objects.iter() {
            ms.draw(ui, projector);
        }

        // project the hull:

        let points = self
            .convex_hull
            .clone()
            .into_iter()
            .map(|p| projector.project(p))
            .collect();

        ui.painter()
            .line(points, egui::Stroke::new(2., egui::Color32::RED));
    }
}

impl DrawableGeometry {
    fn draw(&self, ui: &mut egui::Ui, projector: &walkers::Projector) {
        match &self {
            DrawableGeometry::Polygon(color, poly) => {
                poly.draw(ui, projector, color);
            }
            DrawableGeometry::Line(stroke, line) => {
                line.draw(ui, projector, stroke);
            }
            DrawableGeometry::Point(stroke, point) => {
                point.draw(ui, projector, stroke);
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
    fn draw(&self, ui: &mut egui::Ui, projector: &walkers::Projector, color: &egui::Color32) {
        let painter = ui.painter();

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
    fn draw(&self, ui: &mut egui::Ui, projector: &walkers::Projector, stroke: &egui::Stroke) {
        let painter = ui.painter();
        let screen_point = projector.project(self.0);

        painter.circle_filled(screen_point, stroke.width, stroke.color);
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
