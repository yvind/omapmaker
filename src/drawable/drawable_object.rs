use crate::Result;
use eframe::{
    egui::{self, Align2, Color32, Stroke},
    epaint::CubicBezierShape,
};
use geo::{Coord, LineString, TriangulateEarcut};
use linestring2bezier::BezierString;
use proj4rs::{transform::transform, Proj};

#[derive(Clone)]
pub enum DrawableGeometry {
    Polygon(DrawablePolygonObject),
    Line(DrawableLineObject),
    Point(DrawablePointObject),
    Text(DrawableTextObject),
}

impl DrawableGeometry {
    pub(crate) fn draw(
        &self,
        ui: &mut egui::Ui,
        projector: &walkers::Projector,
        stroke: Stroke,
        special: bool,
    ) {
        match &self {
            DrawableGeometry::Polygon(poly) => {
                poly.draw(ui, projector, &stroke, special);
            }
            DrawableGeometry::Line(line) => {
                line.draw(ui, projector, &stroke, special);
            }
            DrawableGeometry::Point(point) => {
                point.draw(ui, projector, &stroke, special);
            }
            DrawableGeometry::Text(text) => {
                text.draw(ui, projector, &stroke);
            }
        }
    }
}

#[derive(Clone)]
pub struct DrawablePolygonObject(Triangulation);

impl DrawablePolygonObject {
    pub(crate) fn draw(
        &self,
        ui: &mut egui::Ui,
        projector: &walkers::Projector,
        stroke: &Stroke,
        special: bool,
    ) {
        self.0.draw(ui, projector, stroke, special);
    }

    pub(crate) fn from_geo(
        poly: geo::Polygon,
        ref_point: Coord,
        crs: Option<u16>,
        _bezier_error: Option<f64>,
    ) -> Result<Self> {
        let tri = poly.earcut_triangles_raw();

        let mut vertices: Vec<(f64, f64)> = tri
            .vertices
            .chunks(2)
            .map(|c| (c[0] + ref_point.x, c[1] + ref_point.y))
            .collect();
        let obj = if let Some(epsg) = crs {
            let geo_proj = Proj::from_epsg_code(4326)?;
            let local_proj = Proj::from_epsg_code(epsg)?;

            transform(&local_proj, &geo_proj, vertices.as_mut_slice())?;
            vertices
                .iter()
                .map(|c| walkers::pos_from_lon_lat(c.0.to_degrees(), c.1.to_degrees()))
                .collect()
        } else {
            vertices
                .iter()
                .map(|c| walkers::pos_from_lon_lat(c.0, c.1))
                .collect()
        };

        let triangulation = Triangulation {
            indices: tri.triangle_indices.iter().map(|t| *t as u32).collect(),
            vertices: obj,
        };

        Ok(DrawablePolygonObject(triangulation))
    }
}

#[derive(Clone)]
pub struct Triangulation {
    indices: Vec<u32>,
    vertices: Vec<walkers::Position>,
}

impl Triangulation {
    pub(crate) fn draw(
        &self,
        ui: &mut egui::Ui,
        projector: &walkers::Projector,
        stroke: &Stroke,
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
                color: stroke.color,
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
            ui.painter().line(
                pos,
                egui::Stroke::new(stroke.width, stroke.color.to_opaque()),
            );
        }
    }
}

#[derive(Clone)]
pub struct DrawableLineObject(Vec<walkers::Position>, bool);

impl DrawableLineObject {
    pub(crate) fn draw(
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
                40. * stroke.width,
                8. * stroke.width,
            ));
        } else {
            ui.painter().line(points, *stroke);
        }
    }

    pub(crate) fn from_geo(
        line: geo::LineString,
        ref_point: Coord,
        crs: Option<u16>,
        bezier_error: Option<f64>,
    ) -> Result<Self> {
        let line = if let Some(bezier_error) = bezier_error {
            let mut vec = Vec::with_capacity(line.0.len());
            let bezier_string = BezierString::from_linestring(line, bezier_error);

            for segment in bezier_string.0 {
                vec.push(segment.start);
                if let Some(handles) = segment.handles {
                    vec.push(handles.0);
                    vec.push(handles.1);
                } else {
                    vec.push(segment.start + (segment.end - segment.start) / 3.);
                    vec.push(segment.start + (segment.end - segment.start) * 2. / 3.);
                }
                vec.push(segment.end);
            }
            LineString::new(vec)
        } else {
            line
        };

        let obj = if let Some(epsg) = crs {
            let geo_proj = Proj::from_epsg_code(4326)?;
            let local_proj = Proj::from_epsg_code(epsg)?;

            let mut line: Vec<(f64, f64)> = line
                .0
                .iter()
                .map(|c| (c.x + ref_point.x, c.y + ref_point.y))
                .collect();

            transform(&local_proj, &geo_proj, line.as_mut_slice())?;

            line.iter()
                .map(|c| walkers::pos_from_lon_lat(c.0.to_degrees(), c.1.to_degrees()))
                .collect()
        } else {
            line.coords()
                .map(|c| walkers::pos_from_lon_lat(c.x + ref_point.x, c.y + ref_point.y))
                .collect()
        };

        Ok(DrawableLineObject(obj, bezier_error.is_some()))
    }
}

#[derive(Clone)]
pub struct DrawablePointObject(walkers::Position, f32);

impl DrawablePointObject {
    pub(crate) fn draw(
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

    pub(crate) fn from_geo(
        point: geo::Point,
        rot: f64,
        ref_point: Coord,
        crs: Option<u16>,
    ) -> Result<Self> {
        let pos = if let Some(epsg) = crs {
            let geo_proj = Proj::from_epsg_code(4326)?;
            let local_proj = Proj::from_epsg_code(epsg)?;

            let mut p = (point.x() + ref_point.x, point.y() + ref_point.y);
            transform(&local_proj, &geo_proj, &mut p)?;

            walkers::pos_from_lon_lat(p.0.to_degrees(), p.1.to_degrees())
        } else {
            walkers::pos_from_lon_lat(point.x() + ref_point.x, point.y() + ref_point.y)
        };

        Ok(DrawablePointObject(pos, rot as f32))
    }
}

#[derive(Clone)]
pub struct DrawableTextObject(walkers::Position, String);

impl DrawableTextObject {
    pub(crate) fn draw(&self, ui: &mut egui::Ui, projector: &walkers::Projector, stroke: &Stroke) {
        let screen_point = projector.project(self.0);

        ui.painter().text(
            screen_point,
            Align2::CENTER_CENTER,
            &self.1,
            egui::FontId::new(stroke.width, egui::FontFamily::Proportional),
            stroke.color,
        );
    }

    pub(crate) fn from_geo(
        point: geo::Point,
        text: String,
        ref_point: Coord,
        crs: Option<u16>,
    ) -> Result<Self> {
        let pos = if let Some(epsg) = crs {
            let geo_proj = Proj::from_epsg_code(4326)?;
            let local_proj = Proj::from_epsg_code(epsg)?;

            let mut p = (point.x() + ref_point.x, point.y() + ref_point.y);
            let _ = transform(&local_proj, &geo_proj, &mut p);

            walkers::pos_from_lon_lat(p.0.to_degrees(), p.1.to_degrees())
        } else {
            walkers::pos_from_lon_lat(point.x() + ref_point.x, point.y() + ref_point.y)
        };

        Ok(DrawableTextObject(pos, text))
    }
}
