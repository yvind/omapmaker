use crate::Result;
use eframe::{
    egui::{self, Color32, Stroke},
    epaint::CubicBezierShape,
};
use geo::{Coord, LineString, TriangulateEarcut};
use linestring2bezier::BezierString;
use proj_core::{CrsDef, Transform};

#[derive(Clone)]
pub enum DrawableGeometry {
    Polygon(DrawablePolygonObject),
    Line(DrawableLineObject),
    Point(DrawablePointObject),
}

impl DrawableGeometry {
    pub(crate) fn draw(
        &self,
        ui: &mut egui::Ui,
        projector: &walkers::ScreenProjector,
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
        }
    }
}

#[derive(Clone)]
pub struct DrawablePolygonObject(Triangulation);

impl DrawablePolygonObject {
    pub(crate) fn draw(
        &self,
        ui: &mut egui::Ui,
        projector: &walkers::ScreenProjector,
        stroke: &Stroke,
        special: bool,
    ) {
        self.0.draw(ui, projector, stroke, special);
    }

    pub(crate) fn from_geo(
        poly: geo::Polygon,
        ref_point: Coord,
        crs: Option<CrsDef>,
        _bezier_error: Option<f64>,
    ) -> Result<Self> {
        let tri = poly.earcut_triangles_raw();

        let vertices: Vec<(f64, f64)> = tri
            .vertices
            .into_iter()
            .map(|c| (c[0] + ref_point.x, c[1] + ref_point.y))
            .collect();
        let obj = if let Some(epsg) = crs {
            let transform = Transform::from_epsg(epsg as u32, 4326).unwrap();

            let transformed_points = transform.convert_batch(&vertices).unwrap();
            transformed_points
                .iter()
                .map(|c| walkers::lon_lat(c.0, c.1))
                .collect()
        } else {
            vertices
                .iter()
                .map(|c| walkers::lon_lat(c.0, c.1))
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
        projector: &walkers::ScreenProjector,
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
        projector: &walkers::ScreenProjector,
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
        crs: Option<CrsDef>,
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
            let transform = Transform::from_epsg(epsg as u32, 4326).unwrap();

            let line: Vec<(f64, f64)> = line
                .0
                .iter()
                .map(|c| (c.x + ref_point.x, c.y + ref_point.y))
                .collect();

            let transformed_line = transform.convert_batch(&line).unwrap();

            transformed_line
                .iter()
                .map(|c| walkers::lon_lat(c.0, c.1))
                .collect()
        } else {
            line.coords()
                .map(|c| walkers::lon_lat(c.x + ref_point.x, c.y + ref_point.y))
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
        projector: &walkers::ScreenProjector,
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
        crs: Option<CrsDef>,
    ) -> Result<Self> {
        let pos = if let Some(epsg) = crs {
            let transform = Transform::from_epsg(epsg as u32, 4326).unwrap();

            let p = (point.x() + ref_point.x, point.y() + ref_point.y);
            let transformed_p = transform.convert(p).unwrap();

            walkers::lon_lat(transformed_p.0, transformed_p.1)
        } else {
            walkers::lon_lat(point.x() + ref_point.x, point.y() + ref_point.y)
        };

        Ok(DrawablePointObject(pos, rot as f32))
    }
}
