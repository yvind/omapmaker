use std::collections::HashMap;

use eframe::egui::{self, Color32, Response, Ui};
use geo::{Area, BooleanOps, Contains, TriangulateEarcut, Validation};
use proj_core::{CrsDef, Transform};
use walkers::{Plugin, Position, ScreenProjector};

use crate::{drawable::DrawableOmap, map_gen::egui_map::Symbol};

const COLOR_LIST: [egui::Color32; 9] = [
    egui::Color32::ORANGE,
    egui::Color32::BLUE,
    egui::Color32::BLACK,
    egui::Color32::RED,
    egui::Color32::YELLOW,
    egui::Color32::GREEN,
    egui::Color32::BROWN,
    egui::Color32::WHITE,
    egui::Color32::GOLD,
];

pub struct LasComponentPainter<'a> {
    boundaries: &'a Vec<[Position; 4]>,
    components: &'a Vec<Vec<usize>>,
}

impl<'a> LasComponentPainter<'a> {
    pub fn new(b: &'a Vec<[Position; 4]>, c: &'a Vec<Vec<usize>>) -> LasComponentPainter<'a> {
        LasComponentPainter {
            boundaries: b,
            components: c,
        }
    }
}

impl Plugin for LasComponentPainter<'_> {
    fn run(self: Box<Self>, ui: &mut Ui, _response: &Response, projector: &ScreenProjector) {
        for (ci, component) in self.components.iter().enumerate() {
            let component_color = COLOR_LIST[ci];
            for boundary_index in component {
                let bound = &self.boundaries[*boundary_index];

                // painting is most performant in clockwise order
                let screen_coords = [
                    projector.project(bound[0]),
                    projector.project(bound[3]),
                    projector.project(bound[2]),
                    projector.project(bound[1]),
                ];

                ui.painter().add(egui::Shape::convex_polygon(
                    screen_coords.to_vec(),
                    component_color.gamma_multiply(0.2),
                    egui::Stroke::new(2., component_color),
                ));
            }
        }
    }
}

pub struct LasBoundaryPainter<'a> {
    boundaries: &'a Vec<[Position; 4]>,
}

impl<'a> LasBoundaryPainter<'a> {
    pub fn new(b: &'a Vec<[Position; 4]>) -> LasBoundaryPainter<'a> {
        LasBoundaryPainter { boundaries: b }
    }
}

impl Plugin for LasBoundaryPainter<'_> {
    fn run(self: Box<Self>, ui: &mut Ui, _response: &Response, projector: &ScreenProjector) {
        for bound in self.boundaries.iter() {
            // painting is most performant in clockwise order
            let screen_coords = [
                projector.project(bound[0]),
                projector.project(bound[3]),
                projector.project(bound[2]),
                projector.project(bound[1]),
            ];

            let fill = Color32::RED.gamma_multiply(0.2);
            ui.painter().add(egui::Shape::convex_polygon(
                screen_coords.to_vec(),
                fill,
                egui::Stroke::new(2., Color32::RED),
            ));
        }
    }
}

pub struct PolygonDrawer<'a> {
    drawing_enabled: Option<&'a mut bool>,
    area_of_interest: &'a mut geo::LineString,
}

impl<'a> PolygonDrawer<'a> {
    pub fn new(area_of_interest: &'a mut geo::LineString, drawing_enabled: &'a mut bool) -> Self {
        PolygonDrawer {
            drawing_enabled: Some(drawing_enabled),
            area_of_interest,
        }
    }

    pub fn readonly(area_of_interest: &'a mut geo::LineString) -> Self {
        PolygonDrawer {
            drawing_enabled: None,
            area_of_interest,
        }
    }
}

impl Plugin for PolygonDrawer<'_> {
    fn run(self: Box<Self>, ui: &mut Ui, response: &Response, projector: &ScreenProjector) {
        let PolygonDrawer {
            drawing_enabled,
            area_of_interest,
        } = *self;
        let drawing = drawing_enabled.as_deref().copied().unwrap_or(false);
        let closed = area_of_interest.is_closed() && area_of_interest.0.len() > 3;

        // register clicks
        if drawing && !closed && !response.changed() {
            if response.double_clicked() {
                if area_of_interest.0.len() < 3 {
                    area_of_interest.0.clear();
                } else {
                    area_of_interest.close();
                }

                let valid_check = geo::Polygon::new(area_of_interest.clone(), vec![]);
                if !valid_check.is_valid() {
                    area_of_interest.0.clear();
                }

                if let Some(enabled) = drawing_enabled {
                    *enabled = false;
                }
            } else if response.clicked_by(egui::PointerButton::Primary) {
                let clicked_pos = response
                    .interact_pointer_pos()
                    .map(|p| projector.unproject(p));

                if let Some(cp) = clicked_pos {
                    area_of_interest.0.push(cp.0);
                }
            }
        }

        // draw the polygon
        if !area_of_interest.0.is_empty() {
            let mut line = area_of_interest.clone();
            if drawing
                && !closed
                && response.hovered()
                && let Some(pos) = response.hover_pos()
            {
                line.0.push(projector.unproject(pos).0);
            }
            line.close();

            let poly = geo::Polygon::new(line, vec![]);

            if poly.exterior().0.len() > 3 {
                // does the triangulation WGS84 coordinates, but the earth is locally almost flat so it's almost ok
                let tri = poly.earcut_triangles_raw();

                let points: Vec<egui::epaint::Vertex> = tri
                    .vertices
                    .into_iter()
                    .map(|c| egui::epaint::Vertex {
                        pos: projector.project(geo::Point(geo::Coord { x: c[0], y: c[1] })),
                        uv: egui::epaint::WHITE_UV,
                        color: egui::Color32::ORANGE.gamma_multiply(0.5),
                    })
                    .collect();

                let mesh = egui::Mesh {
                    indices: tri.triangle_indices.into_iter().map(|i| i as u32).collect(),
                    vertices: points,
                    texture_id: egui::epaint::TextureId::Managed(0),
                };

                ui.painter().add(mesh);
            }

            let mut outline: Vec<egui::Pos2> = area_of_interest
                .coords()
                .map(|p| projector.project(geo::Point(*p)))
                .collect();
            if drawing
                && !closed
                && response.hovered()
                && let Some(pos) = response.hover_pos()
            {
                outline.push(pos);
            }

            ui.painter()
                .line(outline, egui::Stroke::new(2., egui::Color32::ORANGE));
        }
    }
}

pub struct TestAreaSelector<'a> {
    test_area_display: &'a geo::MultiPolygon,
    test_area_projected: &'a geo::MultiPolygon,
    selected_square: &'a mut Option<geo::Rect>,
    selected_square_boundary: &'a mut Option<[Position; 4]>,
    crs: Option<&'a CrsDef>,
}

impl<'a> TestAreaSelector<'a> {
    pub fn new(
        test_area_display: &'a geo::MultiPolygon,
        test_area_projected: &'a geo::MultiPolygon,
        selected_square: &'a mut Option<geo::Rect>,
        selected_square_boundary: &'a mut Option<[Position; 4]>,
        crs: Option<&'a CrsDef>,
    ) -> Self {
        Self {
            test_area_display,
            test_area_projected,
            selected_square,
            selected_square_boundary,
            crs,
        }
    }
}

impl Plugin for TestAreaSelector<'_> {
    fn run(self: Box<Self>, ui: &mut Ui, response: &Response, projector: &ScreenProjector) {
        if !response.changed()
            && response.clicked_by(egui::PointerButton::Primary)
            && let Some(clicked_pos) = response.interact_pointer_pos()
        {
            let clicked_coord = projector.unproject(clicked_pos).0;
            match display_to_projected_coord(self.crs, clicked_coord)
                .and_then(|center| selected_test_square(center, self.test_area_projected))
                .and_then(|rect| Ok((rect, rect_to_display_boundary(self.crs, &rect)?)))
            {
                Ok((rect, boundary)) => {
                    *self.selected_square = Some(rect);
                    *self.selected_square_boundary = Some(boundary);
                }
                Err(_) => {
                    *self.selected_square = None;
                    *self.selected_square_boundary = None;
                }
            }
        }

        for polygon in &self.test_area_display.0 {
            draw_polygon(
                ui,
                projector,
                polygon,
                Color32::RED.gamma_multiply(0.25),
                egui::Stroke::new(2., Color32::RED),
            );
        }

        if let Some(boundary) = self.selected_square_boundary {
            let points = [
                projector.project(boundary[0]),
                projector.project(boundary[1]),
                projector.project(boundary[2]),
                projector.project(boundary[3]),
                projector.project(boundary[0]),
            ];
            ui.painter()
                .line(points.to_vec(), egui::Stroke::new(3., Color32::ORANGE));
        } else if let Some(hover) = response.hover_pos() {
            let hover_pos = projector.unproject(hover);
            if self.test_area_display.contains(&hover_pos.0) {
                let hover_rect = display_to_projected_coord(self.crs, hover_pos.0)
                    .map(|c| {
                        geo::Rect::new(
                            c - geo::Coord {
                                x: crate::ADJUSTMENT_TILE_SIZE_METERS / 2.,
                                y: crate::ADJUSTMENT_TILE_SIZE_METERS / 2.,
                            },
                            c + geo::Coord {
                                x: crate::ADJUSTMENT_TILE_SIZE_METERS / 2.,
                                y: crate::ADJUSTMENT_TILE_SIZE_METERS / 2.,
                            },
                        )
                    })
                    .and_then(|rect| rect_to_display_boundary(self.crs, &rect));
                if let Ok(hover_rect) = hover_rect {
                    let points = [
                        projector.project(hover_rect[0]),
                        projector.project(hover_rect[1]),
                        projector.project(hover_rect[2]),
                        projector.project(hover_rect[3]),
                        projector.project(hover_rect[0]),
                    ];
                    ui.painter()
                        .line(points.to_vec(), egui::Stroke::new(3., Color32::ORANGE));
                }
            }
        }
    }
}

fn selected_test_square(
    center: geo::Coord,
    test_area_projected: &geo::MultiPolygon,
) -> crate::Result<geo::Rect> {
    let rect = geo::Rect::new(
        center
            - geo::Coord {
                x: crate::ADJUSTMENT_TILE_SIZE_METERS / 2.,
                y: crate::ADJUSTMENT_TILE_SIZE_METERS / 2.,
            },
        center
            + geo::Coord {
                x: crate::ADJUSTMENT_TILE_SIZE_METERS / 2.,
                y: crate::ADJUSTMENT_TILE_SIZE_METERS / 2.,
            },
    );

    let overlap = test_area_projected.intersection(&rect.to_polygon());
    if overlap.unsigned_area() < rect.unsigned_area() * 0.5 {
        anyhow::bail!("Selected test square overlaps the lidar area by less than 50%");
    }

    Ok(rect)
}

fn display_to_projected_coord(
    crs: Option<&CrsDef>,
    coord: geo::Coord,
) -> crate::Result<geo::Coord> {
    let Some(crs) = crs else {
        return Ok(coord);
    };

    let global = proj_wkt::parse_crs("4326")?;
    let transform = Transform::from_crs_defs(&global, crs)?;
    transform
        .convert_geometry(coord)
        .map_err(|e| anyhow::anyhow!(e))
}

fn rect_to_display_boundary(
    crs: Option<&CrsDef>,
    rect: &geo::Rect,
) -> crate::Result<[Position; 4]> {
    let mut corners = [
        geo::Coord {
            x: rect.min().x,
            y: rect.max().y,
        },
        rect.min(),
        geo::Coord {
            x: rect.max().x,
            y: rect.min().y,
        },
        rect.max(),
    ];

    if let Some(crs) = crs {
        let transform = Transform::from_epsg(crs.epsg(), 4326)?;
        for corner in &mut corners {
            let transformed = transform.convert((corner.x, corner.y))?;
            corner.x = transformed.0;
            corner.y = transformed.1;
        }
    }

    Ok(corners.map(geo::Point))
}

fn draw_polygon(
    ui: &mut Ui,
    projector: &ScreenProjector,
    polygon: &geo::Polygon,
    fill: Color32,
    stroke: egui::Stroke,
) {
    if polygon.exterior().0.len() > 3 {
        let tri = polygon.earcut_triangles_raw();

        let points: Vec<egui::epaint::Vertex> = tri
            .vertices
            .into_iter()
            .map(|c| egui::epaint::Vertex {
                pos: projector.project(geo::Point(geo::Coord { x: c[0], y: c[1] })),
                uv: egui::epaint::WHITE_UV,
                color: fill,
            })
            .collect();

        let mesh = egui::Mesh {
            indices: tri.triangle_indices.into_iter().map(|i| i as u32).collect(),
            vertices: points,
            texture_id: egui::epaint::TextureId::Managed(0),
        };

        ui.painter().add(mesh);
    }

    let outline: Vec<egui::Pos2> = polygon
        .exterior()
        .coords()
        .map(|p| projector.project(geo::Point(*p)))
        .collect();
    ui.painter().line(outline, stroke);

    for interior in polygon.interiors() {
        let outline: Vec<egui::Pos2> = interior
            .coords()
            .map(|p| projector.project(geo::Point(*p)))
            .collect();
        ui.painter().line(outline, stroke);
    }
}

pub struct OmapDrawer<'a> {
    map: &'a Option<DrawableOmap>,
    visibilities: &'a HashMap<Symbol, bool>,
    opacity: f32,
}

impl<'a> OmapDrawer<'a> {
    pub fn new(
        map: &'a Option<DrawableOmap>,
        visibilities: &'a HashMap<Symbol, bool>,
        opacity: f32,
    ) -> Self {
        Self {
            map,
            visibilities,
            opacity,
        }
    }
}

impl Plugin for OmapDrawer<'_> {
    fn run(self: Box<Self>, ui: &mut Ui, _response: &Response, projector: &ScreenProjector) {
        if let Some(map) = self.map.as_ref() {
            map.draw(ui, projector, self.visibilities, self.opacity);
        }
    }
}
