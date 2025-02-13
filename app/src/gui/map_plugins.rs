use eframe::egui::{self, Color32, Response, Ui};
use geo::{LineString, Polygon, TriangulateEarcut};
use walkers::{Plugin, Position, Projector};

use super::ProcessStage;
use laz2omap::DrawableOmap;

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
    fn run(self: Box<Self>, ui: &mut Ui, _response: &Response, projector: &Projector) {
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
    selected: Option<usize>,
    hover: bool,
    neighbour_map: Option<&'a Vec<[Option<usize>; 9]>>,
}

impl<'a> LasBoundaryPainter<'a> {
    pub fn new(
        b: &'a Vec<[Position; 4]>,
        si: Option<usize>,
        hover: bool,
        neighbour_map: Option<&'a Vec<[Option<usize>; 9]>>,
    ) -> LasBoundaryPainter<'a> {
        LasBoundaryPainter {
            boundaries: b,
            selected: si,
            hover,
            neighbour_map,
        }
    }
}

impl Plugin for LasBoundaryPainter<'_> {
    fn run(self: Box<Self>, ui: &mut Ui, response: &Response, projector: &Projector) {
        let hover = if self.hover {
            response.hover_pos()
        } else {
            None
        };

        let mut ni: Option<Vec<&usize>> = None;
        if let Some(neighbour_map) = self.neighbour_map {
            if let Some(i) = self.selected {
                ni = Some(neighbour_map[i].iter().skip(1).flatten().collect());
            }
        }

        for (i, bound) in self.boundaries.iter().enumerate() {
            // painting is most performant in clockwise order
            let screen_coords = [
                projector.project(bound[0]),
                projector.project(bound[3]),
                projector.project(bound[2]),
                projector.project(bound[1]),
            ];

            let fill = if let Some(index) = self.selected {
                if i == index {
                    Color32::RED.gamma_multiply(0.5)
                } else if let Some(neighbours) = &ni {
                    let mut c = Color32::RED.gamma_multiply(0.2);
                    for j in neighbours {
                        if i == **j {
                            c = Color32::RED.gamma_multiply(0.35);
                            break;
                        }
                    }
                    c
                } else {
                    Color32::RED.gamma_multiply(0.2)
                }
            } else if let Some(pos) = hover {
                if screen_rectangle_contains(&screen_coords, &pos) {
                    Color32::RED.gamma_multiply(0.5)
                } else {
                    Color32::RED.gamma_multiply(0.2)
                }
            } else {
                Color32::RED.gamma_multiply(0.2)
            };
            ui.painter().add(egui::Shape::convex_polygon(
                screen_coords.to_vec(),
                fill,
                egui::Stroke::new(2., Color32::RED),
            ));
        }
    }
}

pub struct PolygonDrawer<'a> {
    state: &'a mut ProcessStage,
    area_of_interest: &'a mut LineString,
}

impl<'a> PolygonDrawer<'a> {
    pub fn new(area_of_interest: &'a mut LineString, state: &'a mut ProcessStage) -> Self {
        PolygonDrawer {
            state,
            area_of_interest,
        }
    }
}

impl Plugin for PolygonDrawer<'_> {
    fn run(self: Box<Self>, ui: &mut Ui, response: &Response, projector: &Projector) {
        // register clicks
        if *self.state == ProcessStage::DrawPolygon && !response.changed() {
            if response.double_clicked() {
                if self.area_of_interest.0.len() < 3 {
                    self.area_of_interest.0.clear();
                } else {
                    self.area_of_interest.close();
                }

                // TODO!
                // check for self intersections, if so clear the area_of_interest
                // validation trait is added to the next release of geo
                // let valid_check = Polygon::new(self.area_of_interest.clone(), vec![]);
                // if !valid_check.is_valid() {
                //   self.area_of_interest.0.clear();
                // }

                *self.state = ProcessStage::ChooseSquare;
            } else if response.clicked_by(egui::PointerButton::Primary) {
                let clicked_pos = response
                    .interact_pointer_pos()
                    .map(|p| projector.unproject(p));

                if let Some(cp) = clicked_pos {
                    self.area_of_interest.0.push(cp);
                }
            }
        }

        // draw the polygon
        if !self.area_of_interest.0.is_empty() {
            let mut line = self.area_of_interest.clone();
            if *self.state == ProcessStage::DrawPolygon && response.hovered() {
                if let Some(pos) = response.hover_pos() {
                    line.0.push(projector.unproject(pos));
                }
            }
            line.close();

            let poly = Polygon::new(line, vec![]);

            if poly.exterior().0.len() > 2 {
                // does the triangulation WGS84 coordinates, but the earth is locally almost flat so it's almost ok
                let tri = poly.earcut_triangles_raw();

                let points: Vec<egui::epaint::Vertex> = tri
                    .vertices
                    .chunks(2)
                    .map(|c| egui::epaint::Vertex {
                        pos: projector.project(geo::Coord { x: c[0], y: c[1] }),
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

            let mut outline: Vec<egui::Pos2> = self
                .area_of_interest
                .coords()
                .map(|p| projector.project(*p))
                .collect();
            if *self.state == ProcessStage::DrawPolygon && response.hovered() {
                if let Some(pos) = response.hover_pos() {
                    outline.push(pos);
                }
            }

            ui.painter()
                .line(outline, egui::Stroke::new(2., egui::Color32::ORANGE));
        }
    }
}

pub struct ClickListener<'a> {
    pub boundaries: &'a Vec<[Position; 4]>,
    pub selected_file: &'a mut Option<usize>,
}

impl<'a> ClickListener<'a> {
    pub fn new(boundaries: &'a Vec<[Position; 4]>, selected_file: &'a mut Option<usize>) -> Self {
        ClickListener {
            boundaries,
            selected_file,
        }
    }
}

impl Plugin for ClickListener<'_> {
    fn run(self: Box<Self>, _ui: &mut Ui, response: &Response, projector: &Projector) {
        if !response.changed() && response.clicked_by(egui::PointerButton::Primary) {
            let clicked_pos = response
                .interact_pointer_pos()
                .map(|p| projector.unproject(p));

            if let Some(cp) = clicked_pos {
                for (i, bound) in self.boundaries.iter().enumerate() {
                    if rectangle_contains(bound, &cp) {
                        if *self.selected_file == Some(i) {
                            *self.selected_file = None;
                        } else {
                            *self.selected_file = Some(i);
                        }
                        break;
                    }
                }
            }
        }
    }
}

fn rectangle_contains(b: &[Position; 4], p: &Position) -> bool {
    b[0].y >= p.y && b[0].x <= p.x && b[2].y <= p.y && b[2].x >= p.x
}

fn screen_rectangle_contains(b: &[egui::Pos2; 4], p: &egui::Pos2) -> bool {
    b[0].y <= p.y && b[0].x <= p.x && b[2].y >= p.y && b[2].x >= p.x
}

pub struct OmapDrawer<'a> {
    map: &'a Option<DrawableOmap>,
}

impl<'a> OmapDrawer<'a> {
    pub fn new(map: &'a Option<DrawableOmap>) -> Self {
        Self { map }
    }
}

impl Plugin for OmapDrawer<'_> {
    fn run(self: Box<Self>, ui: &mut Ui, _response: &Response, projector: &Projector) {
        if let Some(map) = self.map.as_ref() {
            map.draw(ui, projector);
        }
    }
}
