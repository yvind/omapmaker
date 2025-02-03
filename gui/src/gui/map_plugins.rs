use eframe::egui::{self, Color32, Response, Ui};
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
}

impl<'a> LasBoundaryPainter<'a> {
    pub fn new(
        b: &'a Vec<[Position; 4]>,
        si: Option<usize>,
        hover: bool,
    ) -> LasBoundaryPainter<'a> {
        LasBoundaryPainter {
            boundaries: b,
            selected: si,
            hover,
        }
    }
}

impl Plugin for LasBoundaryPainter<'_> {
    fn run(self: Box<Self>, ui: &mut Ui, response: &Response, projector: &Projector) {
        for (i, bound) in self.boundaries.iter().enumerate() {
            // Project it into the position on the screen.
            // screen coords are positive down

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
                } else {
                    Color32::RED.gamma_multiply(0.2)
                }
            } else if self.hover {
                match response.hover_pos() {
                    None => Color32::RED.gamma_multiply(0.2),
                    Some(pos) => {
                        if screen_rectangle_contains(&screen_coords, &pos) {
                            Color32::RED.gamma_multiply(0.5)
                        } else {
                            Color32::RED.gamma_multiply(0.2)
                        }
                    }
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
    area_of_interest: &'a mut Vec<Position>,
}

impl<'a> PolygonDrawer<'a> {
    pub fn new(area_of_interest: &'a mut Vec<Position>, state: &'a mut ProcessStage) -> Self {
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
                if self.area_of_interest.len() < 3 {
                    self.area_of_interest.clear();
                } else {
                    self.area_of_interest.push(self.area_of_interest[0]);
                }

                // check for self intersections, if so clear the area_of_interest
                // validation trait is added to the next release of geo

                *self.state = ProcessStage::ChooseSquare;
            } else if response.clicked_by(egui::PointerButton::Primary) {
                let clicked_pos = response
                    .interact_pointer_pos()
                    .map(|p| projector.unproject(p));

                if let Some(cp) = clicked_pos {
                    self.area_of_interest.push(cp);
                }
            }
        }

        // draw the polygon
        if !self.area_of_interest.is_empty() {
            let mut points = Vec::with_capacity(self.area_of_interest.len());
            for pos in self.area_of_interest.iter() {
                points.push(projector.project(*pos));
            }

            if *self.state == ProcessStage::DrawPolygon && response.hovered() {
                if let Some(pos) = response.hover_pos() {
                    points.push(pos);
                }
            }

            ui.painter().add(egui::Shape::convex_polygon(
                points.clone(),
                egui::Color32::ORANGE.gamma_multiply(0.2),
                egui::Stroke::NONE,
            ));
            ui.painter()
                .line(points, egui::Stroke::new(2., egui::Color32::ORANGE));
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
    map: &'a Option<Box<DrawableOmap>>,
}

impl<'a> OmapDrawer<'a> {
    pub fn new(map: &'a Option<Box<DrawableOmap>>) -> Self {
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
