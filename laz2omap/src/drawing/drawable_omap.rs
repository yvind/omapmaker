use eframe::egui;

#[derive(Clone)]
pub struct DrawableOmap(Vec<MapSymbol>);

impl DrawableOmap {
    pub fn draw(&self, ui: &mut egui::Ui, projector: &walkers::Projector) {
        // order is determined by the order in the vec
        // should be in reverse order of ISOM color appendix,
        // ie yellow first and so on
        for ms in self.0.iter() {
            ms.draw(ui, projector);
        }
    }
}

#[derive(Clone)]
pub struct MapSymbol(GeometryType);

impl MapSymbol {
    fn draw(&self, ui: &mut egui::Ui, projector: &walkers::Projector) {
        match &self.0 {
            GeometryType::Polygon(color, polygon_objects) => {
                for poly in polygon_objects.0.iter() {
                    poly.draw(ui, projector, color);
                }
            }
            GeometryType::Line(stroke, line_objects) => {
                line_objects.draw(ui, projector, stroke);
            }
            GeometryType::Point(stroke, point_objects) => {
                point_objects.draw(ui, projector, stroke);
            }
        }
    }
}

#[derive(Clone)]
pub enum GeometryType {
    Polygon(egui::Color32, PolygonObjects),
    Line(egui::Stroke, LineObjects),
    Point(egui::Stroke, PointObjects),
}

#[derive(Clone)]
pub struct PolygonObjects(Vec<Triangulation>);

#[derive(Clone)]
pub struct Triangulation(Vec<[walkers::Position; 3]>);

impl Triangulation {
    fn draw(&self, ui: &mut egui::Ui, projector: &walkers::Projector, color: &egui::Color32) {
        let painter = ui.painter();

        for tri in self.0.iter() {
            let points = tri.map(|p| projector.project(p)).to_vec();

            painter.add(egui::Shape::convex_polygon(
                points,
                *color,
                egui::Stroke::NONE,
            ));
        }
    }
}
#[derive(Clone)]
pub struct LineObjects(Vec<Vec<walkers::Position>>);

impl LineObjects {
    fn draw(&self, ui: &mut egui::Ui, projector: &walkers::Projector, stroke: &egui::Stroke) {
        let painter = ui.painter();

        for line_string in self.0.iter() {
            let screen_line = line_string.iter().map(|p| projector.project(*p)).collect();

            painter.line(screen_line, *stroke);
        }
    }
}

#[derive(Clone)]
pub struct PointObjects(Vec<walkers::Position>);

impl PointObjects {
    fn draw(&self, ui: &mut egui::Ui, projector: &walkers::Projector, stroke: &egui::Stroke) {
        let painter = ui.painter();
        for point in self.0.iter() {
            let screen_point = projector.project(*point);

            painter.circle_filled(screen_point, stroke.width, stroke.color);
        }
    }
}
