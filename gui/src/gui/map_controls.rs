use eframe::egui;
use walkers::{sources::Attribution, MapMemory, Position};

use super::ProcessStage;

pub fn render_zoom(ui: &mut egui::Ui, map_memory: &mut MapMemory) {
    egui::Window::new("Zoom")
        .collapsible(false)
        .resizable(false)
        .title_bar(false)
        .anchor(egui::Align2::RIGHT_TOP, [-20., 20.])
        .show(ui.ctx(), |ui| {
            ui.vertical(|ui| {
                if ui
                    .add_enabled(
                        map_memory.zoom() < 21.,
                        egui::Button::new(egui::RichText::new("+").size(30.).strong().monospace()),
                    )
                    .clicked()
                {
                    let _ = map_memory.zoom_in();
                }

                if ui
                    .add_enabled(
                        map_memory.zoom() > 3.,
                        egui::Button::new(egui::RichText::new("-").size(30.).strong().monospace()),
                    )
                    .clicked()
                {
                    let _ = map_memory.zoom_out();
                }
            });
        });
}

pub fn render_home(ui: &mut egui::Ui, map_memory: &mut MapMemory, home_zoom: f64) {
    egui::Window::new("Home")
        .collapsible(false)
        .resizable(false)
        .title_bar(false)
        .anchor(egui::Align2::RIGHT_TOP, [-65., 20.])
        .show(ui.ctx(), |ui| {
            if ui
                .button(egui::RichText::new("⛶").size(28.).strong().monospace())
                .on_hover_text("Reset zoom and pan")
                .clicked()
            {
                map_memory.follow_my_position();
                map_memory.set_zoom(home_zoom).unwrap();
            }
        });
}

pub fn render_draw_button(
    ui: &mut egui::Ui,
    active: bool,
    rect: egui::Rect,
    polygon: &mut Vec<Position>,
    state: &mut ProcessStage,
) {
    egui::Window::new("Draw Polygon")
        .collapsible(false)
        .resizable(false)
        .title_bar(false)
        .anchor(egui::Align2::LEFT_TOP, [rect.min.x + 10., 20.])
        .enabled(active)
        .show(ui.ctx(), |ui| {
            if active {
                if !polygon.is_empty() {
                    if ui.button("Clear polygon").clicked() {
                        polygon.clear();
                    };
                } else if ui.button("Draw polygon").clicked() {
                    *state = ProcessStage::DrawPolygon;
                }
            } else {
                let _ = ui.button("Double click to end polygon");
            }
        });
}

pub fn render_scale_pos_label(ui: &mut egui::Ui, map_memory: &MapMemory, my_pos: Position) {
    // Pos and zoom labels
    let position = match map_memory.detached() {
        None => my_pos,
        Some(p) => p,
    };

    egui::Window::new("Pos and Zoom label")
        .collapsible(false)
        .resizable(false)
        .title_bar(false)
        .anchor(egui::Align2::RIGHT_BOTTOM, [-10., -10.])
        .show(ui.ctx(), |ui| {
            const POINT_SIZE: f64 = 0.0001622; // for my monitor

            // how many meters a single point covers on the map
            let m_per_point = ui.ctx().pixels_per_point()
                / map_memory.scale_pixel_per_meter(position);

            let scale = m_per_point as f64 / POINT_SIZE;

            ui.horizontal(|ui| {
                    ui.label(format!("Scale*: 1:{:.0}", scale))
                    .on_hover_text("*The scale is an approximation based on the UI's scale factor.\nMight be inaccurate for some devices")
                    .on_hover_cursor(egui::CursorIcon::Alias);
                    if map_memory.is_global() {
                        ui.label(format!(
                            "Map position: {}{:.4}, {}{:.4}",
                            if position.lat() >= 0. { 'N' } else { 'S' },
                            position.lat().abs(),
                            if position.lon() >= 0. { 'E' } else { 'W' },
                            position.lon().abs()
                        ))
                        .on_hover_cursor(egui::CursorIcon::Alias);
                    } else {
                        ui.label(format!(
                            "Map position: {:.4}, {:.4}",
                            position.x(),
                            position.y()
                        ))
                        .on_hover_cursor(egui::CursorIcon::Alias);
                    }
                });
            });
}

pub fn render_acknowledge(ui: &egui::Ui, attribution: Attribution, rect: egui::Rect) {
    egui::Window::new("Acknowledge")
        .frame(egui::Frame::none().fill(egui::Color32::from_rgba_unmultiplied(50, 50, 50, 200)))
        .collapsible(false)
        .resizable(false)
        .title_bar(false)
        .anchor(egui::Align2::LEFT_BOTTOM, [rect.min.x + 10., -10.])
        .show(ui.ctx(), |ui| {
            ui.hyperlink_to(attribution.text, attribution.url)
                .on_hover_text(attribution.url);
        });
}
