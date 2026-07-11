use std::collections::HashMap;

use eframe::egui;
use walkers::{MapMemory, Projection, sources::Attribution};

use super::gui_variables::TileProvider;
use crate::{
    drawable::DrawableOmap,
    map_gen::egui_map::{AreaSymbol, Symbol},
};

struct ScaleBar {
    width: f32,
    label: String,
}

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
                let _ = map_memory.set_zoom(home_zoom);
            }
        });
}

pub fn render_scale_pos_label(
    ui: &mut egui::Ui,
    map_memory: &MapMemory,
    my_pos: walkers::Position,
    projection: &dyn Projection,
) {
    // Pos and zoom labels
    let position = match map_memory.detached(projection) {
        None => my_pos,
        Some(p) => p,
    };

    egui::Window::new("Pos and Zoom label")
        .collapsible(false)
        .resizable(false)
        .title_bar(false)
        .anchor(egui::Align2::RIGHT_BOTTOM, [-10., -10.])
        .max_width(250.)
        .show(ui.ctx(), |ui| {
            let scale_bar =
                scale_bar(projection.scale_pixel_per_meter(position, map_memory.zoom()) as f64);

            ui.vertical_centered(|ui| {
                if let Some(scale_bar) = scale_bar {
                    render_scale_bar(ui, scale_bar)
                        .on_hover_text("Scale bar based on the current map position and zoom")
                        .on_hover_cursor(egui::CursorIcon::Alias);
                }

                if projection.is_mercator() {
                    ui.label(format!(
                        "Map position: {}{:.4}, {}{:.4}",
                        if position.y() >= 0. { 'N' } else { 'S' },
                        position.y().abs(),
                        if position.x() >= 0. { 'E' } else { 'W' },
                        position.x().abs()
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

fn scale_bar(pixels_per_meter: f64) -> Option<ScaleBar> {
    const MAX_WIDTH: f64 = 160.0;

    if !pixels_per_meter.is_finite() || pixels_per_meter <= 0.0 {
        return None;
    }

    let distance_meters = nice_distance(MAX_WIDTH / pixels_per_meter)?;
    Some(ScaleBar {
        width: (distance_meters * pixels_per_meter) as f32,
        label: format_distance(distance_meters),
    })
}

fn nice_distance(max_distance_meters: f64) -> Option<f64> {
    if !max_distance_meters.is_finite() || max_distance_meters <= 0.0 {
        return None;
    }

    let base = 10f64.powf(max_distance_meters.log10().floor());
    let normalized = max_distance_meters / base;
    let multiplier = if normalized >= 5.0 {
        5.0
    } else if normalized >= 2.0 {
        2.0
    } else {
        1.0
    };

    Some(multiplier * base)
}

fn format_distance(meters: f64) -> String {
    if meters >= 1000.0 {
        let km = meters / 1000.0;
        format!("{km:.0} km")
    } else if meters >= 1.0 {
        format!("{meters:.0} m")
    } else if meters >= 0.01 {
        format!("{:.0} cm", meters * 100.0)
    } else {
        format!("{:.0} mm", meters * 1000.0)
    }
}

fn render_scale_bar(ui: &mut egui::Ui, scale_bar: ScaleBar) -> egui::Response {
    const HEIGHT: f32 = 22.0;
    const MIN_WIDTH: f32 = 64.0;
    const TICK_HEIGHT: f32 = 6.0;

    let desired_size = egui::vec2(scale_bar.width.max(MIN_WIDTH), HEIGHT);
    let (rect, response) = ui.allocate_exact_size(desired_size, egui::Sense::hover());
    let painter = ui.painter_at(rect);
    let color = ui.visuals().text_color();
    let stroke = egui::Stroke::new(2.0, color);
    let bar_rect = egui::Rect::from_min_max(
        egui::pos2(rect.center().x - scale_bar.width / 2.0, rect.bottom() - 6.0),
        egui::pos2(rect.center().x + scale_bar.width / 2.0, rect.bottom() - 6.0),
    );

    painter.text(
        rect.center_top(),
        egui::Align2::CENTER_TOP,
        scale_bar.label,
        egui::TextStyle::Body.resolve(ui.style()),
        color,
    );
    painter.line_segment([bar_rect.left_top(), bar_rect.right_top()], stroke);
    painter.line_segment(
        [
            bar_rect.left_top(),
            bar_rect.left_top() - egui::vec2(0.0, TICK_HEIGHT),
        ],
        stroke,
    );
    painter.line_segment(
        [
            bar_rect.right_top(),
            bar_rect.right_top() - egui::vec2(0.0, TICK_HEIGHT),
        ],
        stroke,
    );

    response
}

pub fn render_symbol_toggles(
    ui: &mut egui::Ui,
    map_tile: &Option<DrawableOmap>,
    checkboxes: &mut HashMap<Symbol, bool>,
) {
    // add a window for toggling visibilities
    egui::Window::new("Symbol Visibility Toggles")
        .default_open(false)
        .anchor(egui::Align2::RIGHT_BOTTOM, [-10., -60.])
        .show(ui.ctx(), |ui| {
            let white_forest = Symbol::Area(AreaSymbol::WhiteForest);
            let mut keys = map_tile
                .as_ref()
                .map(|map| map.keys().copied().collect::<Vec<_>>())
                .unwrap_or_default();

            keys.push(white_forest);
            keys.sort();
            keys.dedup();

            for symbol in keys {
                let visible = checkboxes.entry(symbol).or_insert(true);
                ui.checkbox(visible, format!("{symbol}"));
            }
        });
}

pub fn render_map_opacity_slider(ui: &mut egui::Ui, slider_val: &mut f32, rect: egui::Rect) {
    egui::Window::new("Map opacity slider")
        .collapsible(false)
        .resizable(false)
        .title_bar(false)
        .anchor(egui::Align2::LEFT_TOP, [rect.min.x + 10., 65.])
        .show(ui.ctx(), |ui| {
            ui.label("Map opacity");
            ui.add(egui::Slider::new(slider_val, 0.0..=1.0));
        });
}

pub fn render_contour_scores(ui: &mut egui::Ui, score: (f32, f32), weight: f32, rect: egui::Rect) {
    egui::Window::new("Contour scores")
        .collapsible(false)
        .resizable(false)
        .title_bar(false)
        .anchor(egui::Align2::LEFT_TOP, [rect.min.x + 10., 10.])
        .show(ui.ctx(), |ui| {
            ui.label(format!("Contour Score: {:.4}", score.0 + weight * score.1));
            ui.label(format!(
                "= Error + lamba * Energy = {:.4} + {:.2}*{:.4}",
                score.0, weight, score.1
            ));
        });
}

pub fn render_acknowledge(ui: &egui::Ui, attribution: Attribution, rect: egui::Rect) {
    egui::Window::new("Acknowledge")
        .frame(egui::Frame::NONE.fill(egui::Color32::from_rgba_unmultiplied(50, 50, 50, 200)))
        .collapsible(false)
        .resizable(false)
        .title_bar(false)
        .max_width(300.)
        .anchor(egui::Align2::LEFT_BOTTOM, [rect.min.x + 10., -10.])
        .show(ui.ctx(), |ui| {
            ui.hyperlink_to(attribution.text, attribution.url)
                .on_hover_text(attribution.url);
        });
}

pub fn render_background_map_choice(ui: &egui::Ui, source: &mut TileProvider) {
    egui::Window::new("Background Map")
        .collapsible(false)
        .resizable(false)
        .title_bar(false)
        .anchor(egui::Align2::RIGHT_TOP, [-120., 20.])
        .show(ui.ctx(), |ui| {
            ui.radio_value(source, TileProvider::OpenStreetMap, "OpenStreetMap");
            ui.radio_value(source, TileProvider::OpenTopoMap, "OpenTopoMap");
            ui.radio_value(source, TileProvider::ArcGIS, "ArcGIS Satellite");
        });
}
