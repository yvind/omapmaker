use eframe::egui;
use walkers::{Map, MapMemory, Tiles};

use super::{map_controls, map_plugins};
use crate::app::{AppState, MapProjection, OmapMaker, ProcessStage, state::TileProvider};

const BG_COLOR: egui::Color32 = egui::Color32::from_rgb(225, 225, 220);

impl OmapMaker {
    fn show_map(
        state: ProcessStage,
        gui_variables: &mut AppState,
        map: Map<'_, '_, '_, MapProjection>,
        ui: &mut egui::Ui,
        rect: egui::Rect,
    ) {
        // add plugins
        let map = match state {
            ProcessStage::ChooseSquare => map.with_plugin(map_plugins::TestAreaSelector::new(
                &gui_variables.tile.test_area_display,
                &gui_variables.tile.test_area_projected,
                &mut gui_variables.tile.selected_square,
                &mut gui_variables.tile.selected_square_boundary,
                gui_variables.generation.params.output.crs.as_ref(),
            )),
            ProcessStage::DrawPolygon => {
                let map = map.with_plugin(map_plugins::LasBoundaryPainter::new(
                    &gui_variables.lidar.boundaries,
                ));
                map.with_plugin(map_plugins::PolygonDrawer::new(
                    &mut gui_variables.area.polygon_filter,
                    &mut gui_variables.area.drawing_polygon,
                ))
            }
            state if state.is_adjustment() => map.with_plugin(map_plugins::OmapDrawer::new(
                &gui_variables.preview.map_tile,
                &gui_variables.preview.visibility_checkboxes,
                gui_variables.preview.map_opacity,
            )),
            ProcessStage::ExportDone => {
                let map = map.with_plugin(map_plugins::LasBoundaryPainter::new(
                    &gui_variables.lidar.boundaries,
                ));
                map.with_plugin(map_plugins::PolygonDrawer::readonly(
                    &mut gui_variables.area.polygon_filter,
                ))
            }
            ProcessStage::Welcome => map,
            ProcessStage::ShowComponents => map.with_plugin(map_plugins::LasComponentPainter::new(
                &gui_variables.lidar.boundaries,
                &gui_variables.lidar.connected_components,
            )),
            _ => unreachable!("The render_map fn should not be called for this state"),
        };

        ui.put(rect, map);
    }

    fn show_map_controls(&mut self, ui: &mut egui::Ui, rect: egui::Rect) {
        // Draw utility windows.
        if self.state.is_adjustment() {
            map_controls::render_map_opacity_slider(
                ui,
                &mut self.gui_variables.preview.map_opacity,
                rect,
            );
            map_controls::render_symbol_toggles(
                ui,
                &self.gui_variables.preview.map_tile,
                &mut self.gui_variables.preview.visibility_checkboxes,
            );
        }
        if self.state == ProcessStage::AdjustContours {
            map_controls::render_contour_scores(
                ui,
                self.gui_variables.preview.contour_score,
                self.gui_variables.generation.params.contour.algo_lambda,
                rect,
            );
        }

        map_controls::render_zoom(ui, &mut self.map_memory);
        map_controls::render_home(ui, &mut self.map_memory, self.home_zoom);
        map_controls::render_scale_pos_label(ui, &self.map_memory, self.home);
    }

    pub fn render_map(&mut self, ui: &mut egui::Ui) {
        let rect = ui.clip_rect();

        ui.painter().rect(
            rect,
            0.,
            BG_COLOR,
            egui::Stroke::NONE,
            egui::StrokeKind::Middle,
        );

        Self::clamp_map(&mut self.map_memory);

        let http_tiles = if self.map_memory.projection().is_mercator() {
            let http_tiles = match self.gui_variables.map_view.tile_provider {
                TileProvider::OpenStreetMap => &mut self.background_tiles.osm,
                TileProvider::OpenTopoMap => &mut self.background_tiles.otm,
                TileProvider::GoogleSatellite => &mut self.background_tiles.satellite,
            };

            map_controls::render_acknowledge(ui, http_tiles.attribution(), rect);
            map_controls::render_background_map_choice(
                ui,
                &mut self.gui_variables.map_view.tile_provider,
            );

            let error_text = match self.gui_variables.map_view.tile_provider {
                TileProvider::OpenStreetMap => {
                    "If you see this the OSM background-map did not load."
                }
                TileProvider::OpenTopoMap => "If you see this the OTM background-map did not load.",
                TileProvider::GoogleSatellite => {
                    "If you see this the Google satellite background-map did not load."
                }
            };

            ui.vertical_centered(|ui| ui.colored_label(egui::Color32::RED, error_text));

            Some(http_tiles)
        } else {
            None
        };

        let map = Map::new(&mut self.map_memory, self.home);
        let map = if let Some(http_tiles) = http_tiles {
            map.with_layer(http_tiles, 1.0)
        } else {
            map
        };
        Self::show_map(self.state, &mut self.gui_variables, map, ui, rect);
        self.show_map_controls(ui, rect);
    }

    fn clamp_map(map_memory: &mut MapMemory<MapProjection>) {
        let zoom_range = map_memory.projection().zoom_range();
        if map_memory.zoom() > *zoom_range.end() {
            let _ = map_memory.set_zoom(*zoom_range.end());
        } else if map_memory.zoom() < *zoom_range.start() {
            let _ = map_memory.set_zoom(*zoom_range.start());
        }

        if let Some(position) = map_memory.detached() {
            let clamped = map_memory.projection().clamp_position(position);
            if position.x() != clamped.x() || position.y() != clamped.y() {
                map_memory.center_at(clamped);
            }
        }
    }

    pub fn render_console(&mut self, ui: &mut egui::Ui) {
        let available_size = ui.available_size();
        let row_height = ui.text_style_height(&egui::TextStyle::Monospace);
        let desired_rows = (available_size.y / row_height).ceil().max(1.) as usize;

        egui::ScrollArea::both()
            .stick_to_bottom(true)
            .auto_shrink(false)
            .show(ui, |ui| {
                egui::TextEdit::multiline(&mut self.gui_variables.log_terminal)
                    .font(egui::FontSelection::Style(egui::TextStyle::Monospace))
                    .desired_width(f32::INFINITY)
                    .desired_rows(desired_rows)
                    .interactive(false)
                    .show(ui);
            });
        ui.request_repaint_after(std::time::Duration::from_millis(100));
    }
}
