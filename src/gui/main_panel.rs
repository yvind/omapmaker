use eframe::egui;
use walkers::{LocalMap, Map, MapMemory, Maps, Tiles};

use super::{map_controls, map_plugins, ProcessStage};
use crate::OmapMaker;

const BG_COLOR: egui::Color32 = egui::Color32::from_rgb(225, 225, 220);

impl OmapMaker {
    pub fn render_map(&mut self, ui: &mut egui::Ui) {
        let rect = ui.ctx().available_rect();

        ui.painter().rect(
            rect,
            0.,
            BG_COLOR,
            egui::Stroke::NONE,
            egui::StrokeKind::Middle,
        );

        let map = if self.state != ProcessStage::Welcome
            && self.gui_variables.map_params.output_epsg.is_none()
        {
            // Local coordinates
            Maps::LocalMap(LocalMap::new(&mut self.map_memory, self.home))
        } else {
            Self::clamp_zoom_pos(&mut self.map_memory);

            let http_tiles = match self.gui_variables.tile_provider {
                super::gui_variables::TileProvider::OpenStreetMap => &mut self.http_tiles.0,
                super::gui_variables::TileProvider::OpenTopoMap => &mut self.http_tiles.1,
            };

            map_controls::render_acknowledge(ui, http_tiles.attribution(), rect);
            map_controls::render_background_map_choice(ui, &mut self.gui_variables.tile_provider);

            let error_text = match self.gui_variables.tile_provider {
                super::gui_variables::TileProvider::OpenStreetMap => {
                    "If you see this the OSM background-map did not load."
                }
                super::gui_variables::TileProvider::OpenTopoMap => {
                    "If you see this the OTM background-map did not load."
                }
            };

            ui.vertical_centered(|ui| ui.colored_label(egui::Color32::RED, error_text));

            // OSM map
            Maps::Map(Map::new(Some(http_tiles), &mut self.map_memory, self.home))
        };

        // add plugins
        let map = match &self.state {
            ProcessStage::ChooseSquare => {
                let map = map.with_plugin(map_plugins::LasBoundaryPainter::new(
                    &self.gui_variables.boundaries,
                    self.gui_variables.file_params.selected_file,
                    true,
                    None,
                ));
                let map = map.with_plugin(map_plugins::ClickListener::new(
                    &self.gui_variables.boundaries,
                    &mut self.gui_variables.file_params.selected_file,
                ));
                map.with_plugin(map_plugins::PolygonDrawer::new(
                    &mut self.gui_variables.polygon_filter,
                    &mut self.state,
                ))
            }
            ProcessStage::ChooseSubTile => {
                let map = map.with_plugin(map_plugins::LasBoundaryPainter::new(
                    &self.gui_variables.subtile_boundaries,
                    self.gui_variables.selected_tile,
                    true,
                    Some(&self.gui_variables.subtile_neighbors),
                ));
                map.with_plugin(map_plugins::ClickListener::new(
                    &self.gui_variables.subtile_boundaries,
                    &mut self.gui_variables.selected_tile,
                ))
            }
            ProcessStage::DrawPolygon => {
                let map = map.with_plugin(map_plugins::LasBoundaryPainter::new(
                    &self.gui_variables.boundaries,
                    self.gui_variables.file_params.selected_file,
                    false,
                    None,
                ));
                map.with_plugin(map_plugins::PolygonDrawer::new(
                    &mut self.gui_variables.polygon_filter,
                    &mut self.state,
                ))
            }
            ProcessStage::AdjustSliders => map.with_plugin(map_plugins::OmapDrawer::new(
                &self.gui_variables.map_tile,
                &self.gui_variables.visibility_checkboxes,
                self.gui_variables.map_opacity,
            )),
            ProcessStage::ExportDone => {
                let map = map.with_plugin(map_plugins::LasBoundaryPainter::new(
                    &self.gui_variables.boundaries,
                    None,
                    false,
                    None,
                ));
                map.with_plugin(map_plugins::PolygonDrawer::new(
                    &mut self.gui_variables.polygon_filter,
                    &mut self.state,
                ))
            }
            ProcessStage::Welcome => map,
            ProcessStage::ShowComponents => map.with_plugin(map_plugins::LasComponentPainter::new(
                &self.gui_variables.boundaries,
                &self.gui_variables.connected_components,
            )),
            _ => unreachable!("The render_map fn should not be called for this state"),
        };

        ui.put(rect, map);

        // Draw utility windows.
        match self.state {
            ProcessStage::ChooseSquare => map_controls::render_draw_button(
                ui,
                true,
                rect,
                &mut self.gui_variables.polygon_filter,
                &mut self.state,
            ),
            ProcessStage::DrawPolygon => map_controls::render_draw_button(
                ui,
                false,
                rect,
                &mut self.gui_variables.polygon_filter,
                &mut self.state,
            ),
            ProcessStage::AdjustSliders => {
                map_controls::render_contour_scores(
                    ui,
                    self.gui_variables.contour_score,
                    self.gui_variables.map_params.contour_algo_lambda as f32,
                    rect,
                );
                map_controls::render_map_opacity_slider(
                    ui,
                    &mut self.gui_variables.map_opacity,
                    rect,
                );
                map_controls::render_symbol_toggles(
                    ui,
                    &self.gui_variables.map_tile,
                    &mut self.gui_variables.visibility_checkboxes,
                );
            }
            _ => (),
        }

        map_controls::render_zoom(ui, &mut self.map_memory);
        map_controls::render_home(ui, &mut self.map_memory, self.home_zoom);
        map_controls::render_scale_pos_label(ui, &self.map_memory, self.home);
    }

    fn clamp_zoom_pos(map_memory: &mut MapMemory) {
        // clamp zoom
        if map_memory.zoom() > 21. {
            map_memory.set_zoom(21.).unwrap();
        } else if map_memory.zoom() < 3. {
            map_memory.set_zoom(3.).unwrap();
        }

        // clamp position
        if let Some(pos) = map_memory.detached() {
            let mut new_pos = (pos.x, pos.y);
            let mut oob = false;
            if pos.x > 180. {
                oob = true;
                new_pos.0 = 180.;
            } else if pos.x < -180. {
                oob = true;
                new_pos.0 = -180.;
            }

            if pos.y > 85. {
                oob = true;
                new_pos.1 = 85.;
            } else if pos.y < -85. {
                oob = true;
                new_pos.1 = -85.;
            }

            if oob {
                map_memory.center_at(walkers::pos_from_lon_lat(new_pos.0, new_pos.1));
            }
        }
    }

    pub fn render_console(&mut self, ui: &mut egui::Ui) {
        egui::ScrollArea::both()
            .stick_to_bottom(true)
            .show(ui, |ui| {
                egui::TextEdit::multiline(&mut self.gui_variables.log_terminal)
                    .font(egui::FontSelection::Style(egui::TextStyle::Monospace))
                    .min_size(ui.available_size())
                    .desired_width(f32::INFINITY)
                    .interactive(false)
                    .show(ui);
            });
        ui.ctx()
            .request_repaint_after(std::time::Duration::from_millis(100));
    }
}
