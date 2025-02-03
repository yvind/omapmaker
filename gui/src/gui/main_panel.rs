use std::time::Duration;

use walkers::{LocalMap, Map, Tiles};

use super::{map_controls, map_plugins, OmapMaker, ProcessStage};
use eframe::egui;

impl OmapMaker {
    pub fn render_map(&mut self, ui: &mut egui::Ui) {
        let rect = if self.state != ProcessStage::Welcome
            && self.gui_variables.map_params.output_epsg.is_none()
        {
            // Local coordinates
            self.render_local_map(ui)
        } else {
            let rect = self.render_walkers_map(ui);
            map_controls::render_acknowledge(ui, self.http_tiles.attribution(), rect);
            rect
        };

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
            _ => (),
        }

        map_controls::render_zoom(ui, &mut self.map_memory);
        map_controls::render_home(ui, &mut self.map_memory, self.home_zoom);
        map_controls::render_scale_pos_label(ui, &self.map_memory, self.home);
    }

    fn render_local_map(&mut self, ui: &mut egui::Ui) -> egui::Rect {
        let map = LocalMap::new(&mut self.map_memory, self.home);

        // add different plugins based on state
        let map = match self.state {
            ProcessStage::ChooseSquare => {
                let map = map.with_plugin(map_plugins::LasBoundaryPainter::new(
                    &self.gui_variables.boundaries,
                    self.gui_variables.file_params.selected_file,
                    true,
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
            ProcessStage::DrawPolygon => {
                let map = map.with_plugin(map_plugins::LasBoundaryPainter::new(
                    &self.gui_variables.boundaries,
                    self.gui_variables.file_params.selected_file,
                    false,
                ));
                map.with_plugin(map_plugins::PolygonDrawer::new(
                    &mut self.gui_variables.polygon_filter,
                    &mut self.state,
                ))
            }
            ProcessStage::AdjustSliders => {
                map.with_plugin(map_plugins::OmapDrawer::new(&self.gui_variables.map_tile))
            }
            ProcessStage::ExportDone => {
                let map = map.with_plugin(map_plugins::LasBoundaryPainter::new(
                    &self.gui_variables.boundaries,
                    None,
                    false,
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
        let rect = ui.ctx().available_rect();
        ui.put(rect, map);
        rect
    }

    fn render_walkers_map(&mut self, ui: &mut egui::Ui) -> egui::Rect {
        // clamp zoom
        if self.map_memory.zoom() > 21. {
            self.map_memory.set_zoom(21.).unwrap();
        } else if self.map_memory.zoom() < 3. {
            self.map_memory.set_zoom(3.).unwrap();
        }

        // clamp position
        if let Some(pos) = self.map_memory.detached() {
            let mut new_pos = (pos.y, pos.x);
            let mut oob = false;
            if pos.x > 180. {
                oob = true;
                new_pos.1 = 180.;
            } else if pos.x < -180. {
                oob = true;
                new_pos.1 = -180.;
            }

            if pos.y > 85. {
                oob = true;
                new_pos.0 = 85.;
            } else if pos.y < -85. {
                oob = true;
                new_pos.0 = -85.;
            }

            if oob {
                self.map_memory
                    .center_at(walkers::pos_from_lat_lon(new_pos.0, new_pos.1));
            }
        }

        let map = Map::new(Some(&mut self.http_tiles), &mut self.map_memory, self.home);

        // add different plugins based on state
        let map = match self.state {
            ProcessStage::ChooseSquare => {
                let map = map.with_plugin(map_plugins::LasBoundaryPainter::new(
                    &self.gui_variables.boundaries,
                    self.gui_variables.file_params.selected_file,
                    true,
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
            ProcessStage::DrawPolygon => {
                let map = map.with_plugin(map_plugins::LasBoundaryPainter::new(
                    &self.gui_variables.boundaries,
                    self.gui_variables.file_params.selected_file,
                    false,
                ));
                map.with_plugin(map_plugins::PolygonDrawer::new(
                    &mut self.gui_variables.polygon_filter,
                    &mut self.state,
                ))
            }
            ProcessStage::AdjustSliders => {
                map.with_plugin(map_plugins::OmapDrawer::new(&self.gui_variables.map_tile))
            }
            ProcessStage::ExportDone => {
                let map = map.with_plugin(map_plugins::LasBoundaryPainter::new(
                    &self.gui_variables.boundaries,
                    None,
                    false,
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

        ui.colored_label(
            egui::Color32::RED,
            "If you see this the OSM background-map did not load.\nThe app still works, just not as nice too look at.",
        );
        // Draw the map widget over the label, so that the label is visible only if the map doesn't load
        let rect = ui.ctx().available_rect();
        ui.put(rect, map);
        rect
    }

    pub fn render_console(&mut self, ui: &mut egui::Ui) {
        egui::ScrollArea::both()
            .stick_to_bottom(true)
            .show(ui, |ui| {
                egui::TextEdit::multiline(&mut self.gui_variables.log_terminal)
                    .code_editor()
                    .min_size(ui.available_size())
                    .desired_width(f32::INFINITY)
                    .interactive(false)
                    .show(ui);
            });
        ui.ctx().request_repaint_after(Duration::from_millis(100));
    }
}
