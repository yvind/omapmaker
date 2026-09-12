use eframe::egui;
use walkers::{Map, MapMemory, MercatorProjection, PlanarProjection, Projection, Tiles};

use super::{map_controls, map_plugins};
use crate::app::{AppState, OmapMaker, ProcessStage, state::TileProvider};

const BG_COLOR: egui::Color32 = egui::Color32::from_rgb(225, 225, 220);

impl OmapMaker {
    fn show_generic_map<P: Projection>(
        state: ProcessStage,
        gui_variables: &mut AppState,
        map: Map<'_, '_, '_, P>,
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

    fn show_map_controls<P: Projection>(
        &mut self,
        ui: &mut egui::Ui,
        rect: egui::Rect,
        projection: &P,
    ) {
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
        map_controls::render_scale_pos_label(ui, &self.map_memory, self.home, projection);
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

        if self.state != ProcessStage::Welcome
            && self.gui_variables.generation.params.output.crs.is_none()
        {
            let mut min_x = f64::MAX;
            let mut max_x = f64::MIN;
            let mut min_y = f64::MAX;
            let mut max_y = f64::MIN;
            for boundary in self.gui_variables.lidar.boundaries.iter() {
                for p in boundary {
                    if p.x() > max_x {
                        max_x = p.x();
                    } else if p.x() < min_x {
                        min_x = p.x();
                    }
                    if p.y() > max_y {
                        max_y = p.y();
                    } else if p.y() < min_y {
                        min_y = p.y();
                    }
                }
            }
            let scale = (max_x - min_x).max(max_y - min_y);
            let planar_proj = PlanarProjection::new(self.home, 1. / scale);
            Self::clamp_projected_zoom_pos(&mut self.map_memory, &planar_proj);
            let map = Map::new(planar_proj.clone(), &mut self.map_memory, self.home);

            // Local coordinates
            Self::show_generic_map(self.state, &mut self.gui_variables, map, ui, rect);
            self.show_map_controls(ui, rect, &planar_proj);
        } else {
            Self::clamp_mercator_zoom_pos(&mut self.map_memory, &MercatorProjection);

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

            let map = Map::new(MercatorProjection, &mut self.map_memory, self.home)
                .with_layer(http_tiles, 1.);
            Self::show_generic_map(self.state, &mut self.gui_variables, map, ui, rect);
            self.show_map_controls(ui, rect, &MercatorProjection);
        };
    }

    fn clamp_mercator_zoom_pos(map_memory: &mut MapMemory, projection: &MercatorProjection) {
        // clamp zoom
        if map_memory.zoom() > 21. {
            let _ = map_memory.set_zoom(21.);
        } else if map_memory.zoom() < 3. {
            let _ = map_memory.set_zoom(3.);
        }

        // clamp position
        if let Some(pos) = map_memory.detached(projection) {
            let mut new_pos = (pos.x(), pos.y());
            let mut oob = false;
            if pos.x() > 180. {
                oob = true;
                new_pos.0 = 180.;
            } else if pos.x() < -180. {
                oob = true;
                new_pos.0 = -180.;
            }

            if pos.y() > 85. {
                oob = true;
                new_pos.1 = 85.;
            } else if pos.y() < -85. {
                oob = true;
                new_pos.1 = -85.;
            }

            if oob {
                map_memory.center_at(walkers::lon_lat(new_pos.0, new_pos.1));
            }
        }
    }

    fn clamp_projected_zoom_pos(map_memory: &mut MapMemory, projection: &PlanarProjection) {
        // clamp zoom
        if map_memory.zoom() > 16. {
            let _ = map_memory.set_zoom(16.);
        } else if map_memory.zoom() < 6. {
            let _ = map_memory.set_zoom(6.);
        }

        // clamp position
        if let Some(pos) = map_memory.detached(projection) {
            let mut new_pos = (pos.x(), pos.y());
            let mut oob = false;
            if pos.x() > projection.origin.x() + 1. / projection.pixels_per_meter_at_zoom_zero {
                oob = true;
                new_pos.0 = projection.origin.x() + 1. / projection.pixels_per_meter_at_zoom_zero;
            } else if pos.x()
                < projection.origin.x() - 1. / projection.pixels_per_meter_at_zoom_zero
            {
                oob = true;
                new_pos.0 = projection.origin.x() - 1. / projection.pixels_per_meter_at_zoom_zero;
            }

            if pos.y() > projection.origin.y() + 1. / projection.pixels_per_meter_at_zoom_zero {
                oob = true;
                new_pos.1 = projection.origin.y() + 1. / projection.pixels_per_meter_at_zoom_zero;
            } else if pos.y()
                < projection.origin.y() - 1. / projection.pixels_per_meter_at_zoom_zero
            {
                oob = true;
                new_pos.1 = projection.origin.y() - 1. / projection.pixels_per_meter_at_zoom_zero;
            }

            if oob {
                map_memory.center_at(walkers::lon_lat(new_pos.0, new_pos.1));
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
