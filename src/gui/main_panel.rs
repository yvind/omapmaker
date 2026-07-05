use eframe::egui;
use walkers::{Map, MapMemory, MercatorProjection, Plugin, ProjectedProjection, Tiles};

use super::{ProcessStage, map_controls, map_plugins};
use crate::OmapMaker;

const BG_COLOR: egui::Color32 = egui::Color32::from_rgb(225, 225, 220);

enum MapType<'a, 'b, 'c> {
    Global(Map<'a, 'b, 'c, MercatorProjection>),
    Local(Map<'a, 'b, 'c, ProjectedProjection>),
}

impl<'c> MapType<'_, '_, 'c> {
    fn with_plugin(self, plugin: impl Plugin + 'c) -> Self {
        match self {
            MapType::Global(map) => MapType::Global(map.with_plugin(plugin)),
            MapType::Local(map) => MapType::Local(map.with_plugin(plugin)),
        }
    }

    fn draw_map(self, ui: &mut egui::Ui, rect: egui::Rect) {
        match self {
            MapType::Global(map) => ui.put(rect, map),
            MapType::Local(map) => ui.put(rect, map),
        };
    }
}

impl OmapMaker {
    pub fn render_map(&mut self, ui: &mut egui::Ui) {
        let rect = ui.clip_rect();

        ui.painter().rect(
            rect,
            0.,
            BG_COLOR,
            egui::Stroke::NONE,
            egui::StrokeKind::Middle,
        );

        let map = if self.state != ProcessStage::Welcome
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
            let projproj = ProjectedProjection::new(self.home, 1. / scale);
            Self::clamp_projected_zoom_pos(&mut self.map_memory, &projproj);

            // Local coordinates
            MapType::Local(Map::new(projproj.clone(), &mut self.map_memory, self.home))
        } else {
            Self::clamp_mercator_zoom_pos(&mut self.map_memory, &MercatorProjection);

            let http_tiles = match self.gui_variables.map_view.tile_provider {
                super::gui_variables::TileProvider::OpenStreetMap => &mut self.http_tiles.0,
                super::gui_variables::TileProvider::OpenTopoMap => &mut self.http_tiles.1,
                super::gui_variables::TileProvider::ArcGIS => &mut self.http_tiles.2,
            };

            map_controls::render_acknowledge(ui, http_tiles.attribution(), rect);
            map_controls::render_background_map_choice(
                ui,
                &mut self.gui_variables.map_view.tile_provider,
            );

            let error_text = match self.gui_variables.map_view.tile_provider {
                super::gui_variables::TileProvider::OpenStreetMap => {
                    "If you see this the OSM background-map did not load."
                }
                super::gui_variables::TileProvider::OpenTopoMap => {
                    "If you see this the OTM background-map did not load."
                }
                super::gui_variables::TileProvider::ArcGIS => {
                    "If you see this the ArcGIS background-map did not load."
                }
            };

            ui.vertical_centered(|ui| ui.colored_label(egui::Color32::RED, error_text));

            // OSM map
            MapType::Global(
                Map::new(MercatorProjection, &mut self.map_memory, self.home)
                    .with_layer(http_tiles, 1.),
            )
        };

        // add plugins
        let map = match &self.state {
            ProcessStage::ChooseSquare => {
                let map = map.with_plugin(map_plugins::LasBoundaryPainter::new(
                    &self.gui_variables.lidar.boundaries,
                    self.gui_variables.project.selected_file,
                    true,
                    None,
                ));

                map.with_plugin(map_plugins::ClickListener::new(
                    &self.gui_variables.lidar.boundaries,
                    &mut self.gui_variables.project.selected_file,
                ))
            }
            ProcessStage::ChooseSubTile => {
                let map = map.with_plugin(map_plugins::LasBoundaryPainter::new(
                    &self.gui_variables.tile.subtile_boundaries,
                    self.gui_variables.tile.selected_tile,
                    true,
                    Some(&self.gui_variables.tile.subtile_neighbors),
                ));
                map.with_plugin(map_plugins::ClickListener::new(
                    &self.gui_variables.tile.subtile_boundaries,
                    &mut self.gui_variables.tile.selected_tile,
                ))
            }
            ProcessStage::DrawPolygon => {
                let map = map.with_plugin(map_plugins::LasBoundaryPainter::new(
                    &self.gui_variables.lidar.boundaries,
                    None,
                    false,
                    None,
                ));
                map.with_plugin(map_plugins::PolygonDrawer::new(
                    &mut self.gui_variables.area.polygon_filter,
                    &mut self.gui_variables.area.drawing_polygon,
                ))
            }
            state if state.is_adjustment() => map.with_plugin(map_plugins::OmapDrawer::new(
                &self.gui_variables.preview.map_tile,
                &self.gui_variables.preview.visibility_checkboxes,
                self.gui_variables.preview.map_opacity,
            )),
            ProcessStage::ExportDone => {
                let map = map.with_plugin(map_plugins::LasBoundaryPainter::new(
                    &self.gui_variables.lidar.boundaries,
                    None,
                    false,
                    None,
                ));
                map.with_plugin(map_plugins::PolygonDrawer::readonly(
                    &mut self.gui_variables.area.polygon_filter,
                ))
            }
            ProcessStage::Welcome => map,
            ProcessStage::ShowComponents => map.with_plugin(map_plugins::LasComponentPainter::new(
                &self.gui_variables.lidar.boundaries,
                &self.gui_variables.lidar.connected_components,
            )),
            _ => unreachable!("The render_map fn should not be called for this state"),
        };

        // ugly hack
        let projection = if let MapType::Local(m) = &map {
            Some(m.projection().clone())
        } else {
            None
        };
        map.draw_map(ui, rect);

        // Draw utility windows.
        match self.state {
            state if state.is_adjustment() => {
                map_controls::render_contour_scores(
                    ui,
                    self.gui_variables.preview.contour_score,
                    self.gui_variables.generation.params.contour.algo_lambda as f32,
                    rect,
                );
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
            _ => (),
        }

        map_controls::render_zoom(ui, &mut self.map_memory);
        map_controls::render_home(ui, &mut self.map_memory, self.home_zoom);
        if let Some(proj) = projection {
            map_controls::render_scale_pos_label(ui, &self.map_memory, self.home, &proj);
        } else {
            map_controls::render_scale_pos_label(
                ui,
                &self.map_memory,
                self.home,
                &MercatorProjection,
            );
        }
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

    fn clamp_projected_zoom_pos(map_memory: &mut MapMemory, projection: &ProjectedProjection) {
        // clamp zoom
        if map_memory.zoom() > 16. {
            let _ = map_memory.set_zoom(16.);
        } else if map_memory.zoom() < 8. {
            let _ = map_memory.set_zoom(8.);
        }

        // clamp position
        if let Some(pos) = map_memory.detached(projection) {
            let mut new_pos = (pos.x(), pos.y());
            let mut oob = false;
            if pos.x() > projection.center.x() + 1. / projection.scale {
                oob = true;
                new_pos.0 = projection.center.x() + 1. / projection.scale;
            } else if pos.x() < projection.center.x() - 1. / projection.scale {
                oob = true;
                new_pos.0 = projection.center.x() - 1. / projection.scale;
            }

            if pos.y() > projection.center.y() + 1. / projection.scale {
                oob = true;
                new_pos.1 = projection.center.y() + 1. / projection.scale;
            } else if pos.y() < projection.center.y() - 1. / projection.scale {
                oob = true;
                new_pos.1 = projection.center.y() - 1. / projection.scale;
            }

            if oob {
                map_memory.center_at(walkers::lon_lat(new_pos.0, new_pos.1));
            }
        }
    }

    pub fn render_console(&mut self, ui: &mut egui::Ui) {
        egui::ScrollArea::both()
            .stick_to_bottom(true)
            .auto_shrink(false)
            .max_height(f32::INFINITY)
            .show(ui, |ui| {
                egui::TextEdit::multiline(&mut self.gui_variables.log_terminal)
                    .font(egui::FontSelection::Style(egui::TextStyle::Monospace))
                    .desired_width(f32::INFINITY)
                    .interactive(false)
                    .show(ui);
            });
        ui.request_repaint_after(std::time::Duration::from_millis(100));
    }
}
