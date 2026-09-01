use crate::app::{
    OmapMaker, OmapModal, ProcessStage,
    protocol::{AppAction, RegenerationScope},
};
use eframe::egui;

impl OmapMaker {
    pub fn render_adjust_slider_panel(&mut self, ui: &mut egui::Ui) {
        let (heading, help_text) = match self.state {
            ProcessStage::AdjustContours => (
                "Adjust contour settings",
                "Tune contour generation and contour geometry.",
            ),
            ProcessStage::AdjustOpenness => (
                "Adjust openness settings",
                "Tune the yellow/open-land layer and its polygon geometry.",
            ),
            ProcessStage::AdjustVegetation => (
                "Adjust vegetation settings",
                "Tune the green vegetation layers and their polygon geometry.",
            ),
            ProcessStage::AdjustBuildings => (
                "Adjust building detection",
                "Tune conservative roof-surface detection and building footprint geometry.",
            ),
            ProcessStage::AdjustCliffs => (
                "Adjust cliff settings",
                "Tune cliff detection and cliff geometry.",
            ),
            ProcessStage::AdjustWater => (
                "Adjust water settings",
                "Tune water seeds, flood-fill behavior, and the resulting polygon geometry.",
            ),
            ProcessStage::AdjustMarsh => (
                "Adjust marsh settings",
                "Tune marsh detection and the resulting polygon geometry.",
            ),
            ProcessStage::AdjustStreams => (
                "Adjust stream settings",
                "Choose and tune stream detection and line geometry.",
            ),
            ProcessStage::AdjustIntensity => (
                "Adjust lidar intensity settings",
                "Tune lidar intensity filters and their polygon geometry.",
            ),
            _ => unreachable!("Should not render adjustment panel for {:?}", self.state),
        };

        ui.heading(heading);
        ui.add_space(20.);
        ui.label(help_text);

        egui::ScrollArea::both()
            .auto_shrink(false)
            .max_height(ui.available_height() / 1.2)
            .max_width(f32::INFINITY)
            .show(ui, |ui| match self.state {
                ProcessStage::AdjustContours => {
                    self.render_contour_adjustments(ui);
                    ui.add_space(20.);
                    ui.label(egui::RichText::new("Contour Bezier simplification").strong());
                    Self::render_bezier_parameters(
                        ui,
                        &mut self.gui_variables.generation.params.geometry.contours,
                    );
                }
                ProcessStage::AdjustOpenness => {
                    ui.label(egui::RichText::new("Openness threshold").strong());
                    ui.add(
                        egui::Slider::new(
                            &mut self.gui_variables.generation.params.vegetation.yellow,
                            0.0..=1.0,
                        )
                        .text("Yellow 403")
                        .show_value(true),
                    );
                    ui.add_space(20.);
                    ui.label(egui::RichText::new("Openness Bezier simplification").strong());
                    Self::render_bezier_parameters(
                        ui,
                        &mut self
                            .gui_variables
                            .generation
                            .params
                            .geometry
                            .openness
                            .bezier,
                    );
                    ui.checkbox(
                        &mut self
                            .gui_variables
                            .generation
                            .params
                            .geometry
                            .openness
                            .min_size_filter,
                        "Filter polygons by minimum symbol size.",
                    );
                    ui.add_space(20.);
                    Self::render_buffer_rules(
                        ui,
                        "openness_buffer_rule",
                        &mut self
                            .gui_variables
                            .generation
                            .params
                            .geometry
                            .openness
                            .buffer_rules,
                    );
                }
                ProcessStage::AdjustVegetation => {
                    self.render_vegetation_adjustments(ui);
                    ui.add_space(20.);
                    ui.label(egui::RichText::new("Vegetation Bezier simplification").strong());
                    Self::render_bezier_parameters(
                        ui,
                        &mut self
                            .gui_variables
                            .generation
                            .params
                            .geometry
                            .vegetation
                            .bezier,
                    );
                    ui.checkbox(
                        &mut self
                            .gui_variables
                            .generation
                            .params
                            .geometry
                            .vegetation
                            .min_size_filter,
                        "Filter polygons by minimum symbol size.",
                    );
                    ui.add_space(20.);
                    Self::render_buffer_rules(
                        ui,
                        "vegetation_buffer_rule",
                        &mut self
                            .gui_variables
                            .generation
                            .params
                            .geometry
                            .vegetation
                            .buffer_rules,
                    );
                }
                ProcessStage::AdjustBuildings => {
                    self.render_building_adjustments(ui);
                    ui.add_space(20.);
                    ui.label(egui::RichText::new("Building Bezier simplification").strong());
                    Self::render_bezier_parameters(
                        ui,
                        &mut self
                            .gui_variables
                            .generation
                            .params
                            .geometry
                            .buildings
                            .bezier,
                    );
                    ui.checkbox(
                        &mut self
                            .gui_variables
                            .generation
                            .params
                            .geometry
                            .buildings
                            .min_size_filter,
                        "Filter polygons by minimum symbol size.",
                    );
                    ui.add_space(20.);
                    Self::render_buffer_rules(
                        ui,
                        "building_buffer_rule",
                        &mut self
                            .gui_variables
                            .generation
                            .params
                            .geometry
                            .buildings
                            .buffer_rules,
                    );
                }
                ProcessStage::AdjustCliffs => self.render_cliff_adjustments(ui),
                ProcessStage::AdjustWater => self.render_water_adjustments(ui),
                ProcessStage::AdjustStreams => self.render_stream_adjustments(ui),
                ProcessStage::AdjustMarsh => self.render_marsh_adjustments(ui),
                ProcessStage::AdjustIntensity => {
                    self.render_intensity_adjustments(ui);
                    ui.add_space(20.);
                    ui.label(egui::RichText::new("Lidar intensity Bezier simplification").strong());
                    Self::render_bezier_parameters(
                        ui,
                        &mut self
                            .gui_variables
                            .generation
                            .params
                            .geometry
                            .intensity
                            .bezier,
                    );
                    ui.checkbox(
                        &mut self
                            .gui_variables
                            .generation
                            .params
                            .geometry
                            .intensity
                            .min_size_filter,
                        "Filter polygons by minimum symbol size.",
                    );
                    ui.add_space(20.);
                    Self::render_buffer_rules(
                        ui,
                        "intensity_buffer_rule",
                        &mut self
                            .gui_variables
                            .generation
                            .params
                            .geometry
                            .intensity
                            .buffer_rules,
                    );
                }
                _ => unreachable!("Should not render adjustment panel for {:?}", self.state),
            });

        ui.add_space(20.);

        let button_txt = if self.gui_variables.preview.generating_map_tile {
            "Generating map..."
        } else {
            "Re-generate map"
        };
        if ui
            .add_enabled(
                !self.gui_variables.preview.generating_map_tile,
                egui::Button::new(button_txt),
            )
            .clicked()
        {
            self.dispatch_action(AppAction::RegenerateMap(RegenerationScope::Changed));
        }

        ui.add_space(20.);

        ui.add_enabled_ui(!self.gui_variables.preview.generating_map_tile, |ui| {
            ui.horizontal(|ui| {
                if ui.button("Prev step").clicked() {
                    self.dispatch_action(AppAction::PrevState);
                }
                if ui.button("Next step").clicked() {
                    if self.state == ProcessStage::AdjustIntensity {
                        self.dispatch_action(AppAction::OpenModal(OmapModal::ConfirmGenerateMap));
                    } else {
                        self.dispatch_action(AppAction::NextState);
                    }
                }
            });
        });
    }
}
