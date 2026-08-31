use crate::{app::OmapMaker, parameters::CliffAlgorithm};
use eframe::egui;

impl OmapMaker {
    pub(super) fn render_cliff_adjustments(&mut self, ui: &mut egui::Ui) {
        ui.horizontal(|ui| {
            ui.label("Cliff algorithm:");
            egui::ComboBox::from_id_salt("Cliff algorithm")
                .selected_text(
                    self.gui_variables
                        .generation
                        .params
                        .cliff
                        .algorithm
                        .to_string(),
                )
                .show_ui(ui, |ui| {
                    ui.selectable_value(
                        &mut self.gui_variables.generation.params.cliff.algorithm,
                        CliffAlgorithm::PolynomialFit,
                        "Polynomial fit (adaptive)",
                    );
                    ui.selectable_value(
                        &mut self.gui_variables.generation.params.cliff.algorithm,
                        CliffAlgorithm::SobelSlope,
                        "Sobel slope (legacy)",
                    );
                });
        });
        ui.label(egui::RichText::new("Cliff threshold").strong());
        ui.add(
            egui::Slider::new(
                &mut self.gui_variables.generation.params.cliff.cliff,
                0.2..=5.0,
            )
            .text("Cliff")
            .show_value(true),
        );
        ui.add_space(10.);
        ui.checkbox(
            &mut self.gui_variables.generation.params.cliff.collapse,
            "Convert linear polygons to line objects",
        );
        ui.add_enabled_ui(self.gui_variables.generation.params.cliff.collapse, |ui| {
                        ui.add(egui::Slider::new(
                            &mut self.gui_variables.generation.params.cliff.collapse_amount_small_cliff,
                            0.1..=5.0
                        ).text("Collapse amount").show_value(true));
                        ui.add(egui::Slider::new(
                            &mut self.gui_variables.generation.params.cliff.collapse_amount_large_cliff,
                            0.1..=5.0
                        ).text("Collapse amount").show_value(true));
                        ui.add(
                            egui::Slider::new(
                                &mut self.gui_variables.generation.params.cliff.collapse_linearity,
                                0.0..=10.0,
                            )
                            .text("Linearity threshold")
                            .show_value(true),
                        )
                        .on_hover_text(
                            "Minimum significant centerline branch length as a multiple of the collapse amount. Smaller values also convert more compact polygons. Polygons below the minimum symbol area bypass this threshold.",
                        );
                    });
        ui.add_space(20.);
        ui.label(egui::RichText::new("Cliff Bezier simplification").strong());
        Self::render_bezier_parameters(
            ui,
            &mut self.gui_variables.generation.params.geometry.cliffs.bezier,
        );
        ui.checkbox(
            &mut self
                .gui_variables
                .generation
                .params
                .geometry
                .cliffs
                .min_size_filter,
            "Filter lines/polygons by minimum symbol size.",
        );
        ui.add_space(20.);
        Self::render_buffer_rules(
            ui,
            "cliffs_buffer_rule",
            &mut self
                .gui_variables
                .generation
                .params
                .geometry
                .cliffs
                .buffer_rules,
        );
    }
}
