use crate::{app::OmapMaker, parameters::CliffAlgorithm};
use eframe::egui;
use egui_double_slider::DoubleSlider;

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
            let cliff = &mut self.gui_variables.generation.params.cliff;
            ui.label(egui::RichText::new("Cliff height classification").strong());
            ui.horizontal(|ui| {
                ui.add(
                    egui::DragValue::new(&mut cliff.minimum_cliff_height_m)
                        .range(1.0..=10.0)
                        .speed(0.1)
                        .suffix(" m"),
                )
                .on_hover_text("Minimum elevation change required for a cliff line.");
                ui.add(
                    DoubleSlider::new(
                        &mut cliff.minimum_cliff_height_m,
                        &mut cliff.impassable_cliff_height_m,
                        1.0..=10.0,
                    )
                    .separation_distance(0.1)
                    .invert_highlighting(true),
                )
                .on_hover_text(
                    "Left: minimum cliff-line height. Right: changeover from a small to an impassable cliff. The outside ranges are highlighted.",
                );
                ui.add(
                    egui::DragValue::new(&mut cliff.impassable_cliff_height_m)
                        .range(1.0..=10.0)
                        .speed(0.1)
                        .suffix(" m"),
                )
                .on_hover_text("Elevation change at which a cliff becomes impassable.");
            });
            ui.add(
                egui::Slider::new(&mut cliff.collapse_linearity, 0.0..=10.0)
                    .text("Linearity threshold")
                    .show_value(true),
            )
            .on_hover_text(
                "Minimum main-centerline length as a multiple of the polygon's local width. Smaller values also convert more compact polygons.",
            );
        });
        ui.add_space(20.);
        ui.label(egui::RichText::new("Cliff RDP simplification").strong());
        Self::render_rdp_parameters(
            ui,
            &mut self.gui_variables.generation.params.geometry.cliffs.rdp,
        );
        ui.add_space(10.);
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
        ui.add(
            egui::Slider::new(
                &mut self
                    .gui_variables
                    .generation
                    .params
                    .geometry
                    .cliffs
                    .maximum_hole_area_m2,
                0.0..=500.0,
            )
            .text("Fill polygon holes up to (m²)"),
        )
        .on_hover_text(
            "Applied after all cliff polygon buffer rules. Set to zero to keep every hole.",
        );
    }
}
