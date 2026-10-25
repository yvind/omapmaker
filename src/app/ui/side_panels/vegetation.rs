use crate::app::OmapMaker;
use eframe::egui;

impl OmapMaker {
    pub(super) fn render_vegetation_adjustments(&mut self, ui: &mut egui::Ui) {
        ui.label(egui::RichText::new("Vegetation weighting").strong());
        let weights = &mut self.gui_variables.generation.params.vegetation.weights;
        ui.add(
            egui::Slider::new(&mut weights.low, 0.0..=1.0)
                .text("Low vegetation (0.8–1.8 m)")
                .show_value(true),
        );
        ui.add(
            egui::Slider::new(&mut weights.medium, 0.0..=1.0)
                .text("Medium vegetation (1.8–3 m)")
                .show_value(true),
        );
        ui.add(
            egui::Slider::new(&mut weights.high, 0.0..=1.0)
                .text("High vegetation (3 m and above)")
                .show_value(true),
        );
        ui.add_space(20.);

        ui.label(egui::RichText::new("Green thresholds").strong());
        ui.add(
            egui::Slider::new(
                &mut self.gui_variables.generation.params.vegetation.green.0,
                0.0..=1.0,
            )
            .text("Green 406")
            .show_value(true),
        );
        ui.add(
            egui::Slider::new(
                &mut self.gui_variables.generation.params.vegetation.green.1,
                0.0..=1.0,
            )
            .text("Green 408")
            .show_value(true),
        );
        ui.add(
            egui::Slider::new(
                &mut self.gui_variables.generation.params.vegetation.green.2,
                0.0..=1.0,
            )
            .text("Green 410")
            .show_value(true),
        );

        let greens = &mut self.gui_variables.generation.params.vegetation.green;
        greens.0 = greens.0.clamp(0., greens.1);
        greens.2 = greens.2.clamp(greens.1, 1.);
        greens.1 = greens.1.clamp(greens.0, greens.2);

        ui.add_space(20.);
        let vegetation = &mut self.gui_variables.generation.params.vegetation;
        ui.checkbox(&mut vegetation.undergrowth_enabled, "Detect undergrowth (409)")
            .on_hover_text("Low vegetation, mainly 0.2–0.8 m above ground. Overlapping green becomes one shade darker; boulder fields take priority over hatching.");
        ui.add(
            egui::Slider::new(&mut vegetation.undergrowth_threshold, 0.1..=0.9)
                .text("Undergrowth threshold"),
        );

        ui.add_space(20.);
        let boundaries = &mut vegetation.boundaries;
        ui.checkbox(&mut boundaries.enabled, "Clear vegetation boundaries (416)")
            .on_hover_text("Trace sharp changes in canopy height. Only continuous edges longer than the minimum length are emitted.");
        ui.add_enabled_ui(boundaries.enabled, |ui| {
            ui.add(
                egui::Slider::new(&mut boundaries.minimum_height_change_m, 0.5..=15.)
                    .text("Minimum height change (m)"),
            ).on_hover_text("Required canopy-height difference across the edge.");
            ui.add(
                egui::Slider::new(&mut boundaries.minimum_gradient, 0.1..=5.)
                    .text("Minimum sharpness (m/m)"),
            ).on_hover_text("Minimum height change per ground metre after smoothing. Increase this to reject gradual transitions.");
            ui.add(
                egui::Slider::new(&mut boundaries.minimum_length_m, 5.0..=100.)
                    .text("Minimum boundary length (m)"),
            ).on_hover_text("Applied to continuous lines after clipping and joining adjacent tiles.");
            ui.add(
                egui::Slider::new(&mut boundaries.smoothing_m, 0.0..=3.)
                    .text("Canopy smoothing (m)"),
            ).on_hover_text("Gaussian smoothing suppresses individual crown fluctuations. Zero disables smoothing.");
        });
    }
}
