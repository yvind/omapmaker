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
    }
}
