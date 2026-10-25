use crate::app::OmapMaker;
use eframe::egui;

impl OmapMaker {
    pub(super) fn render_boulder_field_adjustments(&mut self, ui: &mut egui::Ui) {
        let params = &mut self.gui_variables.generation.params.boulder_fields;
        ui.label("Boulder fields (211) from rough terrain and coherent near-ground surfaces. Trees above the rocks are allowed.");
        ui.add(egui::Slider::new(&mut params.threshold, 0.1..=0.9).text("Boulder field threshold"));
        ui.add_space(20.);
        let geometry = &mut self.gui_variables.generation.params.geometry.boulder_fields;
        ui.checkbox(
            &mut geometry.min_size_filter,
            "Filter fields by minimum symbol size",
        );
        Self::render_buffer_rules(ui, "boulder_field_buffers", &mut geometry.buffer_rules);
    }
}
