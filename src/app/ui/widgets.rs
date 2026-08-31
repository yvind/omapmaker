use crate::{
    app::OmapMaker,
    parameters::{BezierParameters, BufferDirection, BufferRule},
};
use eframe::egui;

impl OmapMaker {
    pub(super) fn render_bezier_parameters(ui: &mut egui::Ui, bezier: &mut BezierParameters) {
        ui.checkbox(&mut bezier.enabled, "Output this process in Bezier curves.");
        ui.add_enabled_ui(bezier.enabled, |ui| {
            ui.label("Permitted error in Bezier simplification:");
            ui.add(
                egui::Slider::new(&mut bezier.error, 0.5..=5.0)
                    .fixed_decimals(2)
                    .show_value(true),
            );
        });
    }

    pub(super) fn render_buffer_rules(
        ui: &mut egui::Ui,
        id_prefix: &str,
        buffer_rules: &mut Vec<BufferRule>,
    ) {
        ui.label("Rules are applied in order.");
        for (i, buffer_rule) in buffer_rules.iter_mut().enumerate() {
            ui.horizontal(|ui| {
                egui::ComboBox::from_id_salt(format!("{id_prefix}_{i}"))
                    .selected_text(format!("{:?}", buffer_rule.direction))
                    .show_ui(ui, |ui| {
                        ui.selectable_value(
                            &mut buffer_rule.direction,
                            BufferDirection::Grow,
                            format!("{:?}", BufferDirection::Grow),
                        );
                        ui.selectable_value(
                            &mut buffer_rule.direction,
                            BufferDirection::Shrink,
                            format!("{:?}", BufferDirection::Shrink),
                        );
                    });
                ui.label("Distance: ");
                ui.add(egui::DragValue::new(&mut buffer_rule.amount).range(0.1..=25.0));
            });
        }
        ui.horizontal(|ui| {
            if ui.button("Add rule").clicked() {
                buffer_rules.push(Default::default());
            }
            if ui
                .add_enabled(!buffer_rules.is_empty(), egui::Button::new("Remove rule"))
                .clicked()
            {
                buffer_rules.pop();
            }
        });
    }
}
