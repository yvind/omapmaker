use crate::{
    app::{DrawOrder, OmapMaker},
    map::AreaSymbol,
};
use eframe::egui;
use egui_double_slider::DoubleSlider;

impl OmapMaker {
    pub(super) fn render_intensity_adjustments(&mut self, ui: &mut egui::Ui) {
        ui.label(egui::RichText::new("Lidar Intensity filters").strong());
        for (i, intensity_filter) in self
            .gui_variables
            .generation
            .params
            .intensity
            .filters
            .iter_mut()
            .enumerate()
        {
            ui.horizontal(|ui| {
                ui.add(egui::DragValue::new(&mut intensity_filter.low).range(0.0..=1.0));
                ui.add(
                    DoubleSlider::new(
                        &mut intensity_filter.low,
                        &mut intensity_filter.high,
                        0.0..=1.0,
                    )
                    .separation_distance(0.01),
                );
                ui.add(egui::DragValue::new(&mut intensity_filter.high).range(0.0..=1.0));
                egui::ComboBox::from_id_salt(format!("Intensity filter {}", i + 1))
                    .selected_text(format!("{:?}", intensity_filter.symbol))
                    .show_ui(ui, |ui| {
                        for area_symbol in AreaSymbol::draw_order() {
                            ui.selectable_value(
                                &mut intensity_filter.symbol,
                                area_symbol,
                                format!("{:?}", area_symbol),
                            );
                        }
                    });
            });
        }
        ui.horizontal(|ui| {
            if ui.button("Add filter").clicked() {
                self.gui_variables
                    .generation
                    .params
                    .intensity
                    .filters
                    .push(Default::default());
            }
            if ui
                .add_enabled(
                    !self
                        .gui_variables
                        .generation
                        .params
                        .intensity
                        .filters
                        .is_empty(),
                    egui::Button::new("Remove filter"),
                )
                .clicked()
            {
                self.gui_variables.generation.params.intensity.filters.pop();
            }
        });
    }
}
