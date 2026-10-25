use crate::app::OmapMaker;
use eframe::egui;

impl OmapMaker {
    pub(super) fn render_boulder_adjustments(&mut self, ui: &mut egui::Ui) {
        let params = &mut self.gui_variables.generation.params.boulder;
        ui.add_enabled_ui(params.enabled, |ui| {
            ui.label("Broad, opaque rocks with no maximum size limit.");
            ui.add(
                egui::Slider::new(&mut params.minimum_height_m, 1.0..=10.)
                    .text("Minimum boulder height (m)"),
            ).on_hover_text("Default 2 m focuses on large boulders (at least 2 m). Set 1 m to include small boulders. Height is measured above surrounding ground.");
            ui.add(egui::Slider::new(&mut params.maximum_height_to_diameter, 0.5..=2.5)
                .text("Maximum height / footprint width"))
                .on_hover_text("Default 1.0 requires a footprint at least as wide as the rock is tall. Width comes from the observed surface area, so sampling gaps cannot inflate it.");
            ui.add(egui::Slider::new(&mut params.minimum_top_fraction, 0.5..=1.)
                .text("Minimum top / footprint area"))
                .on_hover_text("Fraction of the observed footprint at least 75% of the object's height. Default 80% rejects pointed crowns. Lower values include more rounded rocks.");
            ui.add(
                egui::Slider::new(&mut params.minimum_separation_m, 0.0..=15.)
                    .text("Minimum point separation (m)"),
            )
            .on_hover_text(
                "Also respects the spacing of the point symbols at the selected map scale.",
            );
            ui.add(egui::Slider::new(&mut params.maximum_surface_roughness_m, 0.05..=0.6)
                .text("Maximum top roughness (m)"))
                .on_hover_text("Allows smoothly curved boulders; rejects erratic changes in slope across the top.");
            ui.add(egui::Slider::new(&mut params.maximum_nonterminal_fraction, 0.0..=0.5)
                .text("Maximum passing returns at surface"))
                .on_hover_text("Tree echoes continue through the crown. Rock echoes stop at its surface, even when the pulse first hits canopy above it.");
            ui.add(egui::Slider::new(&mut params.maximum_under_surface_fraction, 0.0..=0.3)
                .text("Maximum last returns beneath surface"))
                .on_hover_text("Rejects crowns that allow returns through to branches or ground within the footprint.");
            ui.add(egui::Slider::new(&mut params.minimum_side_slope_degrees, 20.0..=80.)
                .text("Minimum side steepness (°)"))
                .on_hover_text("The search for surrounding ground scales with rock height and this angle, allowing long, steep sides on very large boulders.");
            ui.add(egui::Slider::new(&mut params.minimum_grounded_side_fraction, 0.25..=1.)
                .text("Minimum sides reaching ground"))
                .on_hover_text("Uses nearby ground returns around the object, allowing irregular sampling. Lower values allow partly obscured sides.");
        });
    }
}
