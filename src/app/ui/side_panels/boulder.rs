use crate::app::OmapMaker;
use eframe::egui;

impl OmapMaker {
    pub(super) fn render_boulder_adjustments(&mut self, ui: &mut egui::Ui) {
        let params = &mut self.gui_variables.generation.params.boulder;
        ui.checkbox(&mut params.enabled, "Detect boulders from LiDAR");
        ui.add_enabled_ui(params.enabled, |ui| {
            ui.label("Opaque surfaces above ground, checked against full tree profiles.");
            ui.add(
                egui::Slider::new(&mut params.minimum_height_m, 1.0..=10.)
                    .text("Minimum boulder height (m)"),
            ).on_hover_text("Default 2 m focuses on large boulders (at least 2 m). Set 1 m to include small boulders. Height is measured above surrounding ground.");
            ui.add(
                egui::Slider::new(&mut params.maximum_diameter_m, 3.0..=24.)
                    .text("Maximum footprint diameter (m)"),
            );
            ui.add(egui::Slider::new(&mut params.maximum_height_to_diameter, 1.0..=5.)
                .text("Maximum height / footprint width"))
                .on_hover_text("Rejects narrow canopy fragments high above the ground. Width comes from the observed surface area.");
            ui.add(egui::Slider::new(&mut params.minimum_area_m2, 1.0..=100.).text("Minimum observed surface area (m²)"));
            ui.add(egui::Slider::new(&mut params.minimum_top_area_m2, 0.25..=50.)
                .text("Minimum broad top area (m²)"));
            ui.add(egui::Slider::new(&mut params.minimum_top_fraction, 0.2..=0.7)
                .text("Minimum broad top fraction"))
                .on_hover_text("Requires a substantial part of the footprint near the top; rejects pointed tree crowns.");
            ui.add(egui::Slider::new(&mut params.maximum_surface_roughness_m, 0.05..=0.6)
                .text("Maximum top roughness (m)"))
                .on_hover_text("Allows smoothly curved boulders; rejects erratic changes in slope across the top.");
            ui.add(
                egui::Slider::new(&mut params.minimum_point_support, 6..=100)
                    .text("Minimum distinct last returns"),
            ).on_hover_text("Single returns count as last returns. First and intermediate echoes cannot support a boulder.");
            ui.add(egui::Slider::new(&mut params.maximum_nonterminal_fraction, 0.0..=0.5)
                .text("Maximum passing returns at surface"))
                .on_hover_text("Tree echoes continue through the crown. Rock echoes stop at its surface, even when the pulse first hits canopy above it.");
            ui.add(egui::Slider::new(&mut params.maximum_under_surface_fraction, 0.0..=0.3)
                .text("Maximum last returns beneath surface"))
                .on_hover_text("Rejects crowns that allow returns through to branches or ground within the footprint.");
            ui.add(egui::Slider::new(&mut params.minimum_side_slope_degrees, 20.0..=80.)
                .text("Minimum side steepness (°)"));
            ui.add(egui::Slider::new(&mut params.maximum_side_runoff_m, 0.5..=5.)
                .text("Maximum distance to ground (m)"));
            ui.add(egui::Slider::new(&mut params.minimum_grounded_side_fraction, 0.25..=1.)
                .text("Minimum sides reaching ground"))
                .on_hover_text("Uses nearby ground returns around the object, allowing irregular sampling. Lower values allow partly obscured sides.");
            ui.add(egui::Slider::new(&mut params.ground_level_tolerance_m, 0.1..=0.75)
                .text("Surrounding ground tolerance (m)"));
            ui.add(
                egui::Slider::new(&mut params.confidence_threshold, 0.5..=0.95)
                    .text("Minimum confidence"),
            );
            ui.add(
                egui::Slider::new(&mut params.minimum_separation_m, 0.0..=15.)
                    .text("Minimum point separation (m)"),
            )
            .on_hover_text(
                "Also respects the spacing of the point symbols at the selected map scale.",
            );
        });
    }
}
