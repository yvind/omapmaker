use crate::app::{OmapMaker, protocol::AppAction};
use eframe::egui;

impl OmapMaker {
    pub fn render_generating_map_panel(&mut self, ui: &mut egui::Ui) {
        ui.heading("Generating the rest of the map.");
        ui.add_space(20.);
        ui.label("This might take some time.");
    }

    pub fn render_done_panel(&mut self, ui: &mut egui::Ui) {
        ui.heading("All done!");
        ui.label("The map is saved at: ");
        ui.label(format!(
            "{}.",
            self.gui_variables.project.save_location.display()
        ));
        ui.label("The map can be opened in OpenOrienteering Mapper for editing.");

        ui.add_space(20.);
        ui.label("If you like this application. Please star the project on Github:)");
        ui.hyperlink_to("OmapMaker on Github", "https://github.com/yvind/")
            .on_hover_text("https://github.com/yvind/");

        ui.add_space(20.);
        if ui.button("Start a new map").clicked() {
            self.dispatch_action(AppAction::Reset);
        }
    }
}
