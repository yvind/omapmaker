mod epsg_list;
mod error;
pub mod gui;

use eframe::egui;
use gui::OmapMaker;
use std::sync::Arc;

fn main() {
    // if the program is run without any command line arguments
    // start the gui version
    // else start the CLI version
    if std::env::args().len() == 1 {
        start_gui();
    } else {
        start_clap();
    }
}

fn start_gui() {
    let icon_bytes: &[u8] = include_bytes!("../assets/icon.data");
    let rgba = icon_bytes.to_vec();

    let icon = eframe::egui::IconData {
        rgba,
        width: 64,
        height: 64,
    };

    let options = eframe::NativeOptions {
        viewport: egui::ViewportBuilder {
            min_inner_size: Some(egui::vec2(800., 600.)),
            icon: Some(Arc::new(icon)),
            ..Default::default()
        },
        ..Default::default()
    };

    eframe::run_native(
        "OmapMaker",
        options,
        Box::new(|cc| Ok(Box::new(OmapMaker::new(cc.egui_ctx.clone())))),
    )
    .unwrap();
}

fn start_clap() {
    laz2omap::run_from_cli();
}
