//#![windows_subsystem = "windows"]

mod assets;
mod backend;
mod comms;
mod consts;
mod convert_copc;
mod drawable;
mod error;
mod frontend;
mod geometry;
mod gui;
mod map_gen;
mod neighbors;
mod parameters;
mod parse_crs;
mod project;
mod raster;

pub use consts::*;
pub use error::*;
use frontend::OmapMaker;

use eframe::egui;
use std::sync::Arc;

fn main() {
    let icon_bytes = include_bytes!("./assets/icon.raw");
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
