//#![windows_subsystem = "windows"] // removes the background terminal that opens with the program

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
mod statistics;

pub use consts::*;
pub use error::*;
use frontend::OmapMaker;

use eframe::egui;
use std::sync::Arc;

fn main() -> anyhow::Result<()> {
    let icon = eframe::egui::IconData {
        rgba: include_bytes!("./assets/icon.raw").to_vec(),
        width: 64,
        height: 64,
    };

    let options = eframe::NativeOptions {
        viewport: egui::ViewportBuilder {
            min_inner_size: Some(egui::vec2(1000., 800.)),
            icon: Some(Arc::new(icon)),
            ..Default::default()
        },
        ..Default::default()
    };

    eframe::run_native(
        "OmapMaker",
        options,
        Box::new(|cc| Ok(Box::new(OmapMaker::new(cc.egui_ctx.clone())))),
    )?;

    Ok(())
}
