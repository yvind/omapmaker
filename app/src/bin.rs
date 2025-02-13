mod assets;
mod gui;
mod omap_generator;
mod omap_maker;

pub use omap_generator::OmapGenerator;
pub use omap_maker::OmapMaker;

use eframe::egui;
use std::sync::Arc;

const STACK_SIZE: usize = 10; // thread stack size in MiB

fn main() {
    let icon_bytes: &[u8] = include_bytes!("./assets/icon.data");
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
