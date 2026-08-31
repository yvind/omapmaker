mod channel;
mod controller;
pub(crate) mod protocol;
mod state;
mod ui;
mod worker;

pub(crate) use channel::OmapComms;
pub(crate) use controller::OmapMaker;
pub(crate) use state::{AppState, ProcessStage};
pub(crate) use ui::modals::OmapModal;
pub(crate) use ui::tile_sources;
pub(crate) use ui::{DrawOrder, DrawableOmap};

use eframe::egui;
use std::sync::Arc;

pub fn run() -> anyhow::Result<()> {
    let options = eframe::NativeOptions {
        viewport: egui::ViewportBuilder {
            min_inner_size: Some(egui::vec2(1000., 800.)),
            icon: Some(Arc::new(egui::IconData {
                rgba: include_bytes!("../assets/icon.raw").to_vec(),
                width: 64,
                height: 64,
            })),
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
