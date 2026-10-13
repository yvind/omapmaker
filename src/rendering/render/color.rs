//! Paint-ready colour, as a thin `Color32` wrapper over `omap-paper`'s own
//! `omap`-aware colour-table resolution — see there for how a `SymbolColor`
//! or a whole symbol tree resolves to a colour and a draw-order priority.

use eframe::egui::Color32;
use omap::colors::{ColorSet, Rgb, SymbolColor};

pub(super) use crate::rendering::color::min_prio;

fn rgb_to_color32(rgb: Rgb) -> Color32 {
    Color32::from_rgb(
        (rgb.r.get() * 255.0).round() as u8,
        (rgb.g.get() * 255.0).round() as u8,
        (rgb.b.get() * 255.0).round() as u8,
    )
}

/// Resolve a [`SymbolColor`] into an RGB colour and its priority index in the
/// colour set, for actual painting.
pub(super) fn resolve_color(sc: &SymbolColor, colors: &ColorSet) -> Option<(Color32, i32)> {
    crate::rendering::color::resolve_color(sc, colors).map(|(rgb, p)| (rgb_to_color32(rgb), p))
}
