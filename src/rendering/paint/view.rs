use eframe::egui::Pos2;

use super::tuning::RenderTuning;
use crate::rendering::geo::Mercator;

/// How a frame of primitives is drawn.
///
/// `min` and `max` are the viewport's geographic bounds, computed once per frame
/// by the caller and used to skip primitives outside them with a plain float
/// comparison — no reprojection needed even for primitives that stay off-screen.
/// `to_screen` maps a geographic position to the screen and `mm_to_px` scales an
/// on-paper millimetre width or radius to pixels.
pub struct View<'a, F: Fn(Mercator) -> Pos2> {
    pub min: Mercator,
    pub max: Mercator,
    pub to_screen: F,
    pub mm_to_px: f32,
    pub tuning: &'a RenderTuning,
}
