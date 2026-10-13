//! Matches Walkers' affine screen projection over normalized Mercator geometry.

use eframe::egui::{Pos2, Rect};
use walkers::Position;

use crate::rendering::geo::{Mercator, to_mercator};

#[derive(Clone, Copy)]
pub struct Camera {
    anchor: Mercator,
    total_px: f64,
}

impl Camera {
    /// Anchor the view at a geographic position and Walkers' world-pixel width.
    pub fn at(position: Position, total_px: f64) -> Self {
        Self {
            anchor: to_mercator(position),
            total_px,
        }
    }

    fn unproject(&self, screen_point: Pos2, anchor_px: Pos2) -> Mercator {
        Mercator {
            x: self.anchor.x + (screen_point.x - anchor_px.x) as f64 / self.total_px,
            y: self.anchor.y + (screen_point.y - anchor_px.y) as f64 / self.total_px,
        }
    }

    pub fn to_screen(&self, anchor_px: Pos2) -> impl Fn(Mercator) -> Pos2 + Copy {
        let (anchor, total_px) = (self.anchor, self.total_px);
        move |p: Mercator| {
            Pos2::new(
                anchor_px.x + ((p.x - anchor.x) * total_px) as f32,
                anchor_px.y + ((p.y - anchor.y) * total_px) as f32,
            )
        }
    }

    pub fn view_bounds(&self, rect: Rect, anchor_px: Pos2) -> (Mercator, Mercator) {
        let bounds = geo::Rect::new(
            self.unproject(rect.min, anchor_px),
            self.unproject(rect.max, anchor_px),
        );
        (bounds.min(), bounds.max())
    }
}
