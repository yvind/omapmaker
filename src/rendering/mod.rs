//! OMAP vector preview rendering, maintained as part of OmapMaker.
//!
//! Adapted from the private `omap-projects/omap-render` renderer and the
//! paper-geometry and colour helpers in `omap-projects/omap-paper`. This module
//! contains the object loader, vector painter and projection helpers used by
//! the preview. It uses the public `omap` crate and OmapMaker's existing Walkers
//! dependency; it does not require the private workspace or its sample maps.

mod color;
pub mod geo;
pub mod paint;
mod paper;
pub mod render;
