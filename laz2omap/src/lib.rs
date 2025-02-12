//mod c2hm;
pub mod comms;
mod drawable_omap;
mod error;
pub mod geometry;
mod make_map;
mod matrix;
pub mod params;
mod raster;
mod run_backend;
mod steps;

// must be constant across training and inference if AI is to be applied
const TILE_SIZE_USIZE: usize = 128;
const MIN_NEIGHBOUR_MARGIN_USIZE: usize = 14;
const INV_CELL_SIZE_USIZE: usize = 2; // test 1, 2 or 4
const STACK_SIZE: usize = 10; // thread stack size in MiB
const SIMPLIFICATION_DIST: f64 = 0.1;

const CELL_SIZE: f64 = 1. / INV_CELL_SIZE_USIZE as f64;
const TILE_SIZE: f64 = TILE_SIZE_USIZE as f64;
const MIN_NEIGHBOUR_MARGIN: f64 = MIN_NEIGHBOUR_MARGIN_USIZE as f64;

pub use drawable_omap::DrawableOmap;
pub use run_backend::OmapGenerator;

pub use error::{Error, Result};
pub use make_map::make_map;
