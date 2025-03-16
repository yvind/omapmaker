pub mod comms;
pub mod drawable_omap;
pub mod error;
pub mod geometry;
pub mod parameters;
pub mod project;
pub mod raster;
pub mod steps;

// must be constant across training and inference if AI is to be applied
const TILE_SIZE_USIZE: usize = 128;
const MIN_NEIGHBOUR_MARGIN_USIZE: usize = 14;
const INV_CELL_SIZE_USIZE: usize = 2; // test 1, 2 or 4
const SIMPLIFICATION_DIST: f64 = 0.1;
pub const MERGE_DELTA: f64 = 0.1;

pub const STACK_SIZE: usize = 10; // thread stack size in MiB

const SIDE_LENGTH: usize = INV_CELL_SIZE_USIZE * TILE_SIZE_USIZE;
const CELL_SIZE: f64 = 1. / INV_CELL_SIZE_USIZE as f64;
const TILE_SIZE: f64 = TILE_SIZE_USIZE as f64;
const MIN_NEIGHBOUR_MARGIN: f64 = MIN_NEIGHBOUR_MARGIN_USIZE as f64;

pub use drawable_omap::DrawableOmap;

pub use error::{Error, Result};
