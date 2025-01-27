//mod c2hm;
pub mod comms;
pub mod drawing;
pub mod geometry;
mod matrix;
mod parser;
mod raster;
mod run_cli;
mod run_gui;
mod steps;

// must be constant across training and inference if AI is to be applied
const TILE_SIZE_USIZE: usize = 128;
const MIN_NEIGHBOUR_MARGIN_USIZE: usize = 14;
const INV_CELL_SIZE_USIZE: usize = 2; // test 1, 2 or 4
const BEZIER_ERROR: f64 = 0.4;
const STACK_SIZE: usize = 10; // thread stack size in MiB

const CELL_SIZE: f64 = 1. / INV_CELL_SIZE_USIZE as f64;
const TILE_SIZE: f64 = TILE_SIZE_USIZE as f64;
const MIN_NEIGHBOUR_MARGIN: f64 = MIN_NEIGHBOUR_MARGIN_USIZE as f64;

pub fn run_from_cli() {
    crate::run_cli::run_cli();
}

pub use run_gui::OmapGenerator;
