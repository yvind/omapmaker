// must be constant across training and inference if AI is to be applied
pub const TILE_SIZE_USIZE: usize = 128;
pub const MIN_NEIGHBOUR_MARGIN_USIZE: usize = 14;
pub const INV_CELL_SIZE_USIZE: usize = 2; // test 1, 2 or 4
pub const SIMPLIFICATION_DIST: f64 = 0.1;
pub const MERGE_DELTA: f64 = 0.1;

pub const STACK_SIZE: usize = 10; // thread stack size in MiB

pub const MIN_GRAD_LENGTH: f64 = 1.0;

pub const SIDE_LENGTH: usize = INV_CELL_SIZE_USIZE * TILE_SIZE_USIZE;
pub const CELL_SIZE: f64 = 1. / INV_CELL_SIZE_USIZE as f64;
pub const TILE_SIZE: f64 = TILE_SIZE_USIZE as f64;
pub const MIN_NEIGHBOUR_MARGIN: f64 = MIN_NEIGHBOUR_MARGIN_USIZE as f64;
