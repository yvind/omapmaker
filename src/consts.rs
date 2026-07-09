// must be constant across training and inference if AI is to be applied
pub const TILE_SIZE_METERS_USIZE: usize = 256;
pub const MIN_NEIGHBOR_MARGIN_METERS_USIZE: usize = 14;
pub const INV_CELL_SIZE_METERS_USIZE: usize = 2; // test 1, 2 or 4
pub const SIMPLIFICATION_DIST: f64 = 0.1;
pub const MERGE_DELTA: f64 = 0.1;

pub const MIN_GRAD_LENGTH: f64 = 1.0;

pub const TILE_SIZE_PIXELS: usize = INV_CELL_SIZE_METERS_USIZE * TILE_SIZE_METERS_USIZE;
pub const CELL_SIZE_METERS: f64 = 1. / INV_CELL_SIZE_METERS_USIZE as f64;
pub const TILE_SIZE_METERS: f64 = TILE_SIZE_METERS_USIZE as f64;
pub const MIN_NEIGHBOR_MARGIN_METERS: f64 = MIN_NEIGHBOR_MARGIN_METERS_USIZE as f64;

pub const ADJUSTMENT_TILE_SIZE_METERS: f64 = 480.;
