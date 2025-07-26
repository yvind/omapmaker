use crate::{MIN_NEIGHBOUR_MARGIN, TILE_SIZE, TILE_SIZE_USIZE};

use geo::{Coord, Rect};

pub fn retile_bounds(
    bounds: &Rect,
    neighbour_file_margin: &Rect,
) -> (Vec<Rect>, Vec<Rect>, usize, usize) {
    let x_range = bounds.max().x - bounds.min().x;
    let y_range = bounds.max().y - bounds.min().y;

    let num_x_tiles = ((x_range - MIN_NEIGHBOUR_MARGIN) / (TILE_SIZE - MIN_NEIGHBOUR_MARGIN))
        .ceil()
        .max(2.0) as usize;
    let num_y_tiles = ((y_range - MIN_NEIGHBOUR_MARGIN) / (TILE_SIZE - MIN_NEIGHBOUR_MARGIN))
        .ceil()
        .max(2.0) as usize;

    let neighbour_margin_x =
        ((num_x_tiles * TILE_SIZE_USIZE) as f64 - x_range) / (num_x_tiles - 1) as f64;
    let neighbour_margin_y =
        ((num_y_tiles * TILE_SIZE_USIZE) as f64 - y_range) / (num_y_tiles - 1) as f64;

    let mut bb: Vec<Rect> = Vec::with_capacity(num_x_tiles * num_y_tiles);
    let mut cut_bounds: Vec<Rect> = Vec::with_capacity(num_x_tiles * num_y_tiles);

    for yi in 0..num_y_tiles {
        for xi in 0..num_x_tiles {
            let mut tile_min = Coord::zero();
            let mut tile_max = Coord::zero();

            let mut inner_min = Coord::zero();
            let mut inner_max = Coord::zero();

            if yi == 0 {
                // no neighbor above
                tile_max.y = bounds.max().y;
                tile_min.y = tile_max.y - TILE_SIZE;

                inner_max.y = bounds.max().y - neighbour_file_margin.max().y;
                inner_min.y = tile_min.y + neighbour_margin_y / 2.;
            } else if yi == num_y_tiles - 1 {
                // no neighbor below
                tile_min.y = bounds.min().y;
                tile_max.y = tile_min.y + TILE_SIZE;

                inner_min.y = bounds.min().y - neighbour_file_margin.min().y;
                inner_max.y = tile_max.y - neighbour_margin_y / 2.;
            } else {
                tile_max.y = bounds.max().y - (TILE_SIZE - neighbour_margin_y) * yi as f64;
                tile_min.y = tile_max.y - TILE_SIZE;

                inner_max.y = tile_max.y - neighbour_margin_y / 2.;
                inner_min.y = tile_min.y + neighbour_margin_y / 2.;
            }
            if xi == 0 {
                // no neighbor to the left
                tile_min.x = bounds.min().x;
                tile_max.x = tile_min.x + TILE_SIZE;

                inner_min.x = bounds.min().x - neighbour_file_margin.min().x;
                inner_max.x = tile_max.x - neighbour_margin_x / 2.;
            } else if xi == num_x_tiles - 1 {
                // no neighbor to the right
                tile_max.x = bounds.max().x;
                tile_min.x = tile_max.x - TILE_SIZE;

                inner_max.x = bounds.max().x - neighbour_file_margin.max().x;
                inner_min.x = tile_min.x + neighbour_margin_x / 2.;
            } else {
                tile_min.x = bounds.min().x + (TILE_SIZE - neighbour_margin_x) * xi as f64;
                tile_max.x = tile_min.x + TILE_SIZE;

                inner_min.x = tile_min.x + neighbour_margin_x / 2.;
                inner_max.x = tile_max.x - neighbour_margin_x / 2.;
            }

            bb.push(Rect::new(tile_min, tile_max));
            cut_bounds.push(Rect::new(inner_min, inner_max));
        }
    }
    (bb, cut_bounds, num_x_tiles, num_y_tiles)
}
