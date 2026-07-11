use crate::{
    MIN_NEIGHBOR_MARGIN_METERS, TILE_SIZE_METERS, TILE_SIZE_METERS_USIZE, neighbors::Neighborhood,
};

pub fn retile_bounds(
    bounds: &geo::Rect,
    lidar_neighbors: &Neighborhood,
) -> (Vec<geo::Rect>, Vec<geo::Rect>, usize, usize) {
    let mut neighbor_file_margin = [(0., 0.), (0., 0.)];
    let mut cut_margin = [(0., 0.), (0., 0.)];
    if lidar_neighbors.has_neighbor_above() {
        neighbor_file_margin[1].1 = MIN_NEIGHBOR_MARGIN_METERS;
    } else {
        cut_margin[1].1 = 2. * crate::CELL_SIZE_METERS;
    }
    if lidar_neighbors.has_neighbor_below() {
        neighbor_file_margin[0].1 = -MIN_NEIGHBOR_MARGIN_METERS;
    } else {
        cut_margin[0].1 = -2. * crate::CELL_SIZE_METERS;
    }
    if lidar_neighbors.has_neighbor_right() {
        neighbor_file_margin[1].0 = MIN_NEIGHBOR_MARGIN_METERS;
    } else {
        cut_margin[1].0 = 2. * crate::CELL_SIZE_METERS;
    }
    if lidar_neighbors.has_neighbor_left() {
        neighbor_file_margin[0].0 = -MIN_NEIGHBOR_MARGIN_METERS;
    } else {
        cut_margin[0].0 = -2. * crate::CELL_SIZE_METERS;
    }
    let neighbor_file_margin = geo::Rect::new(neighbor_file_margin[0], neighbor_file_margin[1]);
    let cut_margin = geo::Rect::new(cut_margin[0], cut_margin[1]);

    let x_range = bounds.max().x - bounds.min().x - neighbor_file_margin.min().x
        + neighbor_file_margin.max().x;
    let y_range = bounds.max().y - bounds.min().y - neighbor_file_margin.min().y
        + neighbor_file_margin.max().y;

    let num_x_tiles = ((x_range - MIN_NEIGHBOR_MARGIN_METERS)
        / (TILE_SIZE_METERS - MIN_NEIGHBOR_MARGIN_METERS))
        .ceil()
        .max(2.0) as usize;
    let num_y_tiles = ((y_range - MIN_NEIGHBOR_MARGIN_METERS)
        / (TILE_SIZE_METERS - MIN_NEIGHBOR_MARGIN_METERS))
        .ceil()
        .max(2.0) as usize;

    let neighbor_margin_x =
        ((num_x_tiles * TILE_SIZE_METERS_USIZE) as f64 - x_range) / (num_x_tiles - 1) as f64;
    let neighbor_margin_y =
        ((num_y_tiles * TILE_SIZE_METERS_USIZE) as f64 - y_range) / (num_y_tiles - 1) as f64;

    let mut bb: Vec<geo::Rect> = Vec::with_capacity(num_x_tiles * num_y_tiles);
    let mut cut_bounds: Vec<geo::Rect> = Vec::with_capacity(num_x_tiles * num_y_tiles);

    for yi in 0..num_y_tiles {
        for xi in 0..num_x_tiles {
            let mut tile_min = geo::Coord::zero();
            let mut tile_max = geo::Coord::zero();

            let mut inner_min = geo::Coord::zero();
            let mut inner_max = geo::Coord::zero();

            if yi == 0 {
                tile_max.y = bounds.max().y + neighbor_file_margin.max().y;
                tile_min.y = tile_max.y - TILE_SIZE_METERS;

                inner_max.y = bounds.max().y - cut_margin.max().y;
                inner_min.y = tile_min.y + neighbor_margin_y / 2.;
            } else if yi == num_y_tiles - 1 {
                tile_min.y = bounds.min().y + neighbor_file_margin.min().y;
                tile_max.y = tile_min.y + TILE_SIZE_METERS;

                inner_min.y = bounds.min().y - cut_margin.min().y;
                inner_max.y = tile_max.y - neighbor_margin_y / 2.;
            } else {
                tile_max.y = bounds.max().y + neighbor_file_margin.max().y
                    - (TILE_SIZE_METERS - neighbor_margin_y) * yi as f64;
                tile_min.y = tile_max.y - TILE_SIZE_METERS;

                inner_max.y = tile_max.y - neighbor_margin_y / 2.;
                inner_min.y = tile_min.y + neighbor_margin_y / 2.;
            }
            if xi == 0 {
                tile_min.x = bounds.min().x + neighbor_file_margin.min().x;
                tile_max.x = tile_min.x + TILE_SIZE_METERS;

                inner_min.x = bounds.min().x - cut_margin.min().x;
                inner_max.x = tile_max.x - neighbor_margin_x / 2.;
            } else if xi == num_x_tiles - 1 {
                tile_max.x = bounds.max().x + neighbor_file_margin.max().x;
                tile_min.x = tile_max.x - TILE_SIZE_METERS;

                inner_max.x = bounds.max().x - cut_margin.max().x;
                inner_min.x = tile_min.x + neighbor_margin_x / 2.;
            } else {
                tile_min.x = bounds.min().x
                    + neighbor_file_margin.min().x
                    + (TILE_SIZE_METERS - neighbor_margin_x) * xi as f64;
                tile_max.x = tile_min.x + TILE_SIZE_METERS;

                inner_min.x = tile_min.x + neighbor_margin_x / 2.;
                inner_max.x = tile_max.x - neighbor_margin_x / 2.;
            }

            bb.push(geo::Rect::new(tile_min, tile_max));
            cut_bounds.push(geo::Rect::new(inner_min, inner_max));
        }
    }
    (bb, cut_bounds, num_x_tiles, num_y_tiles)
}
