use crate::{MIN_TILE_OVERLAP_METERS, TILE_SIZE_METERS, TILE_SIZE_METERS_USIZE};

pub fn retile_bounds(bounds: &geo::Rect) -> (Vec<geo::Rect>, Vec<geo::Rect>, usize, usize) {
    let x_range = bounds.max().x - bounds.min().x;
    let y_range = bounds.max().y - bounds.min().y;
    let outer_cut_margin = 2. * crate::CELL_SIZE_METERS;

    let num_x_tiles = ((x_range - MIN_TILE_OVERLAP_METERS)
        / (TILE_SIZE_METERS - MIN_TILE_OVERLAP_METERS))
        .ceil()
        .max(2.0) as usize;
    let num_y_tiles = ((y_range - MIN_TILE_OVERLAP_METERS)
        / (TILE_SIZE_METERS - MIN_TILE_OVERLAP_METERS))
        .ceil()
        .max(2.0) as usize;

    let tile_overlap_x =
        ((num_x_tiles * TILE_SIZE_METERS_USIZE) as f64 - x_range) / (num_x_tiles - 1) as f64;
    let tile_overlap_y =
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
                tile_max.y = bounds.max().y;
                tile_min.y = tile_max.y - TILE_SIZE_METERS;

                inner_max.y = bounds.max().y - outer_cut_margin;
                inner_min.y = tile_min.y + tile_overlap_y / 2.;
            } else if yi == num_y_tiles - 1 {
                tile_min.y = bounds.min().y;
                tile_max.y = tile_min.y + TILE_SIZE_METERS;

                inner_min.y = bounds.min().y + outer_cut_margin;
                inner_max.y = tile_max.y - tile_overlap_y / 2.;
            } else {
                tile_max.y = bounds.max().y - (TILE_SIZE_METERS - tile_overlap_y) * yi as f64;
                tile_min.y = tile_max.y - TILE_SIZE_METERS;

                inner_max.y = tile_max.y - tile_overlap_y / 2.;
                inner_min.y = tile_min.y + tile_overlap_y / 2.;
            }
            if xi == 0 {
                tile_min.x = bounds.min().x;
                tile_max.x = tile_min.x + TILE_SIZE_METERS;

                inner_min.x = bounds.min().x + outer_cut_margin;
                inner_max.x = tile_max.x - tile_overlap_x / 2.;
            } else if xi == num_x_tiles - 1 {
                tile_max.x = bounds.max().x;
                tile_min.x = tile_max.x - TILE_SIZE_METERS;

                inner_max.x = bounds.max().x - outer_cut_margin;
                inner_min.x = tile_min.x + tile_overlap_x / 2.;
            } else {
                tile_min.x = bounds.min().x + (TILE_SIZE_METERS - tile_overlap_x) * xi as f64;
                tile_max.x = tile_min.x + TILE_SIZE_METERS;

                inner_min.x = tile_min.x + tile_overlap_x / 2.;
                inner_max.x = tile_max.x - tile_overlap_x / 2.;
            }

            bb.push(geo::Rect::new(tile_min, tile_max));
            cut_bounds.push(geo::Rect::new(inner_min, inner_max));
        }
    }
    (bb, cut_bounds, num_x_tiles, num_y_tiles)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn adjacent_subtiles_keep_overlapping_query_halos() {
        let bounds = geo::Rect::new((0., 0.), (2. * TILE_SIZE_METERS, TILE_SIZE_METERS));
        let (query_bounds, cut_bounds, nx, ny) = retile_bounds(&bounds);

        for row in 0..ny {
            for column in 0..nx - 1 {
                let left = row * nx + column;
                let right = left + 1;

                assert!(query_bounds[left].max().x > query_bounds[right].min().x);
                assert_eq!(cut_bounds[left].max().x, cut_bounds[right].min().x);
            }
        }
    }
}
