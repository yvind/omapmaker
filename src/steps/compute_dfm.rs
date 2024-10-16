#![allow(dead_code)]

use crate::geometry::{Line, Point2D, PointCloud};
use crate::raster::{Dfm, FieldType};

use kiddo::{immutable::float::kdtree::ImmutableKdTree, SquaredEuclidean};

pub fn compute_dfms(
    pt: &ImmutableKdTree<f64, usize, 2, 32>,
    pc: &PointCloud,
    ch: &Line,
    dem_info: (usize, usize, f64, Point2D),
) -> (Dfm, Dfm, Dfm, Dfm, Dfm, Dfm) {
    let (width, height, cell_size, tl) = dem_info;
    let mut dem = Dfm::new(width, height, tl, cell_size);
    let mut grad_dem = dem.clone();
    let mut drm = dem.clone();
    let mut grad_drm = dem.clone();
    let mut dim = dem.clone();
    let mut grad_dim = dem.clone();

    let num_neighbours = 32;

    for y_index in 0..height {
        for x_index in 0..width {
            let coords: Point2D = dem.index2coord(x_index, y_index).unwrap();
            if !ch.contains(&coords).unwrap() {
                continue;
            }

            // slow due to very many lookups
            let nearest_n = pt.nearest_n::<SquaredEuclidean>(&[coords.x, coords.y], num_neighbours);
            let neighbours: Vec<usize> = nearest_n.iter().map(|n| n.item).collect();

            // slow due to matrix inversion
            // gradients are almost for free
            let (elev, grad_elev) =
                pc.interpolate_field(FieldType::Elevation, &neighbours, &coords, 0.5);
            let (intens, grad_intens) =
                pc.interpolate_field(FieldType::Intensity, &neighbours, &coords, 1.);
            let (rn, grad_rn) =
                pc.interpolate_field(FieldType::ReturnNumber, &neighbours, &coords, 1.);

            dem.field[y_index][x_index] = elev;
            grad_dem.field[y_index][x_index] = grad_elev;
            drm.field[y_index][x_index] = rn;
            grad_drm.field[y_index][x_index] = grad_rn;
            dim.field[y_index][x_index] = intens;
            grad_dim.field[y_index][x_index] = grad_intens;
        }
    }

    (dem, grad_dem, drm, grad_drm, dim, grad_dim)
}
