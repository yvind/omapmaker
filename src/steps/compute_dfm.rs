#![allow(dead_code)]

use crate::geometry::{Coord, LineString, PointCloud, Polygon};
use crate::raster::{Dfm, FieldType};
use crate::{INV_CELL_SIZE_USIZE, TILE_SIZE_USIZE};

const SIDE_LENGTH: usize = TILE_SIZE_USIZE * INV_CELL_SIZE_USIZE;

use geo::Contains;
use kiddo::{immutable::float::kdtree::ImmutableKdTree, SquaredEuclidean};

pub fn compute_dfms(pc: &PointCloud, ch: &LineString, tl: Coord) -> (Dfm, Dfm, Dfm, Dfm, Dfm, Dfm) {
    let mut dem = Dfm::new(tl);
    let mut grad_dem = dem.clone();
    let mut drm = dem.clone();
    let mut grad_drm = dem.clone();
    let mut dim = dem.clone();
    let mut grad_dim = dem.clone();

    let pt: ImmutableKdTree<f64, usize, 2, 32> = ImmutableKdTree::new_from_slice(&pc.to_2d_slice());

    let num_neighbours = 32;

    let pch = Polygon::new(ch.clone(), vec![]);

    for y_index in 0..SIDE_LENGTH {
        for x_index in 0..SIDE_LENGTH {
            let coords = dem.index2coord(x_index, y_index);

            if !pch.contains(&coords) {
                continue;
            }

            // slow due to very many lookups
            let nearest_n = pt.nearest_n::<SquaredEuclidean>(&[coords.x, coords.y], num_neighbours);
            let neighbours: Vec<usize> = nearest_n.iter().map(|n| n.item).collect();

            // slow due to matrix inversion
            // gradients are almost for free
            let (elev, grad_elev) =
                pc.interpolate_field(FieldType::Elevation, &neighbours, &coords, 5.);
            let (intens, grad_intens) =
                pc.interpolate_field(FieldType::Intensity, &neighbours, &coords, 5.);
            let (rn, grad_rn) =
                pc.interpolate_field(FieldType::ReturnNumber, &neighbours, &coords, 5.);

            dem[(y_index, x_index)] = elev;
            grad_dem[(y_index, x_index)] = grad_elev;
            drm[(y_index, x_index)] = rn;
            grad_drm[(y_index, x_index)] = grad_rn;
            dim[(y_index, x_index)] = intens;
            grad_dim[(y_index, x_index)] = grad_intens;
        }
    }

    (dem, grad_dem, drm, grad_drm, dim, grad_dim)
}
