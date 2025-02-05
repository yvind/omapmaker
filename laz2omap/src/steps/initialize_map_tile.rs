use copc_rs::{Bounds, BoundsSelection, CopcReader, LodSelection, Vector};
use fastrand::f64 as random;
use geo::{Coord, LineString, Rect};
use kiddo::{immutable::float::kdtree::ImmutableKdTree, SquaredEuclidean};
use las::{point::Classification, Reader};

use std::{
    path::{Path, PathBuf},
    sync::mpsc::Sender,
};

use crate::{
    comms::messages::*,
    geometry::PointCloud,
    raster::{Dfm, FieldType},
};

use crate::{INV_CELL_SIZE_USIZE, MIN_NEIGHBOUR_MARGIN, TILE_SIZE, TILE_SIZE_USIZE};
const SIDE_LENGTH: usize = TILE_SIZE_USIZE * INV_CELL_SIZE_USIZE;

pub fn initialize_map_tile(
    sender: Sender<FrontendTask>,
    path: PathBuf,
) -> (Dfm, Dfm, LineString, Coord, (f64, f64)) {
    sender
        .send(FrontendTask::Log(
            "Calculating test tile rasters".to_string(),
        ))
        .unwrap();
    sender.send(FrontendTask::StartProgressBar).unwrap();

    let inc_size = 1. / SIDE_LENGTH as f32;

    // get a central tile from the lidar file
    let tile_bounds = get_tile_from_laz(&path);

    let mut reader = CopcReader::from_path(&path).unwrap();
    let header_bounds = reader.header().bounds();

    let bounds = Bounds {
        min: Vector {
            x: tile_bounds.min().x,
            y: tile_bounds.min().y,
            z: header_bounds.min.z,
        },
        max: Vector {
            x: tile_bounds.max().x,
            y: tile_bounds.max().y,
            z: header_bounds.max.z,
        },
    };

    let ref_point = Coord {
        x: ((tile_bounds.min().x + tile_bounds.max().x) / 20.).round() * 10.,
        y: ((tile_bounds.min().y + tile_bounds.max().y) / 20.).round() * 10.,
    };

    let mut shifted_bounds = bounds;
    shifted_bounds.max.x -= ref_point.x;
    shifted_bounds.min.x -= ref_point.x;
    shifted_bounds.max.y -= ref_point.y;
    shifted_bounds.min.y -= ref_point.y;

    let mut point_cloud = PointCloud::new(
        reader
            .points(LodSelection::All, BoundsSelection::Within(bounds))
            .unwrap()
            .filter_map(|p| {
                (!p.is_withheld
                    && (p.classification == Classification::Ground
                        || p.classification == Classification::Water))
                    .then(|| {
                        let mut clone = p.clone();
                        clone.x += 2. * (random() - 0.5) / 1_000. - ref_point.x;
                        clone.y += 2. * (random() - 0.5) / 1_000. - ref_point.y;
                        clone
                    })
            })
            .collect(),
        shifted_bounds,
    );

    let dims = point_cloud.get_dfm_dimensions();

    let hull = point_cloud.bounded_convex_hull(&dims, crate::CELL_SIZE * 2.);

    let z_range = (dims.min.z, dims.max.z);

    let tl = Coord {
        x: dims.min.x,
        y: dims.max.y,
    };

    let mut dem = Dfm::new(tl);
    let mut grad_dem = dem.clone();

    let pt: ImmutableKdTree<f64, usize, 2, 32> =
        ImmutableKdTree::new_from_slice(&point_cloud.to_2d_slice());

    for y_index in 0..SIDE_LENGTH {
        for x_index in 0..SIDE_LENGTH {
            let coords = dem.index2coord(x_index, y_index);

            // slow due to very many lookups
            let nearest_n = pt.nearest_n::<SquaredEuclidean>(&[coords.x, coords.y], 32);
            let neighbours: Vec<usize> = nearest_n.iter().map(|n| n.item).collect();

            // slow due to matrix inversion
            // gradients are almost for free
            let (elev, grad_elev) =
                point_cloud.interpolate_field(FieldType::Elevation, &neighbours, &coords, 5.);

            dem[(y_index, x_index)] = elev;
            grad_dem[(y_index, x_index)] = grad_elev;
        }
        sender
            .send(FrontendTask::IncrementProgressBar(inc_size))
            .unwrap();
    }
    sender.send(FrontendTask::FinishProgrssBar).unwrap();
    sender
        .send(FrontendTask::TaskComplete(TaskDone::InitializeMapTile))
        .unwrap();

    (dem, grad_dem, hull, ref_point, z_range)
}

fn get_tile_from_laz(path: &Path) -> Rect {
    // read the las-header from the file to be tiled, must be readable by now
    let header = {
        let las_reader = Reader::from_path(path).unwrap();
        las_reader.header().clone().into_raw().unwrap()
    };
    let bounds = Rect::new(
        Coord {
            x: header.min_x,
            y: header.min_y,
        },
        Coord {
            x: header.max_x,
            y: header.max_y,
        },
    );

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

    let yi = num_y_tiles / 2;
    let xi = num_x_tiles / 2;

    let mut tile_min = Coord::zero();
    let mut tile_max = Coord::zero();

    if yi == 0 {
        // no neighbour above
        tile_max.y = bounds.max().y;
        tile_min.y = tile_max.y - TILE_SIZE;
    } else if yi == num_y_tiles - 1 {
        // no neigbour below
        tile_min.y = bounds.min().y;
        tile_max.y = tile_min.y + TILE_SIZE;
    } else {
        tile_max.y = bounds.max().y - (TILE_SIZE - neighbour_margin_y) * yi as f64;
        tile_min.y = tile_max.y - TILE_SIZE;
    }
    if xi == 0 {
        // no neighbour to the left
        tile_min.x = bounds.min().x;
        tile_max.x = tile_min.x + TILE_SIZE;
    } else if xi == num_x_tiles - 1 {
        // no neigbour to the right
        tile_max.x = bounds.max().x;
        tile_min.x = tile_max.x - TILE_SIZE;
    } else {
        tile_min.x = bounds.min().x + (TILE_SIZE - neighbour_margin_x) * xi as f64;
        tile_max.x = tile_min.x + TILE_SIZE;
    }

    Rect::new(tile_min, tile_max)
}
