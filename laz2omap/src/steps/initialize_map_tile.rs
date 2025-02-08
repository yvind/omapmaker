use copc_rs::{Bounds, BoundsSelection, CopcReader, LodSelection, Vector};
use fastrand::f64 as random;
use geo::{BooleanOps, Contains, Coord, Polygon, Rect};
use kiddo::{immutable::float::kdtree::ImmutableKdTree, SquaredEuclidean};
use las::point::Classification;

use std::{path::PathBuf, sync::mpsc::Sender};

use crate::{
    comms::messages::*,
    geometry::{MapRect, PointCloud},
    raster::{Dfm, FieldType},
};

use crate::{INV_CELL_SIZE_USIZE, TILE_SIZE_USIZE};
const SIDE_LENGTH: usize = TILE_SIZE_USIZE * INV_CELL_SIZE_USIZE;

pub fn initialize_map_tile(
    sender: Sender<FrontendTask>,
    path: PathBuf,
    tile_indecies: [Option<usize>; 9],
) -> (
    Vec<Dfm>,
    Vec<Dfm>,
    Vec<Dfm>,
    Vec<Polygon>,
    Polygon,
    Coord,
    (f64, f64),
) {
    sender
        .send(FrontendTask::Log(
            "Calculating test tile rasters".to_string(),
        ))
        .unwrap();
    sender
        .send(FrontendTask::ProgressBar(ProgressBar::Start))
        .unwrap();

    let tile_indecies = tile_indecies.into_iter().flatten().collect::<Vec<usize>>();

    let inc_size = 1. / tile_indecies.len() as f32;

    let mut reader = CopcReader::from_path(&path).unwrap();
    let header_bounds = reader.header().bounds();

    let ref_point = Coord {
        x: ((header_bounds.min.x + header_bounds.max.x) / 20.).round() * 10.,
        y: ((header_bounds.min.y + header_bounds.max.y) / 20.).round() * 10.,
    };

    let z_range = (header_bounds.min.z, header_bounds.max.z);

    let (all_tile_bounds, all_cut_bounds, _, _) = crate::steps::retile_bounds(
        &Rect::from_bounds(header_bounds),
        &Rect::new(Coord { x: 0., y: 0. }, Coord { x: 0., y: 0. }),
    );

    let mut cut_bounds = Vec::with_capacity(9);
    let mut all_hulls = Vec::with_capacity(9);
    let mut dems = Vec::with_capacity(9);
    let mut g_dems = Vec::with_capacity(9);
    let mut drms = Vec::with_capacity(9);
    for ti in tile_indecies.iter() {
        let tile_bounds = all_tile_bounds[*ti];
        cut_bounds.push(
            Rect::new(
                all_cut_bounds[*ti].max() - ref_point,
                all_cut_bounds[*ti].min() - ref_point,
            )
            .into(),
        );

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

        let hull = Polygon::new(hull, vec![]);

        let tl = Coord {
            x: dims.min.x,
            y: dims.max.y,
        };

        let mut dem = Dfm::new(tl);
        let mut drm = dem.clone();
        let mut grad_dem = dem.clone();

        let pt: ImmutableKdTree<f64, usize, 2, 32> =
            ImmutableKdTree::new_from_slice(&point_cloud.to_2d_slice());

        for y_index in 0..SIDE_LENGTH {
            for x_index in 0..SIDE_LENGTH {
                let coords = dem.index2coord(x_index, y_index);

                if !hull.contains(&coords) {
                    continue;
                }

                // slow due to very many lookups
                let nearest_n = pt.nearest_n::<SquaredEuclidean>(&[coords.x, coords.y], 32);
                let neighbours: Vec<usize> = nearest_n.iter().map(|n| n.item).collect();

                // slow due to matrix inversion
                // gradients are almost for free
                let (elev, grad_elev) =
                    point_cloud.interpolate_field(FieldType::Elevation, &neighbours, &coords, 5.);
                let (ret, _) = point_cloud.interpolate_field(
                    FieldType::ReturnNumber,
                    &neighbours,
                    &coords,
                    10.,
                );

                dem[(y_index, x_index)] = elev;
                grad_dem[(y_index, x_index)] = grad_elev;
                drm[(y_index, x_index)] = (ret - 1.) / 5.; // want a range between 0-1 and this basic algo does not do that
            }
        }

        all_hulls.push(hull);
        dems.push(dem);
        g_dems.push(grad_dem);
        drms.push(drm);

        sender
            .send(FrontendTask::ProgressBar(ProgressBar::Inc(inc_size)))
            .unwrap();
    }

    let initial = all_hulls[0].clone();
    let super_hull = all_hulls
        .into_iter()
        .skip(1)
        .fold(initial, |acc, p| acc.union(&p).0[0].clone());

    sender
        .send(FrontendTask::ProgressBar(ProgressBar::Finish))
        .unwrap();
    sender
        .send(FrontendTask::TaskComplete(TaskDone::InitializeMapTile))
        .unwrap();

    (
        dems, g_dems, drms, cut_bounds, super_hull, ref_point, z_range,
    )
}
