#![allow(clippy::type_complexity)]

use copc_rs::{Bounds, BoundsSelection, CopcReader, LodSelection, Vector};
use fastrand::f64 as random;
use geo::{BooleanOps, ConvexHull, Coord, Polygon, Rect};
use kiddo::{immutable::float::kdtree::ImmutableKdTree, SquaredEuclidean};
use las::point::Classification;

use std::{num::NonZero, path::PathBuf, sync::mpsc::Sender};

use crate::{
    comms::messages::*,
    geometry::{MapRect, PointCloud, PointLaz},
    map_gen,
    neighbors::Neighborhood,
    raster::Dfm,
};

pub fn initialize_map_tile(
    sender: Sender<FrontendTask>,
    path: PathBuf,
    tile_indecies: Neighborhood,
) -> (
    Vec<Dfm>,
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
            "Calculating test tile rasters...".to_string(),
        ))
        .unwrap();
    sender
        .send(FrontendTask::ProgressBar(ProgressBar::Start))
        .unwrap();

    let tile_indecies = tile_indecies.all_indices();

    let inc_size = 1. / tile_indecies.len() as f32;

    let mut reader = CopcReader::from_path(&path).unwrap();
    let header_bounds = reader.header().bounds();

    let ref_point = Coord {
        x: ((header_bounds.min.x + header_bounds.max.x) / 20.).round() * 10.,
        y: ((header_bounds.min.y + header_bounds.max.y) / 20.).round() * 10.,
    };

    let (all_tile_bounds, all_cut_bounds, _, _) =
        map_gen::common::retile_bounds(&Rect::from_bounds(header_bounds), &Neighborhood::new(0));

    let mut z_range = (f64::MAX, f64::MIN);
    let mut cut_bounds = Vec::with_capacity(9);
    let mut all_hulls = Vec::with_capacity(9);
    let mut dems = Vec::with_capacity(9);
    let mut g_dems = Vec::with_capacity(9);
    let mut drms = Vec::with_capacity(9);
    let mut dims = Vec::with_capacity(9);
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
                .filter_map(|mut p| {
                    (!p.is_withheld
                        && (p.classification == Classification::Ground
                            || p.classification == Classification::Water))
                        .then(|| {
                            p.x += 2. * (random() - 0.5) / 1_000. - ref_point.x;
                            p.y += 2. * (random() - 0.5) / 1_000. - ref_point.y;
                            PointLaz(p)
                        })
                })
                .collect(),
            shifted_bounds,
        );

        // add ghost points at the corners of the bounds to make the entire dem interpolate-able
        // IDW interpolating the ghost points from the 8 closest real points
        let query_points = [
            [shifted_bounds.min.x, shifted_bounds.max.y],
            [shifted_bounds.min.x, shifted_bounds.min.y],
            [shifted_bounds.max.x, shifted_bounds.min.y],
            [shifted_bounds.max.x, shifted_bounds.max.y],
        ];
        let mut zs = [0.; 4];

        {
            let pt: ImmutableKdTree<f64, usize, 2, 32> =
                ImmutableKdTree::new_from_slice(&point_cloud.to_2d_slice());
            for (i, qp) in query_points.iter().enumerate() {
                let neighbors = pt.nearest_n::<SquaredEuclidean>(qp, NonZero::new(4).unwrap());
                let tot_weight = neighbors.iter().fold(0., |acc, n| acc + 1. / n.distance);

                zs[i] = neighbors
                    .iter()
                    .fold(0., |acc, n| acc + point_cloud[n.item].0.z / n.distance)
                    / tot_weight;
            }
        }

        point_cloud.add(vec![
            PointLaz::new(query_points[0][0], query_points[0][1], zs[0]),
            PointLaz::new(query_points[1][0], query_points[1][1], zs[1]),
            PointLaz::new(query_points[2][0], query_points[2][1], zs[2]),
            PointLaz::new(query_points[3][0], query_points[3][1], zs[3]),
        ]);

        let dfm_bounds = point_cloud.get_dfm_dimensions();

        let hull = point_cloud.bounded_convex_hull(&dfm_bounds, crate::CELL_SIZE * 2.);

        let (dem, drm, dim, tile_z_range) = map_gen::common::compute_dfms(point_cloud);
        let grad_dem = dem.slope(3);

        if z_range.0 > tile_z_range.0 {
            z_range.0 = tile_z_range.0;
        }
        if z_range.1 < tile_z_range.1 {
            z_range.1 = tile_z_range.1;
        }

        all_hulls.push(hull);
        dems.push(dem);
        g_dems.push(grad_dem);
        drms.push(drm);
        dims.push(dim);

        sender
            .send(FrontendTask::ProgressBar(ProgressBar::Inc(inc_size)))
            .unwrap();
    }

    let initial = all_hulls[0].clone();
    let super_hull = all_hulls
        .into_iter()
        .skip(1)
        .fold(initial, |acc, p| acc.union(&p).0[0].clone());
    let super_hull = super_hull.convex_hull();

    sender
        .send(FrontendTask::ProgressBar(ProgressBar::Finish))
        .unwrap();
    sender
        .send(FrontendTask::TaskComplete(TaskDone::InitializeMapTile))
        .unwrap();

    (
        dems, g_dems, drms, dims, cut_bounds, super_hull, ref_point, z_range,
    )
}
