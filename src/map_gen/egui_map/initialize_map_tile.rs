use copc_rs::{Bounds, BoundsSelection, CopcReader, LodSelection, Vector};
use geo::{BooleanOps, ConvexHull};
use las::point::Classification;

use std::path::PathBuf;

use rstar::{PointDistance, RTree, primitives::GeomWithData};

use crate::{
    Result,
    comms::{FrontendSender, messages::*},
    geometry::{MapRect, PointCloud, PointLaz},
    map_gen::{self, pipeline::PreparedTile},
    neighbors::Neighborhood,
    statistics::LidarStats,
};

pub struct InitializedMapTile {
    pub tiles: Vec<PreparedTile>,
    pub hull: geo::Polygon,
    pub ref_point: geo::Coord,
}

pub fn initialize_map_tile(
    sender: FrontendSender,
    path: PathBuf,
    tile_indecies: Neighborhood,
    stats: LidarStats,
) -> Result<InitializedMapTile> {
    let _ = sender.send(FrontendTask::Log(
        "Calculating test tile rasters...".to_string(),
    ));
    let _ = sender.send(FrontendTask::ProgressBar(ProgressBar::Start));

    let tile_indecies = tile_indecies.all_indices();

    let inc_size = 1. / tile_indecies.len() as f32;

    let mut reader = CopcReader::from_path(&path)?;
    let header_bounds = reader.header().bounds();

    let ref_point = geo::Coord {
        x: ((header_bounds.min.x + header_bounds.max.x) / 20.).round() * 10.,
        y: ((header_bounds.min.y + header_bounds.max.y) / 20.).round() * 10.,
    };

    let (all_tile_bounds, all_cut_bounds, _, _) = map_gen::common::retile_bounds(
        &geo::Rect::from_bounds(header_bounds),
        &Neighborhood::new(0),
    );

    let mut z_range = (f64::MAX, f64::MIN);
    let mut all_hulls = Vec::with_capacity(9);
    let mut tiles = Vec::with_capacity(9);
    for ti in tile_indecies.iter() {
        let tile_bounds = all_tile_bounds[*ti];
        let cut_overlay = geo::Rect::new(
            all_cut_bounds[*ti].max() - ref_point,
            all_cut_bounds[*ti].min() - ref_point,
        )
        .into();

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

        let mut ground_point_cloud = PointCloud::new(
            reader
                .points(LodSelection::All, BoundsSelection::Within(bounds))?
                .filter_map(|mut p| {
                    if !p.is_withheld
                        && (p.classification == Classification::Ground
                            || p.classification == Classification::Water)
                    {
                        p.x -= ref_point.x;
                        p.y -= ref_point.y;
                        Some(PointLaz(p))
                    } else {
                        None
                    }
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

        let pt = RTree::bulk_load(
            ground_point_cloud
                .to_2d_slice()
                .into_iter()
                .enumerate()
                .map(|(index, point)| GeomWithData::new(point, index))
                .collect(),
        );

        for (i, qp) in query_points.iter().enumerate() {
            let neighbors = pt.nearest_neighbor_iter(*qp).take(4).collect::<Vec<_>>();
            let tot_weight = neighbors
                .iter()
                .fold(0., |acc, n| acc + 1. / n.distance_2(qp).max(f64::EPSILON));

            zs[i] = neighbors.iter().fold(0., |acc, n| {
                acc + ground_point_cloud[n.data].0.z / n.distance_2(qp).max(f64::EPSILON)
            }) / tot_weight;
        }

        ground_point_cloud.add(vec![
            PointLaz::new(query_points[0][0], query_points[0][1], zs[0]),
            PointLaz::new(query_points[1][0], query_points[1][1], zs[1]),
            PointLaz::new(query_points[2][0], query_points[2][1], zs[2]),
            PointLaz::new(query_points[3][0], query_points[3][1], zs[3]),
        ]);

        let dfm_bounds = ground_point_cloud.get_dfm_dimensions();

        let hull = ground_point_cloud.bounded_convex_hull(&dfm_bounds, crate::CELL_SIZE * 2.)?;

        let (dem, return_number, intensity, tile_z_range) =
            map_gen::common::compute_dfms(ground_point_cloud, &stats)?;

        if z_range.0 > tile_z_range.0 {
            z_range.0 = tile_z_range.0;
        }
        if z_range.1 < tile_z_range.1 {
            z_range.1 = tile_z_range.1;
        }

        tiles.push(PreparedTile::new(
            dem,
            return_number,
            intensity,
            hull.clone(),
            cut_overlay,
            tile_z_range,
        ));
        all_hulls.push(hull);

        let _ = sender.send(FrontendTask::ProgressBar(ProgressBar::Inc(inc_size)));
    }

    let Some(initial) = all_hulls.first().cloned() else {
        anyhow::bail!("No tile hulls were initialized");
    };
    let super_hull = all_hulls
        .into_iter()
        .skip(1)
        .fold(initial, |acc, p| acc.union(&p).0[0].clone());
    let super_hull = super_hull.convex_hull();
    for tile in tiles.iter_mut() {
        tile.hull = super_hull.clone();
        tile.z_range = z_range;
    }

    let _ = sender.send(FrontendTask::ProgressBar(ProgressBar::Finish));
    let _ = sender.send(FrontendTask::TaskComplete(TaskDone::InitializeMapTile));

    Ok(InitializedMapTile {
        tiles,
        hull: super_hull,
        ref_point,
    })
}
