use copc_rs::{Bounds, BoundsSelection, CopcReader, LodSelection, Vector};
use geo::{Area, BooleanOps, ConvexHull, Intersects};
use las::point::Classification;

use std::path::PathBuf;

use rstar::{PointDistance, RTree, primitives::GeomWithData};

use crate::{
    Result,
    comms::{FrontendSender, messages::*},
    geometry::{MapRect, PointCloud, PointLaz},
    map_gen::{self, common, pipeline::PreparedTile},
    statistics::LidarStats,
};

pub struct InitializedMapTile {
    pub tiles: Vec<PreparedTile>,
    pub hull: geo::Polygon,
    pub ref_point: geo::Coord,
}

pub fn initialize_map_tile(
    sender: FrontendSender,
    paths: Vec<PathBuf>,
    test_area: geo::Rect,
    stats: LidarStats,
) -> Result<InitializedMapTile> {
    let _ = sender.send(FrontendTask::Log(
        "Calculating test tile rasters...".to_string(),
    ));
    let _ = sender.send(FrontendTask::ProgressBar(ProgressBar::Start));

    let (tile_bounds, cut_bounds, _nx, _ny) =
        common::retile_bounds(&test_area, &Default::default());
    let inc_size = 1. / tile_bounds.len() as f32;

    let ref_point = geo::Coord {
        x: ((test_area.min().x + test_area.max().x) / 20.).round() * 10.,
        y: ((test_area.min().y + test_area.max().y) / 20.).round() * 10.,
    };

    let mut z_range = (f64::MAX, f64::MIN);
    let mut all_hulls = Vec::with_capacity(4);
    let mut tiles = Vec::with_capacity(4);
    for (tile_bounds, cut_bounds) in tile_bounds.iter().zip(cut_bounds.iter()) {
        let cut_bounds = geo::Rect::new(cut_bounds.min() - ref_point, cut_bounds.max() - ref_point);

        let mut shifted_bounds = Bounds {
            min: Vector {
                x: tile_bounds.min().x,
                y: tile_bounds.min().y,
                z: f64::MAX,
            },
            max: Vector {
                x: tile_bounds.max().x,
                y: tile_bounds.max().y,
                z: f64::MIN,
            },
        };
        shifted_bounds.max.x -= ref_point.x;
        shifted_bounds.min.x -= ref_point.x;
        shifted_bounds.max.y -= ref_point.y;
        shifted_bounds.min.y -= ref_point.y;

        let mut points = Vec::new();
        let mut all_points = Vec::new();
        for path in &paths {
            let mut reader = CopcReader::from_path(path)?;
            let header_bounds = reader.header().bounds();
            if !geo::Rect::from_bounds(header_bounds).intersects(tile_bounds) {
                continue;
            }

            shifted_bounds.min.z = shifted_bounds.min.z.min(header_bounds.min.z);
            shifted_bounds.max.z = shifted_bounds.max.z.max(header_bounds.max.z);

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

            for mut p in reader.points(LodSelection::All, BoundsSelection::Within(bounds))? {
                if p.is_withheld {
                    continue;
                }

                p.x -= ref_point.x;
                p.y -= ref_point.y;
                let point = PointLaz(p);

                if point.0.classification == Classification::Ground
                    || point.0.classification == Classification::Water
                {
                    points.push(point.clone());
                }
                all_points.push(point);
            }
        }

        if points.is_empty() {
            // anyhow::bail!("A selected test tile did not contain any ground or water points");
            continue;
        }

        let all_point_cloud = PointCloud::new(all_points, shifted_bounds);
        let mut ground_point_cloud = PointCloud::new(points, shifted_bounds);

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

        let hull =
            ground_point_cloud.bounded_convex_hull(&dfm_bounds, crate::CELL_SIZE_METERS * 2.)?;

        let cut_overlay = hull
            .intersection(&cut_bounds.to_polygon())
            .into_iter()
            .max_by_key(|p| (p.signed_area() * 1000.) as u64);

        let Some(cut_overlay) = cut_overlay else {
            anyhow::bail!("The cut overlay does not overlap with the pointcloud convex hull")
        };

        let dfms = map_gen::common::compute_dfms(
            ground_point_cloud,
            &stats,
            &all_point_cloud,
            cut_bounds,
        )?;

        if z_range.0 > dfms.z_range.0 {
            z_range.0 = dfms.z_range.0;
        }
        if z_range.1 < dfms.z_range.1 {
            z_range.1 = dfms.z_range.1;
        }

        tiles.push(PreparedTile::new(dfms, hull.clone(), cut_overlay));
        all_hulls.push(hull);

        let _ = sender.send(FrontendTask::ProgressBar(ProgressBar::Inc(inc_size)));
    }

    if all_hulls.is_empty() {
        anyhow::bail!("No tile hulls were initialized");
    }
    let super_hull = geo::MultiPolygon(all_hulls).convex_hull();
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
