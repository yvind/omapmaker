use geo::ConvexHull;

use std::path::PathBuf;

use crate::{
    Result,
    generation::{self, pipeline::PreparedTile},
    lidar::{LidarSourceIndex, LidarStats},
    progress::{ProgressReporter, ProgressUpdate},
};

pub struct InitializedMapTile {
    pub tiles: Vec<PreparedTile>,
    pub hull: geo::Polygon,
    pub ref_point: geo::Coord,
}

pub fn initialize_map_tile(
    reporter: &dyn ProgressReporter,
    paths: Vec<PathBuf>,
    test_area: geo::Rect,
    stats: LidarStats,
) -> Result<InitializedMapTile> {
    reporter.log("Calculating test tile rasters...".to_string());
    reporter.progress(ProgressUpdate::Start);

    let source_index = LidarSourceIndex::new(&paths, None)?;

    let (tile_bounds, cut_bounds, _nx, _ny) = generation::pipeline::retile_bounds(&test_area);
    let inc_size = 1. / tile_bounds.len() as f32;

    let ref_point = geo::Coord {
        x: ((test_area.min().x + test_area.max().x) / 20.).round() * 10.,
        y: ((test_area.min().y + test_area.max().y) / 20.).round() * 10.,
    };
    let mut z_range = (f32::MAX, f32::MIN);
    let mut all_hulls = Vec::with_capacity(4);
    let mut tiles = Vec::with_capacity(4);
    for (tile_bounds, cut_bounds) in tile_bounds.iter().zip(cut_bounds.iter()) {
        let cut_bounds = geo::Rect::new(cut_bounds.min() - ref_point, cut_bounds.max() - ref_point);
        if !source_index.intersects(*tile_bounds) {
            continue;
        }
        let (ground_cloud, all_point_cloud, hull) =
            match crate::lidar::read_laz(&source_index, *tile_bounds, ref_point) {
                Ok(clouds) => clouds,
                Err(error)
                    if error
                        .downcast_ref::<crate::Error>()
                        .is_some_and(|error| matches!(error, crate::Error::NoGroundPoints)) =>
                {
                    continue;
                }
                Err(error) => return Err(error),
            };
        let Some(tile) = PreparedTile::from_cloud(
            ground_cloud,
            all_point_cloud,
            &stats,
            hull.clone(),
            cut_bounds,
        )?
        else {
            continue;
        };

        if z_range.0 > tile.z_range.0 {
            z_range.0 = tile.z_range.0;
        }
        if z_range.1 < tile.z_range.1 {
            z_range.1 = tile.z_range.1;
        }

        tiles.push(tile);
        all_hulls.push(hull);

        reporter.progress(ProgressUpdate::Advance(inc_size));
    }

    if all_hulls.is_empty() {
        anyhow::bail!("No tile hulls were initialized");
    }
    let super_hull = geo::MultiPolygon(all_hulls).convex_hull();
    for tile in tiles.iter_mut() {
        tile.hull = super_hull.clone();
        tile.z_range = z_range;
    }
    crate::raster::accumulate_cross_tile_flow(
        tiles.iter_mut().map(|tile| &mut tile.rasters.stream_flow),
    )?;

    reporter.progress(ProgressUpdate::Finish);

    Ok(InitializedMapTile {
        tiles,
        hull: super_hull,
        ref_point,
    })
}
