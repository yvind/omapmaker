use crate::{
    geometry::PointCloud,
    map_gen::{
        egui_map::MapObject,
        pipeline::{self, PipelineSteps, PreparedTile},
    },
    parameters::MapParameters,
    statistics::LidarStats,
};

use geo::{Polygon, Rect};

pub fn compute_map_objects(
    args: &MapParameters,
    ground_cloud: PointCloud,
    stats: &LidarStats,
    convex_hull: Polygon,
    cut_bounds: Rect,
) -> crate::Result<Vec<MapObject>> {
    let Some(tile) = PreparedTile::from_cloud(ground_cloud, stats, convex_hull, cut_bounds)? else {
        return Ok(Vec::new());
    };
    Ok(pipeline::compute_tile(
        &tile,
        args,
        PipelineSteps {
            basemap: true,
            contours: true,
            openness: true,
            vegetation: true,
            cliffs: true,
            intensity: true,
        },
        false,
    )?
    .objects)
}
