use crate::{
    map_gen::{
        egui_map::MapObject,
        pipeline::{self, PipelineSteps, PreparedTile},
    },
    parameters::MapParameters,
};

pub fn compute_tile_map_objects(
    args: &MapParameters,
    tile: &PreparedTile,
) -> crate::Result<Vec<MapObject>> {
    Ok(pipeline::compute_tile(
        tile,
        args,
        PipelineSteps {
            basemap: true,
            contours: true,
            openness: true,
            vegetation: true,
            cliffs: true,
            intensity: true,
            water: true,
        },
        false,
    )?
    .objects)
}
