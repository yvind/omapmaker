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
            buildings: true,
            cliffs: true,
            intensity: true,
            water: true,
            // Hydrological streams are deferred until accumulation has been
            // reconciled across tiles. ONNX streams are tile-local.
            streams: !args.streams.algorithm.uses_deferred_hydrology(),
            // Marsh uses the globally reconciled accumulation field and is
            // therefore deferred by final-map generation.
            marsh: false,
        },
        false,
    )?
    .objects)
}
