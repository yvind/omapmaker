use std::collections::HashMap;

use geo::{BooleanOps, Euclidean, Length, Simplify};

use crate::{
    map_gen::egui_map::{LineSymbol, MapObject},
    parameters::MapParameters,
    raster::D8Flow,
};

/// Extract and clip D8 stream reaches for one prepared tile.
pub fn compute_streams(
    flow: &D8Flow,
    cut_overlay: &geo::Polygon,
    params: &MapParameters,
) -> Vec<MapObject> {
    let streams = flow.stream_lines(params.streams.minimum_catchment_area_m2);
    let streams = cut_overlay
        .clip(&streams, false)
        .simplify(crate::CELL_SIZE_METERS * 0.5);
    let minimum_length = LineSymbol::SmallCrossableWatercourse.min_length(params.scale, false)
        * params.scale.denominator()
        / 1_000_000.0;

    streams
        .into_iter()
        .filter(|line| line.0.len() >= 2 && Euclidean.length(line) >= minimum_length)
        .map(|line| MapObject::Line {
            object: line,
            symbol: LineSymbol::SmallCrossableWatercourse,
            tags: HashMap::new(),
        })
        .collect()
}
