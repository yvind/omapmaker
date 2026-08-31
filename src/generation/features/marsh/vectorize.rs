use geo::BooleanOps;

use crate::{
    map::{AreaSymbol, MapObject},
    parameters::MapParameters,
    raster::Threshold,
};

use super::MarshDetection;

pub fn marsh_objects(
    detection: &MarshDetection,
    hull: &geo::Polygon,
    cut_overlay: &geo::Polygon,
    params: &MapParameters,
    exclusions: &geo::MultiPolygon,
) -> Vec<MapObject> {
    let mut objects = super::super::compute_vegetation(
        &detection.mask,
        Threshold::Lower(0.5),
        hull,
        cut_overlay,
        AreaSymbol::Marsh,
        params,
        &params.geometry.marsh.buffer_rules,
    );

    if !exclusions.0.is_empty() {
        let mut clipped = Vec::new();
        for object in objects {
            let MapObject::Area {
                object,
                symbol,
                tags,
            } = object
            else {
                unreachable!("marsh vectorization emits only areas");
            };
            clipped.extend(object.difference(exclusions).into_iter().map(|object| {
                MapObject::Area {
                    object,
                    symbol,
                    tags: tags.clone(),
                }
            }));
        }
        objects = clipped;
    }

    for object in &mut objects {
        if let MapObject::Area { tags, .. } = object {
            tags.insert("Detector".into(), "flow-marsh-v2".into());
        }
    }
    objects
}
