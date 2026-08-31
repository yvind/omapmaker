use std::collections::HashMap;

use geo::{Area, BooleanOps, Simplify};

use crate::{
    geometry::MapMultiPolygon,
    map::{AreaSymbol, MapObject},
    parameters::{BufferRule, BuildingParameters},
};

use super::{BuildingDetection, regularization};

pub fn building_objects(
    detection: &BuildingDetection,
    convex_hull: &geo::Polygon,
    cut_overlay: &geo::Polygon,
    parameters: &BuildingParameters,
    buffer_rules: &[BufferRule],
) -> Vec<MapObject> {
    debug_assert!(
        detection
            .candidate_id
            .grid
            .same_layout(&detection.accepted_mask.grid)
            && detection
                .plane_rejected_mask
                .grid
                .same_layout(&detection.accepted_mask.grid),
        "building diagnostics must use one grid"
    );
    let contours = detection.accepted_mask.marching_squares(0.5);
    let mut traced = geo::MultiPolygon::from_contours(contours, convex_hull, false);
    for rule in buffer_rules {
        traced = traced.apply_buffer_rule(rule);
    }
    let mut polygons = geo::MultiPolygon::new(
        traced
            .into_iter()
            .map(|polygon| regularization::regularize_building_footprint(&polygon, parameters))
            .collect(),
    )
    .simplify(crate::SIMPLIFICATION_DIST);
    polygons = cut_overlay.intersection(&polygons);
    polygons
        .into_iter()
        .filter(|polygon| polygon.unsigned_area() > 0.)
        .map(|polygon| MapObject::Area {
            object: polygon,
            symbol: AreaSymbol::Building,
            tags: HashMap::new(),
        })
        .collect()
}
