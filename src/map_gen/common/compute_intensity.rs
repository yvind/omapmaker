use std::collections::HashMap;

use geo::{BooleanOps, Simplify};

use crate::{
    geometry::MapMultiPolygon,
    map_gen::egui_map::MapObject,
    parameters::{BufferRule, MapParameters},
    raster::{Dfm, dfm::Intensity},
};

pub fn compute_intensity(
    dim: &Dfm<Intensity>,
    convex_hull: &geo::Polygon,
    cut_overlay: &geo::Polygon,
    params: &MapParameters,
    buffer_rules: &[BufferRule],
) -> Vec<MapObject> {
    let mut objects = Vec::new();
    for filter in params.intensity.filters.iter() {
        let lower_contours = dim.marching_squares(filter.low);
        let upper_contours = dim.marching_squares(filter.high);

        let lower_polygons = geo::MultiPolygon::from_contours(lower_contours, convex_hull, false);
        let upper_polygons = geo::MultiPolygon::from_contours(upper_contours, convex_hull, true);

        let mut polygons = lower_polygons.intersection(&upper_polygons);

        polygons = polygons.simplify(crate::SIMPLIFICATION_DIST);

        for buffer in buffer_rules.iter() {
            polygons = polygons.apply_buffer_rule(buffer);
        }

        polygons = cut_overlay.intersection(&polygons);

        let num_polys = polygons.0.len();
        objects.reserve(num_polys);

        for polygon in polygons.into_iter() {
            let intensity_object = MapObject::Area {
                object: polygon,
                symbol: filter.symbol,
                tags: HashMap::new(),
            };

            objects.push(intensity_object)
        }
    }
    objects
}
