use std::collections::HashMap;

use crate::{
    geometry::MapMultiPolygon,
    map_gen::egui_map::{AreaSymbol, MapObject},
    parameters::{BufferRule, MapParameters},
    raster::Dfm,
};

use geo::{BooleanOps, MultiPolygon, Polygon, Simplify};

pub fn compute_cliffs(
    slope: &Dfm,
    convex_hull: &Polygon,
    cut_overlay: &Polygon,
    params: &MapParameters,
    buffer_rules: &[BufferRule],
) -> Vec<MapObject> {
    let symbol = AreaSymbol::GiganticBoulder;
    let cliff_contours = slope.marching_squares(params.cliff.cliff);

    let mut cliff_polygons = MultiPolygon::from_contours(cliff_contours, convex_hull, false);

    cliff_polygons = cliff_polygons.simplify(crate::SIMPLIFICATION_DIST);

    for buffer in buffer_rules.iter() {
        cliff_polygons = cliff_polygons.apply_buffer_rule(buffer);
    }

    cliff_polygons = cut_overlay.intersection(&cliff_polygons);

    let num_polys = cliff_polygons.0.len();

    let mut objects = Vec::with_capacity(num_polys);

    for polygon in cliff_polygons.into_iter() {
        let cliff_object = MapObject::Area {
            object: polygon,
            symbol,
            tags: HashMap::new(),
        };

        objects.push(cliff_object);
    }
    objects
}
