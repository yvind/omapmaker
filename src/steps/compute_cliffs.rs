use crate::{
    geometry::{LineString, MapMultiPolygon, MultiPolygon, Polygon},
    map::{AreaObject, MapObject, Omap, Symbol},
    raster::Dfm,
};

use geo::{BooleanOps, Simplify};

use std::sync::{Arc, Mutex};

pub fn compute_cliffs(
    slope: &Dfm,
    cliff_threshold: f64,
    convex_hull: &LineString,
    cut_overlay: &Polygon,
    simplify_epsilon: f64,
    map: &Arc<Mutex<Omap>>,
) {
    let symbol = Symbol::GiganticBoulder;
    let cliff_contours = slope.marching_squares(cliff_threshold);

    let cliff_hint = match slope.hint_value() {
        Some(v) => *v,
        None => return,
    } > cliff_threshold;

    let mut cliff_polygons =
        MultiPolygon::from_contours(cliff_contours, convex_hull, symbol.min_size(), cliff_hint);

    cliff_polygons = cut_overlay.intersection(&cliff_polygons);

    cliff_polygons = cliff_polygons.simplify(&simplify_epsilon);

    for polygon in cliff_polygons.into_iter() {
        let mut cliff_object = AreaObject::from_polygon(polygon, symbol);
        cliff_object.add_auto_tag();

        map.lock().unwrap().add_object(cliff_object);
    }
}
