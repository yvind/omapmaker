use crate::{geometry::MapMultiPolygon, parameters::MapParameters, raster::Dfm};

use geo::{BooleanOps, MultiPolygon, Polygon, Simplify};
use omap::{
    objects::AreaObject,
    symbols::{AreaSymbol, SymbolTrait},
    Omap,
};

use std::sync::{Arc, Mutex};

pub fn compute_cliffs(
    slope: &Dfm,
    convex_hull: &Polygon,
    cut_overlay: &Polygon,
    params: &MapParameters,
    map: &Arc<Mutex<Omap>>,
) {
    let symbol = AreaSymbol::GiganticBoulder;
    let cliff_contours = slope.marching_squares(params.cliff);

    let mut cliff_polygons = MultiPolygon::from_contours(cliff_contours, convex_hull, false);

    cliff_polygons = cliff_polygons.simplify(crate::SIMPLIFICATION_DIST);

    for buffer in params.buffer_rules.iter() {
        cliff_polygons = cliff_polygons.apply_buffer_rule(buffer);
    }

    cliff_polygons = cliff_polygons.remove_small_polygons(symbol.min_size(params.scale));
    cliff_polygons = cut_overlay.intersection(&cliff_polygons);

    let num_polys = cliff_polygons.0.len();
    {
        map.lock().unwrap().reserve_capacity(symbol, num_polys);
    }

    for polygon in cliff_polygons.into_iter() {
        let cliff_object = AreaObject::from_polygon(polygon, symbol, 0.);

        map.lock().unwrap().add_object(cliff_object);
    }
}
