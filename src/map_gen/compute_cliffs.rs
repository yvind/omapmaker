use crate::{geometry::MapMultiPolygon, parameters::MapParameters, raster::Dfm};

use geo::{BooleanOps, LineString, MultiPolygon, Polygon, Simplify};
use omap::{
    objects::AreaObject,
    symbols::{AreaSymbol, SymbolTrait},
    Omap,
};

use std::sync::{Arc, Mutex};

pub fn compute_cliffs(
    slope: &Dfm,
    convex_hull: &LineString,
    cut_overlay: &Polygon,
    params: &MapParameters,
    map: &Arc<Mutex<Omap>>,
) {
    let symbol = AreaSymbol::GiganticBoulder;
    let cliff_contours = slope.marching_squares(params.cliff);

    let mut cliff_polygons = MultiPolygon::from_contours(
        cliff_contours,
        convex_hull,
        symbol.min_size(params.scale),
        false,
    );

    cliff_polygons = cut_overlay.intersection(&cliff_polygons);

    cliff_polygons = cliff_polygons.simplify(&crate::SIMPLIFICATION_DIST);

    for polygon in cliff_polygons.into_iter() {
        let cliff_object = AreaObject::from_polygon(polygon, symbol, 0.);

        map.lock().unwrap().add_object(cliff_object);
    }
}
