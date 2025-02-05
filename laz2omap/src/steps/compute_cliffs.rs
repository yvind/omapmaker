use crate::{
    geometry::MapMultiPolygon,
    params::MapParams,
    raster::{Dfm, Threshold},
};

use geo::{BooleanOps, LineString, MultiPolygon, Polygon, Simplify};
use omap::{AreaObject, AreaSymbol, MapObject, Omap, Symbol, TagTrait};

use std::sync::{Arc, Mutex};

pub fn compute_cliffs(
    slope: &Dfm,
    convex_hull: &LineString,
    cut_overlay: Option<&Polygon>,
    params: &MapParams,
    map: &Arc<Mutex<Omap>>,
) {
    let symbol = AreaSymbol::GiganticBoulder;
    let cliff_contours = slope.marching_squares(params.cliff);

    let mut cliff_polygons = MultiPolygon::from_contours(
        cliff_contours,
        convex_hull,
        symbol.min_size(omap::Scale::S15_000),
        false,
    );

    if let Some(overlay) = cut_overlay {
        cliff_polygons = overlay.intersection(&cliff_polygons);
    }

    cliff_polygons = cliff_polygons.simplify(&params.simplification_distance);

    for polygon in cliff_polygons.into_iter() {
        let mut cliff_object = AreaObject::from_polygon(polygon, symbol);
        cliff_object.add_auto_tag();

        map.lock()
            .unwrap()
            .add_object(MapObject::AreaObject(cliff_object));
    }
}
