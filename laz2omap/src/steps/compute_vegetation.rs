#![allow(clippy::too_many_arguments)]

use crate::geometry::MapMultiPolygon;
use crate::parameters::MapParameters;
use crate::raster::{Dfm, Threshold};

use geo::{BooleanOps, LineString, MultiPolygon, Polygon, Simplify};
use omap::{AreaObject, AreaSymbol, MapObject, Omap, Symbol, TagTrait};

use std::sync::{Arc, Mutex};

pub fn compute_vegetation(
    dfm: &Dfm,
    threshold: Threshold,
    convex_hull: &LineString,
    cut_overlay: &Polygon,
    symbol: AreaSymbol,
    params: &MapParameters,
    map: &Arc<Mutex<Omap>>,
) {
    let contours = dfm.marching_squares(threshold.inner());

    let mut veg_polygons = MultiPolygon::from_contours(
        contours,
        convex_hull,
        symbol.min_size(params.scale),
        threshold.is_upper(),
    );

    veg_polygons = cut_overlay.intersection(&veg_polygons);

    veg_polygons = veg_polygons.simplify(&crate::SIMPLIFICATION_DIST);

    let num_polys = veg_polygons.0.len();
    {
        map.lock()
            .unwrap()
            .reserve_capacity(Symbol::from(symbol), num_polys);
    }

    for polygon in veg_polygons {
        let mut veg_object = AreaObject::from_polygon(polygon, symbol);
        veg_object.add_auto_tag();

        map.lock()
            .unwrap()
            .add_object(MapObject::AreaObject(veg_object));
    }
}
