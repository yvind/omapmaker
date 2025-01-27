#![allow(clippy::too_many_arguments)]

use crate::geometry::MapMultiPolygon;
use crate::parser::Args;
use crate::raster::{Dfm, Threshold};

use geo::{BooleanOps, LineString, MultiPolygon, Polygon, Simplify};
use omap::{AreaObject, MapObject, Omap, Symbol};

use std::sync::{Arc, Mutex};

pub fn compute_vegetation(
    dfm: &Dfm,
    threshold: Threshold,
    convex_hull: &LineString,
    cut_overlay: &Polygon,
    args: &Args,
    symbol: Symbol,
    map: &Arc<Mutex<Omap>>,
) {
    let contours = dfm.marching_squares(threshold.inner());

    let mut veg_polygons = MultiPolygon::from_contours(
        contours,
        convex_hull,
        symbol.min_size(),
        threshold.is_upper(),
    );

    veg_polygons = cut_overlay.intersection(&veg_polygons);

    veg_polygons = veg_polygons.simplify(&args.simplification_distance);

    for polygon in veg_polygons {
        let mut veg_object = AreaObject::from_polygon(polygon, symbol);
        veg_object.add_auto_tag();

        map.lock().unwrap().add_object(veg_object);
    }
}
