#![allow(clippy::too_many_arguments)]

use crate::geometry::{LineString, MapMultiPolygon, MultiPolygon, Polygon};
use crate::map::{AreaObject, MapObject, Omap, Symbol};
use crate::raster::{Dfm, Threshold};

use geo::{BooleanOps, Simplify};

use std::sync::{Arc, Mutex};

pub fn compute_vegetation(
    dfm: &Dfm,
    threshold: Threshold,
    convex_hull: &LineString,
    cut_overlay: &Polygon,
    simplify_epsilon: f64,
    symbol: Symbol,
    map: &Arc<Mutex<Omap>>,
) {
    let mut contours;
    let hint_val = match dfm.hint_value() {
        Some(f) => *f,
        None => return,
    };

    let veg_hint;

    match threshold {
        Threshold::Lower(threshold) => {
            // Interested in area above lower threshold
            contours = dfm.marching_squares(threshold);
            veg_hint = hint_val > threshold;
        }
        Threshold::Upper(threshold) => {
            // Interested in area below upper threshold
            contours = dfm.marching_squares(threshold);
            veg_hint = hint_val < threshold;
            for c in contours.iter_mut() {
                c.0.reverse();
            }
        }
    }

    let mut veg_polygons =
        MultiPolygon::from_contours(contours, convex_hull, symbol.min_size(), veg_hint);

    veg_polygons = cut_overlay.intersection(&veg_polygons);

    veg_polygons = veg_polygons.simplify(&simplify_epsilon);

    for polygon in veg_polygons {
        let mut veg_object = AreaObject::from_polygon(polygon, symbol);
        veg_object.add_auto_tag();

        map.lock().unwrap().add_object(veg_object);
    }
}
