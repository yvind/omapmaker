use crate::geometry::{LineString, MapMultiPolygon, MultiPolygon, Polygon};
use crate::map::{AreaObject, MapObject, Omap, Symbol};
use crate::raster::Dfm;

use geo::{BooleanOps, Simplify};

use std::sync::{Arc, Mutex};

pub fn compute_vegetation(
    dfm: &Dfm,
    opt_thresholds: (Option<f64>, Option<f64>),
    convex_hull: &LineString,
    cut_overlay: &Polygon,
    dist_to_hull_epsilon: f64,
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

    match opt_thresholds {
        (Some(lower_threshold), Some(upper_threshold)) => {
            // Interested in a band of values
            contours = dfm.marching_squares(lower_threshold).unwrap();
            let mut upper_contours = dfm.marching_squares(upper_threshold).unwrap();

            veg_hint = hint_val < upper_threshold && hint_val > lower_threshold;

            for c in upper_contours.iter_mut() {
                c.0.reverse();
            }

            contours.0.extend(upper_contours);
        }
        (Some(lower_threshold), None) => {
            // Only interested in area above lower threshold
            contours = dfm.marching_squares(lower_threshold).unwrap();
            veg_hint = hint_val > lower_threshold;
        }
        (None, Some(upper_threshold)) => {
            // Only interested in area below upper threshold
            contours = dfm.marching_squares(upper_threshold).unwrap();
            veg_hint = hint_val < upper_threshold;
            for c in contours.iter_mut() {
                c.0.reverse();
            }
        }
        (None, None) => return,
    }

    let veg_polygons = MultiPolygon::from_contours(
        contours,
        convex_hull,
        symbol.min_size(),
        dist_to_hull_epsilon,
        veg_hint,
    );

    let mut veg_polygons = cut_overlay.intersection(&veg_polygons);

    if simplify_epsilon > 0. {
        veg_polygons = veg_polygons.simplify(&simplify_epsilon);
    }

    for polygon in veg_polygons {
        let mut veg_object = AreaObject::from_polygon(polygon, symbol);
        veg_object.add_auto_tag();

        map.lock().unwrap().add_object(veg_object);
    }
}
