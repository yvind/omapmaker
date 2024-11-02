use crate::geometry::{
    LineString, MapMultiLineString, MapMultiPolygon, MapRectangle, MultiLineString, MultiPolygon,
    Polygon, PolygonTrigger, Rectangle,
};
use crate::map::{AreaObject, MapObject, Omap, Symbol};
use crate::raster::Dfm;

use crate::{INV_CELL_SIZE_USIZE, TILE_SIZE_USIZE};
const SIDE_LENGTH: usize = INV_CELL_SIZE_USIZE * TILE_SIZE_USIZE;

use geo::Simplify;

use std::sync::{Arc, Mutex};

pub fn compute_vegetation(
    dfm: &Dfm,
    opt_thresholds: (Option<f64>, Option<f64>),
    convex_hull: &LineString,
    temp_cut: &Rectangle,
    cut_overlay: &Polygon,
    dist_to_hull_epsilon: f64,
    simplify_epsilon: f64,
    symbol: Symbol,
    min_size: f64,
    map: &Arc<Mutex<Omap>>,
) {
    let mut contours;
    let hint_val = dfm[(SIDE_LENGTH / 2, SIDE_LENGTH / 2)];
    let veg_hint;
    let polygon_trigger;

    match opt_thresholds {
        (Some(lower_threshold), Some(upper_threshold)) => {
            // Interested in a band of values
            contours = dfm.marching_squares(lower_threshold).unwrap();
            let mut upper_contours = dfm.marching_squares(upper_threshold).unwrap();

            veg_hint = hint_val < upper_threshold && hint_val > lower_threshold;

            for c in upper_contours.iter_mut() {
                c.0.reverse();
            }
            polygon_trigger = PolygonTrigger::Above;

            contours.0.extend(upper_contours);
        }
        (Some(lower_threshold), None) => {
            // Only interested in area above lower threshold
            contours = dfm.marching_squares(lower_threshold).unwrap();
            veg_hint = hint_val > lower_threshold;
            polygon_trigger = PolygonTrigger::Above;
        }
        (None, Some(upper_threshold)) => {
            // Only interested in area below upper threshold
            contours = dfm.marching_squares(upper_threshold).unwrap();
            veg_hint = hint_val < upper_threshold;
            polygon_trigger = PolygonTrigger::Below;
        }
        (None, None) => return,
    }

    contours.fix_ends_to_line(convex_hull, dist_to_hull_epsilon);

    let veg_polygons = MultiPolygon::from_contours(
        contours,
        convex_hull,
        polygon_trigger,
        min_size,
        dist_to_hull_epsilon,
        veg_hint,
    );

    let mut veg_contours = MultiLineString::from_polygons(veg_polygons);

    veg_contours = temp_cut.clip_lines(veg_contours); // clip in geo is not trust-worthy, randomly splits and reverses LineStrings

    //veg_contours.fix_ends_to_line(cut_overlay.exterior(), dist_to_hull_epsilon);

    let mut veg_polygons = MultiPolygon::from_contours(
        veg_contours,
        cut_overlay.exterior(),
        polygon_trigger,
        0.,
        dist_to_hull_epsilon,
        veg_hint,
    );

    if simplify_epsilon > 0. {
        veg_polygons = veg_polygons.simplify(&simplify_epsilon);
    }

    for polygon in veg_polygons {
        let mut veg_object = AreaObject::from_polygon(polygon, symbol);
        veg_object.add_auto_tag();

        map.lock().unwrap().add_object(veg_object);
    }
}
