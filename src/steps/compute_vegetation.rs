use crate::{
    geometry::{Line, Polygon, PolygonTrigger},
    map::{AreaObject, MapObject, Omap, Symbol},
    raster::Dfm,
};

pub fn compute_vegetation(
    dsm: &Dfm,
    opt_lower_threshold: Option<f64>,
    opt_upper_threshold: Option<f64>,
    convex_hull: &Line,
    dist_to_hull_epsilon: f64,
    simplify_epsilon: f64,
    symbol: Symbol,
    min_size: f64,
    map: &mut Omap,
) {
    let mut contours;
    let veg_hint;
    let polygon_trigger;
    if let (Some(lower_threshold), Some(upper_threshold)) =
        (opt_lower_threshold, opt_upper_threshold)
    {
        // Interested in a band of values
        contours = dsm.marching_squares(lower_threshold).unwrap();
        let mut upper_contours = dsm.marching_squares(upper_threshold).unwrap();

        veg_hint = dsm.field[dsm.height / 2][dsm.width / 2] < upper_threshold
            && dsm.field[dsm.height / 2][dsm.width / 2] > lower_threshold;

        for c in upper_contours.iter_mut() {
            c.vertices.reverse();
        }
        polygon_trigger = PolygonTrigger::Above;

        contours.extend(upper_contours);
    } else if let Some(lower_threshold) = opt_lower_threshold {
        // Only interested in area above lower threshold
        contours = dsm.marching_squares(lower_threshold).unwrap();
        veg_hint = dsm.field[dsm.height / 2][dsm.width / 2] > lower_threshold;
        polygon_trigger = PolygonTrigger::Above;
    } else if let Some(upper_threshold) = opt_upper_threshold {
        // Only interested in area below upper threshold
        contours = dsm.marching_squares(upper_threshold).unwrap();
        veg_hint = dsm.field[dsm.height / 2][dsm.width / 2] < upper_threshold;
        polygon_trigger = PolygonTrigger::Below;
    } else {
        // Both thresholds are None so we want nothing and just returns
        return;
    }

    for vc in contours.iter_mut() {
        vc.fix_ends_to_line(convex_hull, dist_to_hull_epsilon);
    }

    let veg_polygons = Polygon::from_contours(
        contours,
        convex_hull,
        polygon_trigger,
        min_size,
        dist_to_hull_epsilon,
        veg_hint,
    );

    for mut polygon in veg_polygons {
        if simplify_epsilon > 0. {
            polygon.simplify(simplify_epsilon);
        }
        let mut veg_object = AreaObject::from_polygon(polygon, symbol);
        veg_object.add_auto_tag();
        map.add_object(veg_object);
    }
}
