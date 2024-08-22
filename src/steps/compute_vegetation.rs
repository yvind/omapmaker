fn compute_open_land(
    drm: &Dfm,
    yellow_level: f64,
    dist_to_hull_epsilon: f64,
    convex_hull: &Line,
    simplify_epsilon: f64,
    map: &mut Omap,
) {
    let mut yellow_contours = drm.marching_squares(yellow_level).unwrap();

    for yc in yellow_contours.iter_mut() {
        yc.fix_ends_to_line(&convex_hull, dist_to_hull_epsilon);
    }

    let yellow_hint = drm.field[drm.height / 2][drm.width / 2] > yellow_level;
    let yellow_polygons = Polygon::from_contours(
        yellow_contours,
        &convex_hull,
        PolygonTrigger::Below,
        10.,
        dist_to_hull_epsilon,
        yellow_hint,
    );

    for mut polygon in yellow_polygons {
        if simplify_epsilon > 0. {
            polygon.simplify(simplify_epsilon);
        }
        let mut yellow_object = AreaObject::from_polygon(polygon, Symbol::RoughOpenLand);
        yellow_object.add_auto_tag();
        map.add_object(yellow_object);
    }
}
