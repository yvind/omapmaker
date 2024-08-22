use crate::{
    geometry::{Line, Polygon, PolygonTrigger},
    map::{AreaObject, MapObject, Omap, Symbol},
    raster::Dfm,
};

pub fn compute_cliffs(
    slope: &Dfm,
    cliff_threshold: f64,
    dist_to_hull_epsilon: f64,
    convex_hull: &Line,
    simplify_epsilon: f64,
    map: &mut Omap,
) {
    let mut cliff_contours = slope.marching_squares(cliff_threshold).unwrap();

    for yc in cliff_contours.iter_mut() {
        yc.fix_ends_to_line(convex_hull, dist_to_hull_epsilon);
    }

    let cliff_hint = slope.field[slope.height / 2][slope.width / 2] > cliff_threshold;
    let cliff_polygons = Polygon::from_contours(
        cliff_contours,
        convex_hull,
        PolygonTrigger::Above,
        0.,
        dist_to_hull_epsilon,
        cliff_hint,
    );

    for mut polygon in cliff_polygons {
        if simplify_epsilon > 0. {
            polygon.simplify(simplify_epsilon);
        }
        let mut cliff_object = AreaObject::from_polygon(polygon, Symbol::GiganticBoulder);
        cliff_object.add_auto_tag();
        map.add_object(cliff_object);
    }
}
