use crate::{
    geometry::{
        LineString, MapMultiLineString, MapMultiPolygon, MapRectangle, MultiPolygon, Polygon,
        PolygonTrigger, Rectangle,
    },
    map::{AreaObject, MapObject, Omap, Symbol},
    raster::Dfm,
};

use geo::Simplify;

use crate::{INV_CELL_SIZE_USIZE, TILE_SIZE_USIZE};
const SIDE_LENGTH: usize = INV_CELL_SIZE_USIZE * TILE_SIZE_USIZE;

use std::sync::{Arc, Mutex};

pub fn compute_cliffs(
    slope: &Dfm,
    cliff_threshold: f64,
    dist_to_hull_epsilon: f64,
    convex_hull: &LineString,
    temp_cut: &Rectangle,
    cut_overlay: &Polygon,
    simplify_epsilon: f64,
    map: &Arc<Mutex<Omap>>,
) {
    let mut cliff_contours = slope.marching_squares(cliff_threshold).unwrap();

    cliff_contours.fix_ends_to_line(convex_hull, dist_to_hull_epsilon);
    cliff_contours = temp_cut.clip_lines(cliff_contours); // clip in geo is not trust-worthy, randomly splits and reverses LineStrings

    let cliff_hint = slope[(SIDE_LENGTH / 2, SIDE_LENGTH / 2)] > cliff_threshold;
    let mut cliff_polygons = MultiPolygon::from_contours(
        cliff_contours,
        cut_overlay.exterior(),
        PolygonTrigger::Above,
        10.,
        dist_to_hull_epsilon,
        cliff_hint,
    );

    if simplify_epsilon > 0. {
        cliff_polygons = cliff_polygons.simplify(&simplify_epsilon);
    }

    for polygon in cliff_polygons.into_iter() {
        let mut cliff_object = AreaObject::from_polygon(polygon, Symbol::GiganticBoulder);
        cliff_object.add_auto_tag();

        map.lock().unwrap().add_object(cliff_object);
    }
}
