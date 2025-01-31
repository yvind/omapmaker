use crate::{
    geometry::MapMultiPolygon,
    parser::Args,
    raster::{Dfm, Threshold},
};

use geo::{BooleanOps, LineString, MultiPolygon, Polygon, Simplify};
use omap::{AreaObject, MapObject, Omap, Symbol, TagTrait};

use std::sync::{Arc, Mutex};

pub fn compute_cliffs(
    slope: &Dfm,
    cliff_threshold: Threshold,
    convex_hull: &LineString,
    cut_overlay: &Polygon,
    args: &Args,
    map: &Arc<Mutex<Omap>>,
) {
    let symbol = Symbol::GiganticBoulder(geo::Polygon::new(geo::line_string![], vec![]));
    let cliff_contours = slope.marching_squares(cliff_threshold.inner());

    let mut cliff_polygons = MultiPolygon::from_contours(
        cliff_contours,
        convex_hull,
        symbol.min_size(omap::Scale::S15_000),
        cliff_threshold.is_upper(),
    );

    cliff_polygons = cut_overlay.intersection(&cliff_polygons);

    cliff_polygons = cliff_polygons.simplify(&args.simplification_distance);

    for polygon in cliff_polygons.into_iter() {
        let symbol = Symbol::GiganticBoulder(polygon);

        let mut cliff_object = AreaObject::from_symbol(symbol);
        cliff_object.add_auto_tag();

        map.lock()
            .unwrap()
            .add_object(MapObject::AreaObject(cliff_object));
    }
}
