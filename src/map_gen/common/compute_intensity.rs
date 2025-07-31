use std::sync::{Arc, Mutex};

use geo::{BooleanOps, MultiPolygon, Polygon, Simplify};
use omap::{objects::AreaObject, symbols::SymbolTrait, Omap};

use crate::{geometry::MapMultiPolygon, parameters::MapParameters, raster::Dfm};

pub fn compute_intensity(
    dim: &Dfm,
    convex_hull: &Polygon,
    cut_overlay: &Polygon,
    params: &MapParameters,
    map: &Arc<Mutex<Omap>>,
) {
    for filter in params.intensity_filters.iter() {
        let lower_contours = dim.marching_squares(filter.low);
        let upper_contours = dim.marching_squares(filter.high);

        let lower_polygons = MultiPolygon::from_contours(lower_contours, convex_hull, false);
        let upper_polygons = MultiPolygon::from_contours(upper_contours, convex_hull, true);

        let mut polygons = lower_polygons.intersection(&upper_polygons);

        polygons = polygons.simplify(crate::SIMPLIFICATION_DIST);

        for buffer in params.buffer_rules.iter() {
            polygons = polygons.apply_buffer_rule(buffer);
        }

        polygons = polygons.remove_small_polygons(filter.symbol.min_size(params.scale));
        polygons = cut_overlay.intersection(&polygons);

        let num_polys = polygons.0.len();
        {
            map.lock()
                .unwrap()
                .reserve_capacity(filter.symbol, num_polys);
        }

        for polygon in polygons.into_iter() {
            let intensity_object = AreaObject::from_polygon(polygon, filter.symbol, 0.);

            map.lock().unwrap().add_object(intensity_object);
        }
    }
}
