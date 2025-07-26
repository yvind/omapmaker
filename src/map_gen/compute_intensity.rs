use std::sync::{Arc, Mutex};

use geo::{BooleanOps, LineString, MultiPolygon, Polygon, Simplify};
use omap::{objects::AreaObject, symbols::SymbolTrait, Omap};

use crate::{geometry::MapMultiPolygon, parameters::MapParameters, raster::Dfm};

pub fn compute_intensity(
    dim: &Dfm,
    convex_hull: &LineString,
    cut_overlay: &Polygon,
    params: &MapParameters,
    map: &Arc<Mutex<Omap>>,
) {
    for filter in params.intensity_filters.iter() {
        let lower_contours = dim.marching_squares(filter.low);
        let upper_contours = dim.marching_squares(filter.high);

        let lower_polygons = MultiPolygon::from_contours(
            lower_contours,
            convex_hull,
            filter.symbol.min_size(params.scale),
            false,
        );
        let upper_polygons = MultiPolygon::from_contours(
            upper_contours,
            convex_hull,
            filter.symbol.min_size(params.scale),
            true,
        );

        let mut polygons = lower_polygons.intersection(&upper_polygons);

        polygons = cut_overlay.intersection(&polygons);
        polygons = polygons.simplify(&crate::SIMPLIFICATION_DIST);

        for polygon in polygons.into_iter() {
            let intensity_object = AreaObject::from_polygon(polygon, filter.symbol, 0.);

            map.lock().unwrap().add_object(intensity_object);
        }
    }
}
