#![allow(clippy::too_many_arguments)]

use std::collections::HashMap;

use crate::geometry::MapMultiPolygon;
use crate::map_gen::egui_map::{AreaSymbol, MapObject};
use crate::parameters::{BufferRule, MapParameters};
use crate::raster::{Dfm, Threshold};

use geo::{BooleanOps, Simplify};

pub fn compute_vegetation<T: Clone>(
    dfm: &Dfm<T>,
    threshold: Threshold,
    convex_hull: &geo::Polygon,
    cut_overlay: &geo::Polygon,
    symbol: AreaSymbol,
    _params: &MapParameters,
    buffer_rules: &[BufferRule],
) -> Vec<MapObject> {
    let contours = dfm.marching_squares(threshold.inner());

    let mut veg_polygons =
        geo::MultiPolygon::from_contours(contours, convex_hull, threshold.is_upper());

    veg_polygons = veg_polygons.simplify(crate::SIMPLIFICATION_DIST);

    for buffer in buffer_rules.iter() {
        veg_polygons = veg_polygons.apply_buffer_rule(buffer);
    }

    veg_polygons = cut_overlay.intersection(&veg_polygons);

    let num_polys = veg_polygons.0.len();
    let mut objects = Vec::with_capacity(num_polys);

    for polygon in veg_polygons {
        let veg_object = MapObject::Area {
            object: polygon,
            symbol,
            tags: HashMap::new(),
        };

        objects.push(veg_object);
    }
    objects
}
