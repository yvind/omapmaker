use std::collections::HashMap;

use crate::{
    map_gen::egui_map::{LineSymbol, MapObject},
    raster::{Dfm, dfm::Elevation},
};

use geo::{BooleanOps, Simplify};

pub fn compute_basemap(
    dem: &Dfm<Elevation>,
    z_range: (f64, f64),
    cut_overlay: &geo::Polygon,
    basemap_interval: f64,
) -> Vec<MapObject> {
    let bm_levels = ((z_range.1 - z_range.0) / basemap_interval).ceil() as usize + 1;
    let start_level = (z_range.0 / basemap_interval).floor() * basemap_interval;

    let mut objects = Vec::new();
    for c_index in 0..bm_levels {
        let bm_level = c_index as f64 * basemap_interval + start_level;

        let mut bm_contours = dem.marching_squares(bm_level);

        bm_contours = bm_contours.simplify(crate::SIMPLIFICATION_DIST);

        bm_contours = cut_overlay.clip(&bm_contours, false);

        let num_lines = bm_contours.0.len();
        objects.reserve(num_lines);

        for c in bm_contours {
            let mut c_object = MapObject::Line {
                object: c,
                tags: HashMap::new(),
                symbol: LineSymbol::BasemapContour,
            };
            c_object.add_elevation_tag(bm_level);
            objects.push(c_object);
        }
    }
    objects
}
