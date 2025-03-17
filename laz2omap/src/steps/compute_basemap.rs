use crate::raster::Dfm;
use omap::{LineObject, LineSymbol, MapObject, Omap, Symbol, TagTrait};

use geo::{BooleanOps, Polygon, Simplify};
use std::sync::{Arc, Mutex};

pub fn compute_basemap(
    dem: &Dfm,
    z_range: (f64, f64),
    cut_overlay: &Polygon,
    basemap_interval: f64,
    map: &Arc<Mutex<Omap>>,
) {
    let bm_levels = ((z_range.1 - z_range.0) / basemap_interval).ceil() as usize + 1;
    let start_level = (z_range.0 / basemap_interval).floor() * basemap_interval;

    for c_index in 0..bm_levels {
        let bm_level = c_index as f64 * basemap_interval + start_level;

        let mut bm_contours = dem.marching_squares(bm_level);

        bm_contours = bm_contours.simplify(&crate::SIMPLIFICATION_DIST);

        bm_contours = cut_overlay.clip(&bm_contours, false);

        let num_lines = bm_contours.0.len();
        {
            let mut aq_map = map.lock().unwrap();
            aq_map.reserve_capacity(Symbol::BasemapContour, num_lines);
            aq_map.reserve_capacity(Symbol::NegBasemapContour, num_lines);
        }

        for c in bm_contours {
            let mut c_object = LineObject::from_line_string(c, LineSymbol::BasemapContour);
            c_object.add_elevation_tag(bm_level);

            map.lock()
                .unwrap()
                .add_object(MapObject::LineObject(c_object));
        }
    }
}
