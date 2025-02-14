use crate::raster::Dfm;
use crate::{geometry::MapLineString, parameters::MapParameters};
use omap::{LineObject, LineSymbol, MapObject, Omap, Symbol, TagTrait};

use geo::{BooleanOps, Polygon, Simplify};
use std::sync::{Arc, Mutex};

pub fn compute_basemap(
    dem: &Dfm,
    z_range: (f64, f64),
    cut_overlay: &Polygon,
    args: &MapParameters,
    map: &Arc<Mutex<Omap>>,
) {
    let basemap_interval = args.basemap_interval;

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
            let sym = if let Some(a) = c.line_string_signed_area() {
                if a < 0. {
                    LineSymbol::NegBasemapContour
                } else {
                    LineSymbol::BasemapContour
                }
            } else {
                LineSymbol::BasemapContour
            };

            let mut c_object = LineObject::from_line_string(c, sym);
            c_object.add_auto_tag();
            c_object.add_tag("Elevation", format!("{:.2}", bm_level));

            map.lock()
                .unwrap()
                .add_object(MapObject::LineObject(c_object));
        }
    }
}
