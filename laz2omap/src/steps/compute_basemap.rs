use crate::parser::Args;
use crate::raster::Dfm;
use omap::{LineObject, MapObject, Omap, Symbol, TagTrait};

use geo::{BooleanOps, Polygon, Simplify};
use std::sync::{Arc, Mutex};

pub fn compute_basemap(
    dem: &Dfm,
    z_range: (f64, f64),
    cut_overlay: &Polygon,
    args: &Arc<Args>,
    map: &Arc<Mutex<Omap>>,
) {
    let basemap_interval = args.basemap_contours;

    let bm_levels = ((z_range.1 - z_range.0) / basemap_interval).ceil() as usize + 1;
    let start_level = (z_range.0 / basemap_interval).floor() * basemap_interval;

    for c_index in 0..bm_levels {
        let bm_level = c_index as f64 * basemap_interval + start_level;

        let mut bm_contours = dem.marching_squares(bm_level);

        bm_contours = bm_contours.simplify(&args.simplification_distance);

        bm_contours = cut_overlay.clip(&bm_contours, false);

        for c in bm_contours {
            let symbol = Symbol::BasemapContour(c);

            let mut c_object = LineObject::from_symbol(symbol);
            c_object.add_auto_tag();
            c_object.add_tag("Elevation", format!("{:.2}", bm_level).as_str());

            map.lock()
                .unwrap()
                .add_object(MapObject::LineObject(c_object));
        }
    }
}
