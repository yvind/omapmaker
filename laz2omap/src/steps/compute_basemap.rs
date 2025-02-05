use crate::params::MapParams;
use crate::raster::Dfm;
use omap::{LineObject, LineSymbol, MapObject, Omap, TagTrait};

use geo::{BooleanOps, Polygon, Simplify};
use std::sync::{Arc, Mutex};

pub fn compute_basemap(
    dem: &Dfm,
    z_range: (f64, f64),
    cut_overlay: Option<&Polygon>,
    args: &MapParams,
    map: &Arc<Mutex<Omap>>,
) {
    let basemap_interval = args.basemap_interval;

    let bm_levels = ((z_range.1 - z_range.0) / basemap_interval).ceil() as usize + 1;
    let start_level = (z_range.0 / basemap_interval).floor() * basemap_interval;

    for c_index in 0..bm_levels {
        let bm_level = c_index as f64 * basemap_interval + start_level;

        let mut bm_contours = dem.marching_squares(bm_level);

        bm_contours = bm_contours.simplify(&args.simplification_distance);

        if let Some(overlay) = cut_overlay {
            bm_contours = overlay.clip(&bm_contours, false);
        }

        for c in bm_contours {
            let mut c_object = LineObject::from_line_string(c, LineSymbol::BasemapContour);
            c_object.add_auto_tag();
            c_object.add_tag("Elevation", format!("{:.2}", bm_level));

            map.lock()
                .unwrap()
                .add_object(MapObject::LineObject(c_object));
        }
    }
}
