use crate::geometry::{LineString, MapMultiLineString};
use crate::map::{LineObject, MapObject, Omap, Symbol};
use crate::raster::Dfm;

use geo::Simplify;
use std::sync::{Arc, Mutex};

pub fn compute_basemap(
    dem: &Dfm,
    min_z: f64,
    max_z: f64,
    basemap_interval: f64,
    cut_overlay: &LineString,
    simplify_epsilon: f64,
    map: &Arc<Mutex<Omap>>,
) {
    let bm_levels = ((max_z - min_z) / basemap_interval).ceil() as usize;
    let start_level = (min_z / basemap_interval).floor() * basemap_interval;

    for c_index in 0..bm_levels {
        let bm_level = c_index as f64 * basemap_interval + start_level;

        let mut bm_contours = dem.marching_squares(bm_level).unwrap();

        let simplified_contours;
        if simplify_epsilon > 0. {
            simplified_contours = bm_contours
                .into_iter()
                .map(|c| c.simplify(&simplify_epsilon))
                .collect();
        } else {
            simplified_contours = bm_contours;
        }

        let inside_contours = simplified_contours.clip(cut_overlay);

        for c in inside_contours {
            let mut c_object = LineObject::from_line(c, Symbol::BasemapContour);
            c_object.add_auto_tag();
            c_object.add_tag("Elevation", format!("{:.2}", bm_level).as_str());

            map.lock().unwrap().add_object(c_object);
        }
    }
}
