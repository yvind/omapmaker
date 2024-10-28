use crate::geometry::LineString;
use crate::map::{LineObject, MapObject, Omap, Symbol};
use crate::raster::Dfm;

use std::sync::{Arc, Mutex};

pub fn compute_basemap(
    min_z: f64,
    max_z: f64,
    basemap_interval: f64,
    dem: &Dfm,
    cut_overlay: &LineString,
    simplify_epsilon: f64,
    map: &Arc<Mutex<Omap>>,
) {
    let bm_levels = ((max_z - min_z) / basemap_interval).ceil() as usize;
    let start_level = (min_z / basemap_interval).floor() * basemap_interval;

    for c_index in 0..bm_levels {
        let bm_level = c_index as f64 * basemap_interval + start_level;

        let mut bm_contours = dem.marching_squares(bm_level).unwrap();

        if simplify_epsilon > 0. {
            for c in bm_contours.iter_mut() {
                c.simplify(simplify_epsilon)
            }
        }

        let mut inside_contours = Vec::with_capacity(bm_contours.len());
        for c in bm_contours {
            let ncs = c.clip(cut_overlay);
            for nc in ncs.into_iter() {
                inside_contours.push(nc);
            }
        }

        for c in inside_contours {
            let mut c_object = LineObject::from_line(c, Symbol::BasemapContour);
            c_object.add_auto_tag();
            c_object.add_tag("Elevation", format!("{:.2}", bm_level).as_str());

            map.lock().unwrap().add_object(c_object);
        }
    }
}
