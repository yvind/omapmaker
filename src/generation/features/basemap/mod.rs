use std::collections::HashMap;

use crate::{
    map::{LineSymbol, MapObject},
    raster::{ContourTerrain, Dfm},
};

use geo::{BooleanOps, Simplify};

pub fn compute_basemap(
    contour_dem: &Dfm<ContourTerrain>,
    z_range: (f32, f32),
    cut_overlay: &geo::Polygon,
    basemap_interval: f32,
) -> Vec<MapObject> {
    let basemap_interval_f64 = f64::from(basemap_interval);
    let min_z = f64::from(z_range.0);
    let max_z = f64::from(z_range.1);
    let bm_levels = ((max_z - min_z) / basemap_interval_f64).ceil() as usize + 1;
    let start_level = (min_z / basemap_interval_f64).floor() * basemap_interval_f64;

    let mut objects = Vec::new();
    for c_index in 0..bm_levels {
        let bm_level = (c_index as f64 * basemap_interval_f64 + start_level) as f32;

        let mut bm_contours = contour_dem.marching_squares(bm_level);

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
