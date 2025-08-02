use crate::parameters::MapParameters;
use crate::raster::Threshold;
use crate::statistics::LidarStats;
use crate::{geometry::PointCloud, map_gen};

use geo::{Area, BooleanOps, Polygon, Rect};
use omap::{symbols::AreaSymbol, Omap};

use std::sync::{Arc, Mutex};

pub fn compute_map_objects(
    map: &Arc<Mutex<Omap>>,
    args: &MapParameters,
    ground_cloud: PointCloud,
    stats: &LidarStats,
    convex_hull: Polygon,
    cut_bounds: Rect,
) {
    // Compute the DFMs
    let (dem, drm, dim, z_range) = map_gen::common::compute_dfms(ground_cloud, stats);
    let grad_dem = dem.slope(3);

    // figure out the cut-overlay (intersect of cut-bounds and convex hull)
    let mut mp = cut_bounds.to_polygon().intersection(&convex_hull);
    if mp.0.is_empty() {
        return;
    }

    mp.0.sort_by(|a, b| {
        a.signed_area()
            .partial_cmp(&b.signed_area())
            .expect("Non-normal polygon area!")
    });
    let cut_overlay = mp.0.swap_remove(0);

    // Compute contours
    if args.basemap_interval >= 0.1 {
        map_gen::common::compute_basemap(&dem, z_range, &cut_overlay, args.basemap_interval, map);
    }

    match args.contour_algorithm {
        crate::parameters::ContourAlgo::AI => {
            unimplemented!("No AI contours yet...");
        }
        crate::parameters::ContourAlgo::NaiveIterations => {
            map_gen::common::compute_naive_contours(
                &dem,
                z_range,
                &cut_overlay,
                (0.9, 1.1),
                args,
                map,
            );
        }
        crate::parameters::ContourAlgo::NormalFieldSmoothing => {
            let smooth_dem = dem.smoothen(15., 15, args.contour_algo_steps as usize);
            map_gen::common::extract_contours(&smooth_dem, z_range, &cut_overlay, args, map, false);
        }
        crate::parameters::ContourAlgo::Raw => {
            map_gen::common::extract_contours(&dem, z_range, &cut_overlay, args, map, false);
        }
    }

    // Compute vegetation
    map_gen::common::compute_vegetation(
        &drm,
        Threshold::Upper(args.yellow),
        &convex_hull,
        &cut_overlay,
        AreaSymbol::RoughOpenLand,
        args,
        map,
    );

    map_gen::common::compute_vegetation(
        &drm,
        Threshold::Lower(args.green.0),
        &convex_hull,
        &cut_overlay,
        AreaSymbol::LightGreen,
        args,
        map,
    );

    map_gen::common::compute_vegetation(
        &drm,
        Threshold::Lower(args.green.1),
        &convex_hull,
        &cut_overlay,
        AreaSymbol::MediumGreen,
        args,
        map,
    );

    map_gen::common::compute_vegetation(
        &drm,
        Threshold::Lower(args.green.2),
        &convex_hull,
        &cut_overlay,
        AreaSymbol::DarkGreen,
        args,
        map,
    );

    // Compute cliffs
    map_gen::common::compute_cliffs(&grad_dem, &convex_hull, &cut_overlay, args, map);

    // Compute lidar intensity filters
    map_gen::common::compute_intensity(&dim, &convex_hull, &cut_overlay, args, map);
}
