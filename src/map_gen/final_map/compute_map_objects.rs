use crate::parameters::MapParameters;
use crate::raster::Threshold;
use crate::statistics::LidarStats;
use crate::{geometry::PointCloud, map_gen};

use geo::{Area, BooleanOps, Polygon, Rect};
use map_gen::egui_map::{AreaSymbol, MapObject};
use std::cmp::Ordering;

pub fn compute_map_objects(
    args: &MapParameters,
    ground_cloud: PointCloud,
    stats: &LidarStats,
    convex_hull: Polygon,
    cut_bounds: Rect,
) -> crate::Result<Vec<MapObject>> {
    let (dem, drm, dim, z_range) = map_gen::common::compute_dfms(ground_cloud, stats)?;
    let grad_dem = dem.slope(3);

    let mut mp = cut_bounds.to_polygon().intersection(&convex_hull);
    if mp.0.is_empty() {
        return Ok(Vec::new());
    }

    mp.0.sort_by(|a, b| {
        a.signed_area()
            .partial_cmp(&b.signed_area())
            .unwrap_or(Ordering::Equal)
    });
    let cut_overlay = mp.0.swap_remove(0);

    let mut objects = Vec::new();

    if args.contour.basemap_interval >= 0.1 {
        objects.extend(map_gen::common::compute_basemap(
            &dem,
            z_range,
            &cut_overlay,
            args.contour.basemap_interval,
        ));
    }

    match args.contour.algorithm {
        crate::parameters::ContourAlgo::NaiveIterations => {
            let (contours, _, _) = map_gen::common::compute_naive_contours(
                &dem,
                z_range,
                &cut_overlay,
                (0.9, 1.1),
                args,
            )?;
            objects.extend(contours);
        }
        crate::parameters::ContourAlgo::NormalFieldSmoothing => {
            let smooth_dem = dem.smoothen(15., 15, args.contour.algo_steps as usize);
            let (contours, _, _) =
                map_gen::common::extract_contours(&smooth_dem, z_range, &cut_overlay, args, false)?;
            objects.extend(contours);
        }
        crate::parameters::ContourAlgo::Raw => {
            let (contours, _, _) =
                map_gen::common::extract_contours(&dem, z_range, &cut_overlay, args, false)?;
            objects.extend(contours);
        }
    }

    objects.extend(map_gen::common::compute_vegetation(
        &drm,
        Threshold::Upper(args.vegetation.yellow),
        &convex_hull,
        &cut_overlay,
        AreaSymbol::RoughOpenLand,
        args,
        &args.geometry.openness.buffer_rules,
    ));

    objects.extend(map_gen::common::compute_vegetation(
        &drm,
        Threshold::Lower(args.vegetation.green.0),
        &convex_hull,
        &cut_overlay,
        AreaSymbol::LightGreen,
        args,
        &args.geometry.vegetation.buffer_rules,
    ));

    objects.extend(map_gen::common::compute_vegetation(
        &drm,
        Threshold::Lower(args.vegetation.green.1),
        &convex_hull,
        &cut_overlay,
        AreaSymbol::MediumGreen,
        args,
        &args.geometry.vegetation.buffer_rules,
    ));

    objects.extend(map_gen::common::compute_vegetation(
        &drm,
        Threshold::Lower(args.vegetation.green.2),
        &convex_hull,
        &cut_overlay,
        AreaSymbol::DarkGreen,
        args,
        &args.geometry.vegetation.buffer_rules,
    ));

    objects.extend(map_gen::common::compute_cliffs(
        &grad_dem,
        &convex_hull,
        &cut_overlay,
        args,
    ));

    objects.extend(map_gen::common::compute_intensity(
        &dim,
        &convex_hull,
        &cut_overlay,
        args,
        &args.geometry.intensity.buffer_rules,
    ));

    Ok(objects)
}
