use crate::comms::messages::FrontendTask;
use crate::geometry::{MapLineString, MapRect};
use crate::parameters::MapParameters;
use crate::raster::Threshold;
use crate::steps;

use geo::{Coord, Rect};
use omap::{AreaSymbol, LineObject, LineSymbol, MapObject, Omap};

use std::fs::File;
use std::io::BufReader;
use std::sync::mpsc::Sender;
use std::sync::{Arc, Mutex};

pub fn compute_map_objects(
    sender: Sender<FrontendTask>,
    map: Arc<Mutex<Omap>>,
    args: &MapParameters,
    points: copc_rs::PointIter<BufReader<File>>,
    ref_point: Coord,
    mut cut_bounds: Rect,
    threads: usize,
) {
    // step 2: read each laz file and its neighbours and build point-cloud(s)
    let (ground_cloud, convex_hull, tl) = steps::read_laz(tile_path, ref_point);

    let z_range = (ground_cloud.bounds.min.z, ground_cloud.bounds.max.z);

    // step 3: compute the DFMs
    let (dem, drm) = steps::compute_dfms(ground_cloud, tl);
    let grad_dem = dem.slope(3);

    // figure out the cut-overlay (intersect of cut-bounds and convex hull)
    cut_bounds.set_min(cut_bounds.min() - ref_point);
    cut_bounds.set_max(cut_bounds.max() - ref_point);

    let cut_overlay = match convex_hull.inner_line(&cut_bounds.into_line_string()) {
        Some(l) => l,
        None => {
            // send inc message
            return;
        }
    };

    map.lock()
        .unwrap()
        .add_object(MapObject::LineObject(LineObject::from_line_string(
            cut_overlay.exterior().clone(),
            LineSymbol::Formline,
        )));

    // step 4: contour generation
    if args.basemap_interval >= 0.1 {
        steps::compute_basemap(&dem, z_range, &cut_overlay, args.basemap_interval, &map);
    }

    match args.contour_algorithm {
        crate::parameters::ContourAlgo::AI => {
            unimplemented!("No AI contours yet...");
        }
        crate::parameters::ContourAlgo::NaiveIterations => {
            steps::compute_contours::compute_naive_contours(
                &dem,
                z_range,
                &cut_overlay,
                (0.9, 1.1),
                &args,
                &map,
            );
        }
        crate::parameters::ContourAlgo::NormalFieldSmoothing => {
            let smooth_dem = dem.smoothen(15., 15, args.contour_algo_steps as usize);
            steps::compute_contours::extract_contours(
                &smooth_dem,
                z_range,
                &cut_overlay,
                &args,
                &map,
            );
        }
        crate::parameters::ContourAlgo::Raw => {
            steps::compute_contours::extract_contours(&dem, z_range, &cut_overlay, &args, &map);
        }
    }

    // TODO: make thresholds adaptive to local terrain (create a smoothed version of the dfm and use that value to adapt threshold)
    // step 5: compute vegetation
    steps::compute_vegetation(
        &drm,
        Threshold::Upper(1.2),
        &convex_hull,
        &cut_overlay,
        AreaSymbol::RoughOpenLand,
        &args,
        &map,
    );

    steps::compute_vegetation(
        &drm,
        Threshold::Lower(2.1),
        &convex_hull,
        &cut_overlay,
        AreaSymbol::LightGreen,
        &args,
        &map,
    );

    steps::compute_vegetation(
        &drm,
        Threshold::Lower(3.0),
        &convex_hull,
        &cut_overlay,
        AreaSymbol::MediumGreen,
        &args,
        &map,
    );

    steps::compute_vegetation(
        &drm,
        Threshold::Lower(4.0),
        &convex_hull,
        &cut_overlay,
        AreaSymbol::DarkGreen,
        &args,
        &map,
    );

    // step 6: compute cliffs
    steps::compute_cliffs(&grad_dem, &convex_hull, &cut_overlay, &args, &map);

    sender
        .send(FrontendTask::ProgressBar(
            crate::comms::messages::ProgressBar::Inc(1.),
        ))
        .unwrap();
}
