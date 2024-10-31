use crate::geometry::{Coord, MapLineString, MapRectangle, Rectangle};
use crate::map::{Omap, Symbol};
use crate::parser::Args;
use crate::steps;

use crate::CELL_SIZE;

use indicatif::ProgressBar;

use std::path::{Path, PathBuf};
use std::sync::{Arc, Mutex};
use std::thread;

pub fn compute_map_objects(
    map: Arc<Mutex<Omap>>,
    args: &Args,
    tile_paths: Vec<PathBuf>,
    ref_point: Coord,
    cut_bounds: Vec<Rectangle>,
    tiff_directory: &Path,
    pb: Arc<Mutex<ProgressBar>>,
) {
    let dist_to_hull_epsilon = 2. * CELL_SIZE;

    let mut thread_handles = vec![];

    let tile_paths = Arc::new(tile_paths);
    let tiff_directory = Arc::new(tiff_directory.to_owned());
    let args = Arc::new(args.clone());
    let cut_bounds = Arc::new(cut_bounds);

    for thread_i in 0..args.threads {
        let map_ref = map.clone();
        let tile_paths_ref = tile_paths.clone();
        let tiff_directory = tiff_directory.clone();
        let pb = pb.clone();
        let args = args.clone();
        let cut_bounds = cut_bounds.clone();

        thread_handles.push(
            thread::Builder::new()
                .stack_size(10 * 1024 * 1024) // needs to increase thread stack size to accomodate marching squares wo hashmaps
                .spawn(move || {
                    let mut current_index = thread_i;

                    while current_index < tile_paths_ref.len() {
                        let tile_path = &tile_paths_ref[current_index];

                        // step 2: read each laz file and its neighbours and build point-cloud
                        let (ground_cloud, ground_tree, convex_hull, tl) =
                            steps::read_laz(tile_path, dist_to_hull_epsilon, ref_point);

                        // step 3: compute the DFMs
                        let (dem, grad_dem, drm, _, dim, _) =
                            steps::compute_dfms(&ground_tree, &ground_cloud, &convex_hull, tl);

                        // figure out the cut-overlay (intersect of cut-bounds and convex hull)
                        let mut current_cut_bounds = cut_bounds[current_index];
                        current_cut_bounds.set_min(current_cut_bounds.min() - ref_point);
                        current_cut_bounds.set_max(current_cut_bounds.max() - ref_point);

                        let cut_overlay =
                            match convex_hull.inner_line(&current_cut_bounds.into_line_string()) {
                                Some(l) => l,
                                None => {
                                    pb.lock().unwrap().inc(1);
                                    current_index += args.threads;
                                    continue;
                                }
                            };

                        // step 4: contour generation
                        if args.basemap_contours >= 0.1 {
                            steps::compute_basemap(
                                &dem,
                                ground_cloud.bounds.min.z,
                                ground_cloud.bounds.max.z,
                                args.basemap_contours,
                                &cut_overlay,
                                args.simplification_distance,
                                &map_ref,
                            );
                        }

                        /*

                        // TODO: make thresholds adaptive to local terrain (create a smoothed version of the dfm and use that value to adapt threshold)
                        // step 5: compute vegetation
                        steps::compute_vegetation(
                            &drm,
                            (None, Some(1.2)),
                            &convex_hull,
                            &cut_overlay,
                            dist_to_hull_epsilon,
                            args.simplification_distance,
                            Symbol::RoughOpenLand,
                            225.,
                            &map_ref,
                        );

                        steps::compute_vegetation(
                            &drm,
                            (Some(2.1), None), //Some(3.0),
                            &convex_hull,
                            &cut_overlay,
                            dist_to_hull_epsilon,
                            args.simplification_distance,
                            Symbol::LightGreen,
                            225.,
                            &map_ref,
                        );

                        steps::compute_vegetation(
                            &drm,
                            (Some(3.0), None), //Some(4.0),
                            &convex_hull,
                            &cut_overlay,
                            dist_to_hull_epsilon,
                            args.simplification_distance,
                            Symbol::MediumGreen,
                            110.,
                            &map_ref,
                        );

                        steps::compute_vegetation(
                            &drm,
                            (Some(4.0), None),
                            &convex_hull,
                            &cut_overlay,
                            dist_to_hull_epsilon,
                            args.simplification_distance,
                            Symbol::DarkGreen,
                            64.,
                            &map_ref,
                        );

                        // step 6: compute cliffs
                        steps::compute_cliffs(
                            &grad_dem,
                            0.7,
                            dist_to_hull_epsilon,
                            &convex_hull,
                            &cut_overlay,
                            args.simplification_distance,
                            &map_ref,
                        );

                        */

                        // step 7: save dfms
                        if args.write_tiff {
                            steps::save_tiffs(
                                dem,
                                grad_dem,
                                dim,
                                drm,
                                &ref_point,
                                tile_path.file_stem().unwrap(),
                                &tiff_directory,
                            );
                        }

                        pb.lock().unwrap().inc(1);

                        current_index += args.threads;
                    }
                })
                .unwrap(),
        );
    }
    for h in thread_handles {
        h.join().unwrap();
    }
}
