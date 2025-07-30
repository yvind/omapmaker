use crate::{
    comms::messages::*,
    map_gen,
    neighbors::NeighborSide,
    parameters::{FileParameters, MapParameters},
    Result,
};
use geo::{Area, BooleanOps, Intersects};
use omap::Omap;

use std::{
    num::NonZero,
    sync::{mpsc::Sender, Arc, Mutex},
    thread,
};

pub fn make_map(
    sender: Sender<FrontendTask>,
    map_params: MapParameters,
    file_params: FileParameters,
    mut polygon_filter: Option<geo::Polygon>,
) -> Result<()> {
    let _ = sender.send(FrontendTask::Log("Starting map generation!".to_string()));

    let num_threads = std::thread::available_parallelism()
        .unwrap_or(NonZero::new(8_usize).unwrap())
        .get();

    let _ = sender.send(FrontendTask::Log(format!(
        "Running on {} threads",
        num_threads
    )));

    // Figure out spatial relationships of the lidar files, assuming they are divided from a big lidar-project by a square-ish grid
    let (laz_paths, laz_neighbor_map, bounds, ref_point, masl) =
        super::map_laz(&file_params.paths, &polygon_filter)?;

    let map = Arc::new(Mutex::new(Omap::new(
        ref_point,
        map_params.scale,
        map_params.output_epsg,
        Some(masl),
    )?));

    if let Some(polygon) = &mut polygon_filter {
        polygon.exterior_mut(|l| {
            for c in l.0.iter_mut() {
                *c = *c - ref_point;
            }
        });
    }

    for fi in 0..laz_paths.len() {
        #[rustfmt::skip]
        let _ = sender.send(FrontendTask::Log("\n***********************************************".to_string()));
        #[rustfmt::skip]
        let _ = sender.send(FrontendTask::Log(format!("\t Processing Lidar-file {} of {}", fi + 1, laz_paths.len())));
        #[rustfmt::skip]
        let _ = sender.send(FrontendTask::Log(format!("\t{:?}", laz_paths[fi].file_name().unwrap())));
        #[rustfmt::skip]
        let _ = sender.send(FrontendTask::Log("-----------------------------------------------".to_string()));
        let _ = sender.send(FrontendTask::ProgressBar(ProgressBar::Start));

        // first get the sub-tile bounds for the current lidar file
        // need tile-neighbor maps, bounds, cut-bounds and touched files (for the edge tiles)
        let (tile_bounds, mut cut_bounds, nx, ny) =
            map_gen::common::retile_bounds(&bounds[fi], &laz_neighbor_map[fi]);

        for cb in cut_bounds.iter_mut() {
            *cb = geo::Rect::new(cb.min() - ref_point, cb.max() - ref_point);
        }

        let num_tiles = nx * ny;
        let inc = 1. / num_tiles as f32;

        thread::scope(|s| {
            for mut thread_i in 0..num_threads {
                let map = map.clone();
                let tile_bounds = tile_bounds.clone();
                let cut_bounds = cut_bounds.clone();
                let sender = sender.clone();
                let laz_neighbor_map = laz_neighbor_map.clone();
                let laz_paths = laz_paths.clone();
                let map_params = map_params.clone();
                let polygon_filter = polygon_filter.clone();

                let _ = thread::Builder::new()
                    .stack_size(crate::STACK_SIZE * 1024 * 1024)
                    .spawn_scoped(s, move || {
                        while thread_i < num_tiles {
                            let edge_tile = NeighborSide::is_edge_tile(thread_i, nx, ny);

                            if let Some(polygon) = &polygon_filter {
                                if !cut_bounds[thread_i].intersects(polygon) {
                                    thread_i += num_threads;
                                    continue;
                                }
                            }

                            let (cloud, mut hull) = match super::read_laz(
                                &laz_paths,
                                &laz_neighbor_map[fi],
                                tile_bounds[thread_i],
                                edge_tile,
                                ref_point,
                            ) {
                                Ok(p) => p,
                                Err(e) => match e {
                                    crate::Error::NoGroundPoints => {
                                        thread_i += num_threads;
                                        continue;
                                    }
                                    e => {
                                        sender
                                            .send(FrontendTask::Error(e.to_string(), true))
                                            .unwrap();
                                        continue;
                                    }
                                },
                            };

                            if let Some(polygon) = &polygon_filter {
                                let mut mp = polygon.intersection(&hull);

                                if mp.0.is_empty() {
                                    thread_i += num_threads;
                                    continue;
                                }

                                mp.0.sort_by(|a, b| {
                                    a.signed_area()
                                        .partial_cmp(&b.signed_area())
                                        .expect("Non-normal polygon area")
                                });
                                hull = mp.0.swap_remove(0);
                            }

                            super::compute_map_objects(
                                &map,
                                &map_params,
                                cloud,
                                hull,
                                cut_bounds[thread_i],
                            );
                            sender
                                .send(FrontendTask::ProgressBar(ProgressBar::Inc(inc)))
                                .unwrap();
                            thread_i += num_threads;
                        }
                    });
            }
        });

        let _ = sender.send(FrontendTask::ProgressBar(ProgressBar::Finish));
    }

    let mut omap = Arc::<Mutex<Omap>>::into_inner(map)
        .expect("Could not get inner value of arc, stray reference somewhere")
        .into_inner()
        .expect("Map mutex poisoned, a thread panicked while holding mutex");

    sender
        .send(FrontendTask::Log("Merging contour lines...".to_string()))
        .unwrap();

    // merge line symbols
    omap.merge_lines(crate::MERGE_DELTA);
    // mark basemap depressions as such
    omap.mark_basemap_depressions();
    // convert the smallest knolls and depressions to point symbols
    omap.make_dotknolls_and_depressions(
        map_params.dot_knoll_area.0,
        map_params.dot_knoll_area.1,
        1.5,
    );

    sender
        .send(FrontendTask::Log("Writing Omap file...".to_string()))
        .unwrap();

    let bezier_error = if map_params.bezier_bool {
        Some(map_params.bezier_error)
    } else {
        None
    };

    let _ = omap.write_to_file(file_params.save_location.clone(), bezier_error);

    sender.send(FrontendTask::Log("Done!".to_string())).unwrap();
    Ok(())
}
