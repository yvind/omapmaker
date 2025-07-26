use crate::{
    comms::messages::*,
    map_gen,
    parameters::{FileParameters, MapParameters},
};
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
    polygon_filter: Option<geo::Polygon>,
) {
    let _ = sender.send(FrontendTask::Log("Starting map generation!".to_string()));

    let num_threads = std::thread::available_parallelism()
        .unwrap_or(NonZero::new(8_usize).unwrap())
        .get();

    let _ = sender.send(FrontendTask::Log(format!(
        "Running on {} threads",
        num_threads
    )));

    // step 0: figure out spatial relationships of the lidar files, assuming they are divided from a big lidar-project by a square-ish grid
    let (laz_neighbour_map, laz_paths, ref_point, masl) =
        map_gen::map_laz(file_params.paths.clone(), &polygon_filter);
    let laz_paths = Arc::new(laz_paths);

    let laz_paths = file_params.paths;

    let map = Arc::new(Mutex::new(
        Omap::new(ref_point, map_params.scale, map_params.output_epsg, masl)
            .expect("Could not create map file"),
    ));

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
        let (tile_paths, tile_cut_bounds) =
            map_gen::retile_bounds(num_threads, &laz_neighbour_map[fi], laz_paths.clone());

        thread::scope(|s| {
            for thread_i in 0..num_threads {
                let map = map.clone();
                let tile_path = tile_paths.clone();
                let cut_bounds = cut_bounds.clone();
                let sender = sender.clone();

                thread::Builder::new()
                    .stack_size(crate::STACK_SIZE * 1024 * 1024)
                    .spawn(move || {
                        map_gen::compute_map_objects(
                            sender.clone(),
                            map,
                            &map_params,
                            tile_paths,
                            ref_point,
                            tile_cut_bounds,
                            num_threads,
                        );
                    });
            }
        });

        let _ = sender.send(FrontendTask::ProgressBar(ProgressBar::Finish));
    }

    let mut omap = Arc::<Mutex<Omap>>::into_inner(map)
        .expect("Could not get inner value of arc, stray reference somewhere")
        .into_inner()
        .expect("Map mutex poisoned, a thread panicked while holding mutex");

    // merge line symbols
    omap.merge_lines(crate::MERGE_DELTA);

    sender
        .send(FrontendTask::Log("Writing Omap file...".to_string()))
        .unwrap();

    let bezier_error = if map_params.bezier_bool {
        Some(map_params.bezier_error)
    } else {
        None
    };

    omap.write_to_file(file_params.save_location.clone(), bezier_error);

    sender.send(FrontendTask::Log("Done!".to_string())).unwrap();
}
