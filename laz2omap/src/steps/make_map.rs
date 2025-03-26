use crate::{
    comms::messages::*,
    parameters::{FileParameters, MapParameters},
    steps,
};
use omap::Omap;

use std::{
    num::NonZero,
    sync::{mpsc::Sender, Arc, Mutex},
};

pub fn make_map(
    sender: Sender<FrontendTask>,
    map_params: MapParameters,
    file_params: FileParameters,
    polygon_filter: Option<geo::Polygon>,
) {
    sender
        .send(FrontendTask::Log("Map Generation!".to_string()))
        .unwrap();

    let num_threads = std::thread::available_parallelism()
        .unwrap_or(NonZero::new(8_usize).unwrap())
        .get();

    sender
        .send(FrontendTask::Log(format!(
            "Running on {} threads",
            num_threads
        )))
        .unwrap();

    // step 0: figure out spatial relationships of the lidar files, assuming they are divided from a big lidar-project by a square-ish grid
    let (laz_neighbour_map, laz_paths, ref_point) = steps::map_laz(file_params.paths.clone());
    let laz_paths = Arc::new(laz_paths);

    let map = Arc::new(Mutex::new(Omap::new(
        ref_point,
        map_params.output_epsg,
        map_params.scale,
    )));

    for fi in 0..laz_paths.len() {
        #[rustfmt::skip]
        sender.send(FrontendTask::Log("\n***********************************************".to_string())).unwrap();
        #[rustfmt::skip]
        sender.send(FrontendTask::Log(format!("\t Processing Lidar-file {} of {}", fi + 1, laz_paths.len()))).unwrap();
        #[rustfmt::skip]
        sender.send(FrontendTask::Log(format!("\t{:?}", laz_paths[fi].file_name().unwrap()))).unwrap();
        #[rustfmt::skip]
        sender.send(FrontendTask::Log("-----------------------------------------------".to_string())).unwrap();
        sender.send(FrontendTask::ProgressBar(ProgressBar::Start));

        // first get the sub-tile bounds for the current lidar file
        // need tile-neighbour maps, bounds, cut-bounds and touched files (for the edge tiles)
        let (tile_paths, tile_cut_bounds) =
            steps::retile_laz(num_threads, &laz_neighbour_map[fi], laz_paths.clone());

        for thread_i in 0..num_threads {
            let map = map.clone();
            let tile_path = tile_paths.clone();
            let args = args.clone();
            let cut_bounds = cut_bounds.clone();
            let sender = sender.clone();

            thread_handles.push(
                std::thread::Builder::new()
                    .stack_size(crate::STACK_SIZE * 1024 * 1024) // needs to increase thread stack size as dfms are kept on the stack
                    .spawn(move || {
                        // start mt here
                        // first get the point iterator for each tile
                        // pass that iterator to compute map objects
                        steps::compute_map_objects(
                            sender.clone(),
                            map.clone(),
                            &map_params,
                            tile_paths,
                            ref_point,
                            tile_cut_bounds,
                            num_threads,
                        );
                    })
                    .unwrap(),
            );
        }
        for handle in thread_handles {
            handle.join().unwrap();
        }

        // join threads here

        // merge line symbols
        {
            let map_guard = map.lock().unwrap();
            map_guard.merge_lines(crate::MERGE_DELTA);
        }

        sender.send(FrontendTask::ProgressBar(ProgressBar::Finish));
    }

    let omap = Arc::<Mutex<Omap>>::into_inner(map)
        .expect("Could not get inner value of arc, stray refrence somewhere")
        .into_inner()
        .expect("Map mutex poisoned, a thread panicked while holding mutex");

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

    sender
        .send(FrontendTask::TaskComplete(TaskDone::MakeMap))
        .unwrap();
}
