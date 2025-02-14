use crate::{
    comms::messages::*,
    parameters::{FileParameters, MapParameters},
};

use std::sync::mpsc::Sender;

pub fn make_map(
    sender: Sender<FrontendTask>,
    _map_params: MapParameters,
    _file_params: FileParameters,
    _polygon_filter: Option<geo::Polygon>,
) {
    sender
        .send(FrontendTask::Log("Map Generation!".to_string()))
        .unwrap();

    sender
        .send(FrontendTask::ProgressBar(ProgressBar::Start))
        .unwrap();
    let inc_size = 1. / 5.;
    for _ in 0..5 {
        std::thread::sleep(std::time::Duration::from_secs(1));
        sender
            .send(FrontendTask::ProgressBar(ProgressBar::Inc(inc_size)))
            .unwrap();
    }
    sender
        .send(FrontendTask::ProgressBar(ProgressBar::Finish))
        .unwrap();

    sender
        .send(FrontendTask::TaskComplete(TaskDone::MakeMap))
        .unwrap();
}

/*
use crate::parser::Args;
use crate::steps;
use omap::{Omap, Scale};

use std::{
    fs,
    sync::{Arc, Mutex},
};

pub fn run_cli() {
    let args = Args::parse_cli();

    // create output folder, nothing happens if directory already exists
    fs::create_dir_all(&args.output_directory).expect("Could not create output folder");

    let file_stem = args.in_file.file_stem().unwrap();
    let mut tiff_directory = args.output_directory.clone();
    tiff_directory.push("tiffs");
    if args.write_tiff {
        fs::create_dir_all(&tiff_directory).expect("Could not create output folder");
    }

    println!("Running on {} threads", args.threads);

    // step 0: figure out spatial relationships of the lidar files, assuming they are divided from a big lidar-project by a square-ish grid
    let (laz_neighbour_map, laz_paths, ref_point) = steps::map_laz(args.in_file.clone());
    let laz_paths = Arc::new(laz_paths);

    // create map
    let map = Arc::new(Mutex::new(Omap::new(ref_point, None, Scale::S15_000)));

    for fi in 0..laz_paths.len() {
        println!("\n***********************************************");
        println!("\t Processing Lidar-file {} of {}", fi + 1, laz_paths.len());
        println!("\t{:?}", laz_paths[fi].file_name().unwrap());
        println!("-----------------------------------------------");

        tiff_directory.push(laz_paths[fi].file_stem().unwrap());
        if args.write_tiff {
            fs::create_dir_all(&tiff_directory).expect("Could not create output folder");
        }

        println!("Subtiling file...");
        // step 1: preprocess lidar-file, retile into TILE_SIZExTILE_SIZEm tiles
        //         with at least MIN_NEIGHBOUR_MARGINm overlap on all sides


        let (tile_paths, tile_cut_bounds) = steps::retile_laz(
            args.threads,
            &laz_neighbour_map[fi],
            laz_paths.clone(),
        );

        println!("Computing map features...");

        steps::compute_map_objects(
            map.clone(),
            &args,
            tile_paths,
            ref_point,
            tile_cut_bounds,
            &tiff_directory,
        );

        // delete all sub-tiles
        fs::remove_dir_all(laz_paths[fi].with_extension(""))
            .expect("Could not remove dir with sub-tiled las-file");

        tiff_directory.pop();
    }

    // todo!: merge all line objects across boundaries

    // save map to file
    println!("\nWriting omap file...");
    Arc::<Mutex<Omap>>::into_inner(map)
        .expect("Could not get inner value of arc, stray refrence somewhere")
        .into_inner()
        .expect("Map mutex poisoned, a thread paniced while holding mutex")
        .write_to_file(file_stem, &args.output_directory, Some(crate::BEZIER_ERROR));
    println!("Done!");
}
*/
