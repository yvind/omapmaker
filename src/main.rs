#![feature(portable_simd)]

//mod c2hm;
mod geometry;
mod map;
mod matrix;
mod parser;
mod raster;
mod steps;

use map::Omap;
use parser::Args;

use indicatif::{ProgressBar, ProgressStyle};
use std::{
    fs,
    sync::{Arc, Mutex},
};

// must be constant across training and inference if AI is to be applied
const TILE_SIZE_USIZE: usize = 128;
const NEIGHBOUR_MARGIN_USIZE: usize = 14;
const INV_CELL_SIZE_USIZE: usize = 2; // test 1, 2 or 4

const CELL_SIZE: f64 = 1. / INV_CELL_SIZE_USIZE as f64;
const TILE_SIZE: f64 = TILE_SIZE_USIZE as f64;
const NEIGHBOUR_MARGIN: f64 = NEIGHBOUR_MARGIN_USIZE as f64;

fn main() {
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

    // step 0: figure out lidar files spatial relationships, assuming they are divided from a big lidar-project by a square-ish grid
    let (laz_neighbour_map, laz_paths, ref_point, _file_cut_bounds) =
        steps::map_laz(args.in_file.clone());

    // create map
    let map = Arc::new(Mutex::new(Omap::new(ref_point)));

    for fi in 0..laz_paths.len() {
        println!("\n***********************************************");
        println!("\t Processing Lidar-file {} of {}", fi + 1, laz_paths.len());
        println!("\t{:?}", laz_paths[fi].file_name().unwrap());
        println!("-----------------------------------------------");
        println!("Subtiling file...");

        tiff_directory.push(laz_paths[fi].file_stem().unwrap());

        // step 1: preprocess lidar-file, retile into 128mx128m tiles with 14m overlap on all sides
        let (tile_paths, tile_cut_bounds) =
            steps::retile_laz(args.threads, &laz_neighbour_map[fi], &laz_paths);

        println!("Computing map features...");

        let pb = ProgressBar::new(tile_paths.len() as u64);
        pb.set_style(
            ProgressStyle::default_bar()
                .template("[{elapsed_precise}] [{bar:40.white/gray}] ({eta})")
                .unwrap()
                .progress_chars("=>."),
        );
        let pb = Arc::new(Mutex::new(pb));

        // refactor so that it returns a laz-file map where all tiles are cut, merged and
        // filtered for too small objects except those that touch the bounds of the laz
        // only merge polygons across tile boundaries if they are too small to be
        // on their own
        steps::compute_map_objects(
            map.clone(),
            &args,
            tile_paths,
            ref_point,
            tile_cut_bounds,
            &tiff_directory,
            pb.clone(),
        );

        // delete all sub-tiles
        fs::remove_dir_all(laz_paths[fi].with_extension(""))
            .expect("Could not remove dir with sub-tiled las-file");

        pb.lock().unwrap().finish();

        tiff_directory.pop();
    }
    // refactor:
    // merge all laz-file maps and filter out everything that's too small
    // only merge polygons across boundaries if they are too small to be
    // on their own

    // save map to file
    println!("\nWriting omap file...");
    Arc::<Mutex<Omap>>::into_inner(map)
        .expect("Could not get inner value of arc, stray refrence somewhere")
        .into_inner()
        .expect("Map mutex poisoned, a thread paniced while holding mutex")
        .write_to_file(file_stem, &args.output_directory);
    println!("Done!");
}
