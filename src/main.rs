#![feature(portable_simd)]
#![allow(clippy::needless_range_loop)]

mod dfm;
mod geometry;
mod map;
mod matrix;
mod parser;
mod steps;

use map::Omap;
use parser::Args;

use std::fs;

fn main() {
    // step 0: read inputs from command line
    let (
        in_file,
        output_directory,
        _contour_interval,
        cell_size,
        basemap_interval,
        num_threads,
        simd,
        simplify_epsilon,
        write_tiff,
    ) = Args::parse_cli();
    let dist_to_hull_epsilon = 2. * cell_size;
    let neighbour_margin = 20.;

    // create output folder, nothing happens if directory already exists
    fs::create_dir_all(&output_directory).expect("Could not create output folder");

    if num_threads > 1 {
        println!("Running on {num_threads} threads");
    } else {
        println!("Running single-threaded");
    }

    println!("Preparing input lidar file(s) for processing...");
    // step 1: prepare for processing lidar-files
    let (laz_neighbour_map, las_paths, ref_point, file_stem) =
        steps::prepare_laz(in_file, neighbour_margin);

    // create map
    let mut map = Omap::new(&file_stem, &output_directory, ref_point);

    for fi in 0..las_paths.len() {
        println!("********");
        println!("Processing Lidar-file {} of {}...", fi + 1, las_paths.len());
        println!("********");

        // step 2: read each laz file and its neighbours and build point-cloud
        let (xyzir, point_tree, convex_hull, width, height, tl) = steps::read_laz(
            &laz_neighbour_map[fi],
            &las_paths,
            &ref_point,
            cell_size,
            neighbour_margin,
            dist_to_hull_epsilon,
        );

        // step 3: compute the DFMs
        println!("Computing DFMs...");
        let (dem, grad_dem, drm, _, dim, _) = steps::compute_dfms(
            point_tree.clone(),
            xyzir.clone(),
            convex_hull.clone(),
            num_threads,
            (width, height, cell_size, tl),
            simd,
        );

        // step 4: contour generation
        if basemap_interval >= 0.1 {
            println!("Computing basemap contours...");

            // TODO: make this a contour hierarchy object
            steps::compute_basemap(
                num_threads,
                xyzir.bounds.min.z,
                xyzir.bounds.max.z,
                basemap_interval,
                &dem,
                &convex_hull,
                dist_to_hull_epsilon,
                simplify_epsilon,
                &mut map,
            );
        }

        // step 5: compute vegetation
        println!("Computing yellow...");
        steps::compute_open_land(
            &drm,
            1.2,
            dist_to_hull_epsilon,
            &convex_hull,
            simplify_epsilon,
            &mut map,
        );

        // step 6: save dfms
        if write_tiff {
            println!("Writing gridded Las-fields to Tiff files...");
            steps::save_tiffs(
                dem,
                grad_dem,
                dim,
                drm,
                &ref_point,
                &file_stem,
                &output_directory,
            );
        }
    }

    // save map to file
    println!("Writing omap file...");
    map.write_to_file();
    println!("Done!");
}
