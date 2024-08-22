#![feature(portable_simd)]
#![allow(clippy::needless_range_loop)]

mod geometry;
mod map;
mod matrix;
mod parser;
mod raster;
mod steps;

use map::Omap;
use parser::Args;

use std::{fs, path::Path};

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

    // create output folders, nothing happens if directory already exists
    fs::create_dir_all(&output_directory).expect("Could not create output folder");

    let mut tiff_directory = output_directory.clone();
    tiff_directory.push("tiffs");
    if write_tiff {
        fs::create_dir_all(&tiff_directory).expect("Could not create output folder");
    }

    println!("Preparing input lidar file(s) for processing...");
    // step 1: prepare for processing lidar-files
    let (laz_neighbour_map, laz_paths, ref_point, file_stem) =
        steps::prepare_laz(in_file, neighbour_margin);

    // create map
    let mut map = Omap::new(&file_stem, &output_directory, ref_point);

    for fi in 0..laz_paths.len() {
        println!("***********************************************");
        println!(
            "\t Processing Lidar-file {} of {}...",
            fi + 1,
            laz_paths.len()
        );
        println!("\t{:?}", laz_paths[fi].file_name().unwrap());
        println!("-----------------------------------------------");

        let las_name = Path::new(&laz_paths[fi].file_name().unwrap())
            .file_stem()
            .unwrap()
            .to_owned();

        // step 2: read each laz file and its neighbours and build point-cloud
        let (xyzir, point_tree, convex_hull, width, height, tl) = steps::read_laz(
            &laz_neighbour_map[fi],
            &laz_paths,
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

        // TODO: make thresholds adaptive to local terrain ( create a smoothed version of the dfm and use that value to adapt threshold)
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

        // step 6: compute cliffs
        println!("Computing cliffs...");
        steps::compute_cliffs(
            &grad_dem,
            0.7,
            dist_to_hull_epsilon,
            &convex_hull,
            simplify_epsilon,
            &mut map,
        );

        // step 7: save dfms
        if write_tiff {
            println!("Writing gridded Las-fields to Tiff files...");
            steps::save_tiffs(
                dem,
                grad_dem,
                dim,
                drm,
                &ref_point,
                &las_name,
                &tiff_directory,
            );
        }
    }

    // save map to file
    println!("Writing omap file...");
    map.write_to_file();
    println!("Done!");
}
