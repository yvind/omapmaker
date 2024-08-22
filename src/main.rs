#![feature(portable_simd)]

mod dfm;
mod geometry;
mod map;
mod matrix;
mod parser;
mod steps;

use geometry::{Line, Point2D, Polygon, PolygonTrigger};
use map::{AreaObject, LineObject, MapObject, Omap, Symbol};
use parser::Args;

use std::{fs, path::PathBuf, time::Instant};

fn main() {
    // step 0: read inputs from command line
    let (
        in_file,
        output_directory,
        contour_interval,
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
    let mut map = Omap::new(file_stem, output_directory, ref_point);

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
        let now = Instant::now();
        let (dem, grad_dem, drm, grad_drm, dim, grad_dim) = steps::compute_dfms(
            point_tree.clone(),
            xyzir.clone(),
            convex_hull.clone(),
            num_threads,
            (width, height, cell_size, tl),
            simd,
        );
        println!("Elapsed time in DFM generation: {:?}", now.elapsed());

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
                &map,
            );

            /*
            for c in contours {
                let mut bm_object = LineObject::from_line(c, Symbol::BasemapContour);

                bm_object.add_auto_tag();
                bm_object.add_tag("Elevation", format!("{:.2}", level).as_str());

                map.add_object(bm_object);
            }
            */
        }

        // step 5: compute vegetation
        println!("Computing yellow...");
        steps::compute_open_land(
            &drm,
            1.2,
            dist_to_hull_epsilon,
            &convex_hull,
            simplify_epsilon,
            &map,
        );

        // step 6: using
        /*
        if write_tiff {
            // serialize and save the tiff files in a tmp folder so they can be merged after all laz files are processed
            println!("Writing gridded Las-fields and their gradients to Tiff files...");
            dem.write_to_tiff(format!("dem_{}", &file_stem), &output_directory, &ref_point);
            grad_dem.write_to_tiff(
                format!("grad_dem_{}", &file_stem),
                &output_directory,
                &ref_point,
            );
            dim.write_to_tiff(format!("dim_{}", &file_stem), &output_directory, &ref_point);
            grad_dim.write_to_tiff(
                format!("grad_dim_{}", &file_stem),
                &output_directory,
                &ref_point,
            );
            drm.write_to_tiff(format!("drm_{}", &file_stem), &output_directory, &ref_point);
            grad_drm.write_to_tiff(
                format!("grad_drm_{}", &file_stem),
                &output_directory,
                &ref_point,
            );
        }
        */
    }

    // save map to file
    println!("Writing omap file...");
    map.write_to_file();
    println!("Done!");
}
