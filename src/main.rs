#![feature(portable_simd)]

//mod c2hm;
mod geometry;
mod map;
mod matrix;
mod parser;
mod raster;
mod steps;

use map::{Omap, Symbol};
use parser::Args;

use std::{fs, sync::Arc};

fn main() {
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
    const NEIGHBOUR_MARGIN: f64 = 14.;
    const TILE_SIZE: f64 = 128.;

    // create output folders, nothing happens if directory already exists
    fs::create_dir_all(&output_directory).expect("Could not create output folder");

    let file_stem = in_file.file_stem().unwrap();
    let mut tiff_directory = output_directory.clone();
    tiff_directory.push("tiffs");
    if write_tiff {
        fs::create_dir_all(&tiff_directory).expect("Could not create output folder");
    }

    println!("Running on {} threads", num_threads);
    println!("\nMapping input lidar file(s) relations...");
    // step 0: figure out lidar file relationships
    let (laz_neighbour_map, laz_paths, ref_point) = steps::map_laz(in_file.clone());

    // create map
    let mut map = Omap::new(file_stem, &output_directory, ref_point);

    for fi in 0..laz_paths.len() {
        println!("***********************************************");
        println!(
            "\t Processing Lidar-file {} of {}...",
            fi + 1,
            laz_paths.len()
        );
        println!("\t{:?}", laz_paths[fi].file_name().unwrap());
        println!("-----------------------------------------------");

        // step 1: preprocess lidar-file, retile into 128mx128m tiles with 14m overlap on all sides
        let tile_paths = steps::retile_laz(
            &laz_neighbour_map[fi],
            &laz_paths,
            TILE_SIZE,
            NEIGHBOUR_MARGIN,
        );

        if write_tiff {
            tiff_directory.push(laz_paths[fi].file_stem().unwrap())
        }

        let pb = ProgressBar::new(tile_paths.len());
        pb.set_style(
            ProgressStyle::default_bar()
                .template("[{elapsed_precise}] [{bar:40.white/green}] ({eta})")
                .unwrap()
                .progress_chars("=>-"),
        );

        for tile_path in tile_paths {
            // step 2: read each laz file and its neighbours and build point-cloud
            let (xyzir, point_tree, convex_hull, width, height, tl) =
                steps::read_laz(&tile_path, &ref_point, cell_size, dist_to_hull_epsilon);

            // step 3: compute the DFMs
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
            steps::compute_vegetation(
                &drm,
                None,
                Some(1.2),
                &convex_hull,
                dist_to_hull_epsilon,
                simplify_epsilon,
                Symbol::RoughOpenLand,
                225.,
                &mut map,
            );

            steps::compute_vegetation(
                &drm,
                Some(2.1),
                None, //Some(3.0),
                &convex_hull,
                dist_to_hull_epsilon,
                simplify_epsilon,
                Symbol::LightGreen,
                225.,
                &mut map,
            );

            steps::compute_vegetation(
                &drm,
                Some(3.0),
                None, //Some(4.0),
                &convex_hull,
                dist_to_hull_epsilon,
                simplify_epsilon,
                Symbol::MediumGreen,
                110.,
                &mut map,
            );

            steps::compute_vegetation(
                &drm,
                Some(4.0),
                None,
                &convex_hull,
                dist_to_hull_epsilon,
                simplify_epsilon,
                Symbol::DarkGreen,
                64.,
                &mut map,
            );

            // step 6: compute cliffs
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
                steps::save_tiffs(
                    Arc::unwrap_or_clone(dem),
                    Arc::unwrap_or_clone(grad_dem),
                    Arc::unwrap_or_clone(dim),
                    Arc::unwrap_or_clone(drm),
                    &ref_point,
                    tile_path.file_stem().unwrap(),
                    &tiff_directory,
                );
            }
            pb.inc(1);
        }
    }

    // save map to file
    println!("Writing omap file...");
    map.write_to_file();
    println!("Done!");
}
