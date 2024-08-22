#![feature(portable_simd)]

mod dfm;
mod geometry;
mod matrix;
mod omap;
mod parser;
mod steps;

use dfm::Dfm;
use geometry::{Line, Point2D, PointCloud, Polygon, PolygonTrigger};
use omap::{AreaObject, LineObject, MapObject, Omap, Symbol};
use parser::Args;

use std::{
    fs,
    path::Path,
    sync::{mpsc, Arc},
    thread,
    time::Instant,
};

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

    // create output folder
    if !(output_directory == "./".into()) && output_directory.exists() {
        fs::create_dir_all(&output_directory).expect("Could not create output folder");
    }

    if num_threads > 1 {
        println!("Running on {num_threads} threads");
    } else {
        println!("Running single-threaded");
    }

    println!("Preparing input lidar file(s) for processing...");
    // step 1: prepare for processing lidar-files
    let (laz_neighbour_map, las_paths, ref_point, file_stem) =
        steps::prepare_laz(in_file);

    // create map
    let mut map = Omap::new(file_stem, output_directory.as_path(), ref_point);

    for fi in 0..las_paths.len() {
        println!("********");
        println!("Processing Lidar-file {} of {}...", fi + 1, las_paths.len());
        println!("********");
        // step 2: read each laz file and its neighbours and build point-cloud
        let (mut xyzir, point_tree, local_ref) =
            steps::read_laz(fi, &las_paths, &laz_neighbour_map[fi], &ref_point);

        // step 3: compute the DFMs
        println!("Computing DFMs...");
        let now = Instant::now();
        let (width, height, map_bounds) = xyzir.get_dfm_dimensions(cell_size);
        let tl = Point2D {
            x: map_bounds.min.x,
            y: map_bounds.max.y,
        };
        let convex_hull =
            Arc::new(xyzir.bounded_convex_hull(cell_size, &map_bounds, dist_to_hull_epsilon * 2.));
        // has side effects bc sorting in place

        let (dem, grad_dem, drm, grad_drm, dim, grad_dim) =
            steps::compute_dfms(&point_tree, num_threads, width, height, cell_size, tl);
        println!("Elapsed time in DFM generation: {:?}", now.elapsed());

        // step 4: contour generation
        if basemap_interval >= 0.1 {
            println!("Computing basemap contours...");

            bm = steps::compute_basemap(
                num_threads,
                las_bounds.min.z,
                las_bounds.max.z,
                basemap_interval,
            );

            map.
        }

        // step 5: compute vegetation
        println!("Computing yellow...");
        steps::compute_open_land(
            &drm,
            1.3,
            dist_to_hull_epsilon,
            &convex_hull,
            simplify_epsilon,
            &mut map,
        );

        // step 6: using 

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
    }

    // save map to file
    println!("Writing omap file...");
    map.write_to_file();
    println!("Done!");
}

fn compute_intensity_polygon(
    dim: &Dfm,
    intensity_threshold: f64,
    dist_to_hull_epsilon: f64,
    convex_hull: &Line,
    simplify_epsilon: f64,
    symbol: Symbol,
    trigger: PolygonTrigger,
    min_size: f64,
    map: &mut Omap,
) {
    let mut int_contours = dim.marching_squares(intensity_threshold).unwrap();

    for yc in int_contours.iter_mut() {
        yc.fix_ends_to_line(&convex_hull, dist_to_hull_epsilon);
    }

    let int_hint = dim.field[dim.height / 2][dim.width / 2] > intensity_threshold;
    let int_polygons = Polygon::from_contours(
        int_contours,
        &convex_hull,
        trigger,
        min_size,
        dist_to_hull_epsilon,
        int_hint,
    );

    for mut polygon in int_polygons {
        if simplify_epsilon > 0. {
            polygon.simplify(simplify_epsilon);
        }
        let mut int_object = AreaObject::from_polygon(polygon, symbol);
        int_object.add_auto_tag();
        map.add_object(int_object);
    }
}
