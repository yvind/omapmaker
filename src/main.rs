#![feature(portable_simd)]

mod dfm;
mod geometry;
mod matrix;
mod omap;
mod parser;

use dfm::{Dfm, FieldType};
use geometry::{Line, Point2D, PointCloud, PointLaz, Polygon, PolygonTrigger};
use omap::{AreaObject, LineObject, MapObject, Omap, Symbol};
use parser::Args;

use clap::Parser;
use kiddo::{immutable::float::kdtree::ImmutableKdTree, SquaredEuclidean};
use las::{point::Classification, Bounds, Read, Reader};
use rand::random;
use std::{
    fs,
    path::Path,
    sync::{mpsc, Arc},
    thread,
    time::Instant,
};

fn main() {
    // read inputs

    let args = Args::parse();

    let las_path = Path::new(&args.in_file);
    let output_directory = args.output_directory;
    let contour_interval: f64 = if args.form_lines {
        args.contour_interval / 2.
    } else {
        args.contour_interval
    };
    let cell_size = args.grid_size;
    let basemap_interval = args.basemap_contours;
    let num_threads: usize = if args.threads > 2 { args.threads } else { 2 };

    let _simd = args.simd;

    let dist_to_hull_epsilon = 2. * cell_size;

    assert!(contour_interval >= 1.);

    // create output folder and open laz file

    if !(output_directory == "./".to_string()) {
        fs::create_dir_all(&output_directory).expect("Could not create output folder");
    }

    let mut las_reader = Reader::from_path(&las_path).expect("Could not read givem laz/las file");

    let file_stem = Path::new(las_path.file_name().unwrap())
        .file_stem()
        .unwrap()
        .to_str()
        .unwrap();

    // read laz file and build pointcloud and KD-tree

    let header = las_reader.header();
    let mut las_bounds: Bounds = header.bounds();
    println!("Number of points: {:?}", header.number_of_points());
    println!("Point cloud {:?}", las_bounds);

    let ref_point = Point2D {
        x: ((las_bounds.min.x + las_bounds.max.x) / 2.).round(),
        y: ((las_bounds.min.y + las_bounds.max.y) / 2.).round(),
    };

    las_bounds.max.x -= ref_point.x;
    las_bounds.min.x -= ref_point.x;
    las_bounds.max.y -= ref_point.y;
    las_bounds.min.y -= ref_point.y;

    println!("Filtering points...");
    let mut xyzir = PointCloud::new(
        las_reader
            .points()
            .map(|r| r.unwrap())
            .filter_map(|p| {
                (p.classification == Classification::Ground
                    || p.classification == Classification::Water)
                    .then(|| PointLaz {
                        x: p.x + 2. * (random::<f64>() - 0.5) / 1000. - ref_point.x,
                        y: p.y + 2. * (random::<f64>() - 0.5) / 1000. - ref_point.y,
                        z: p.z,
                        i: p.intensity as u32,
                        r: p.return_number,
                        c: if p.classification == Classification::Ground {
                            2
                        } else {
                            9
                        },
                        n: p.number_of_returns,
                    })
            }) // add noise on the order of mm for KD-tree stability
            .collect(),
        las_bounds.clone(),
    );

    let sqm: f64 = (las_bounds.max.x - las_bounds.min.x) * (las_bounds.max.y - las_bounds.min.y);
    println!("Number of ground points: {}", xyzir.len());
    println!("Area: {:.3} sqkm", sqm / 1_000_000.);
    println!(
        "Ground point density: {:.2} points/sqm",
        xyzir.len() as f64 / sqm
    );

    println!("Building Kd-tree...");
    let (width, height, map_bounds) = xyzir.get_dfm_dimensions(cell_size);
    let tl: Point2D = Point2D {
        x: map_bounds.min.x,
        y: map_bounds.max.y,
    };
    let convex_hull: Line = xyzir.bounded_convex_hull(cell_size, &map_bounds);
    let point_tree: ImmutableKdTree<f64, usize, 2, 32> =
        ImmutableKdTree::new_from_slice(&xyzir.to_2d_slice());

    // Compute DFMs using multiple threads

    println!("Computing DFMs...");
    let now = Instant::now();

    let mut dem = Dfm::new(width, height, tl, cell_size);
    let mut grad_dem = Dfm::new(width, height, tl, cell_size);
    let mut drm = Dfm::new(width, height, tl, cell_size);
    let mut grad_drm = Dfm::new(width, height, tl, cell_size);
    let mut dim = Dfm::new(width, height, tl, cell_size);
    let mut grad_dim = Dfm::new(width, height, tl, cell_size);

    let pt_arc = Arc::new(point_tree);
    let pc_arc = Arc::new(xyzir);
    let ch_arc = Arc::new(convex_hull.clone());
    let dem_arc = Arc::new(dem.clone());

    compute_dfms_multithread(
        num_threads,
        &pt_arc,
        &pc_arc,
        &ch_arc,
        &dem_arc,
        &mut dem,
        &mut dim,
        &mut drm,
        &mut grad_dem,
        &mut grad_dim,
        &mut grad_drm,
    );
    println!("Elapsed time in DFM generation: {:?}", now.elapsed());

    // create map and the map objects and add them to the map
    let mut map = Omap::new(file_stem, &output_directory.as_str(), ref_point);

    let dem = Arc::new(dem);

    //println!("Computing contours...");
    if basemap_interval >= 0.1 {
        println!("Computing basemap contours...");

        compute_basemap_contours_multithread(
            num_threads,
            las_bounds.min.z,
            las_bounds.max.z,
            basemap_interval,
            &dem,
            &mut map,
        );
    }

    println!("Computing yellow...");
    let yellow_contours = drm.marching_squares(1.2).unwrap();
    let yellow_polygons = Polygon::from_contours(
        yellow_contours,
        &convex_hull,
        PolygonTrigger::Below,
        225.,
        dist_to_hull_epsilon,
    );

    for polygon in yellow_polygons {
        let yellow_object = AreaObject::from_polygon(polygon, Symbol::RoughOpenLand);
        map.add_object(yellow_object);
    }

    // write dfms to tiff
    if args.write_tiff {
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

    // save map to file
    println!("Writing omap file...");
    map.write_to_file();
    println!("Done!");
}

fn compute_dfms_multithread(
    num_threads: usize,
    pt_arc: &Arc<ImmutableKdTree<f64, usize, 2, 32>>,
    pc_arc: &Arc<PointCloud>,
    ch_arc: &Arc<Line>,
    dem_arc: &Arc<Dfm>,
    dem: &mut Dfm,
    dim: &mut Dfm,
    drm: &mut Dfm,
    grad_dem: &mut Dfm,
    grad_dim: &mut Dfm,
    grad_drm: &mut Dfm,
) {
    let (sender, receiver) = mpsc::channel();

    let mut thread_handles = vec![];
    let num_neighbours = 32;

    for i in 0..(num_threads - 1) {
        let pt_ref = pt_arc.clone();
        let pc_ref = pc_arc.clone();
        let ch_ref = ch_arc.clone();
        let dem_ref = dem_arc.clone();

        let thread_sender = sender.clone();

        thread_handles.push(thread::spawn(move || -> () {
            let mut y_index = i;

            while y_index < dem_ref.height {
                for x_index in 0..dem_ref.width {
                    let coords: Point2D = dem_ref.index2coord(x_index, y_index).unwrap();
                    if !ch_ref.contains(&coords).unwrap() {
                        continue;
                    }

                    // slow due to very many lookups
                    let nearest_n =
                        pt_ref.nearest_n::<SquaredEuclidean>(&[coords.x, coords.y], num_neighbours);
                    let neighbours: Vec<usize> = nearest_n.iter().map(|n| n.item).collect();

                    // slow due to matrix inversion
                    // gradients are almost for free
                    let (elev, grad_elev) =
                        pc_ref.interpolate_field(FieldType::Elevation, &neighbours, &coords, 0.01);
                    let (intens, grad_intens) =
                        pc_ref.interpolate_field(FieldType::Intensity, &neighbours, &coords, 0.1);
                    let (rn, grad_rn) = pc_ref.interpolate_field(
                        FieldType::ReturnNumber,
                        &neighbours,
                        &coords,
                        0.1,
                    );

                    thread_sender.send((elev, y_index, x_index, 0)).unwrap();
                    thread_sender.send((intens, y_index, x_index, 1)).unwrap();
                    thread_sender.send((rn, y_index, x_index, 2)).unwrap();
                    thread_sender
                        .send((grad_elev, y_index, x_index, 3))
                        .unwrap();
                    thread_sender
                        .send((grad_intens, y_index, x_index, 4))
                        .unwrap();
                    thread_sender.send((grad_rn, y_index, x_index, 5)).unwrap();
                }

                y_index += num_threads - 1;
            }
            drop(thread_sender);
        }));
    }
    drop(sender);

    for (value, yi, xi, ii) in receiver.iter() {
        match ii {
            0 => dem.field[yi][xi] = value,
            1 => dim.field[yi][xi] = value,
            2 => drm.field[yi][xi] = value,
            3 => grad_dem.field[yi][xi] = value,
            4 => grad_dim.field[yi][xi] = value,
            _ => grad_drm.field[yi][xi] = value,
        }
    }

    for t in thread_handles {
        t.join().unwrap();
    }
}

fn compute_basemap_contours_multithread(
    num_threads: usize,
    min_z: f64,
    max_z: f64,
    basemap_interval: f64,
    dem_arc: &Arc<Dfm>,
    map: &mut Omap,
) {
    let bm_levels = ((max_z - min_z) / basemap_interval).ceil() as usize;

    let (sender, receiver) = mpsc::channel();
    let mut thread_handles = vec![];

    for i in 0..(num_threads - 1) {
        let dem_ref = dem_arc.clone();

        let thread_sender = sender.clone();

        thread_handles.push(thread::spawn(move || -> () {
            let mut c_index = i;

            while c_index < bm_levels {
                let bm_level = c_index as f64 * basemap_interval + min_z.floor();

                let bm_contours = dem_ref.marching_squares(bm_level).unwrap();

                thread_sender.send((bm_contours, bm_level)).unwrap();

                c_index += num_threads - 1;
            }
            drop(thread_sender);
        }));
    }
    drop(sender);

    for (contours, level) in receiver.iter() {
        for c in contours {
            let mut bm_object = LineObject::from_line(c, Symbol::BasemapContour);

            bm_object.add_auto_tag();
            bm_object.add_tag("Elevation", format!("{:.2}", level).as_str());

            map.add_object(bm_object);
        }
    }

    for handle in thread_handles {
        handle.join().unwrap();
    }
}
