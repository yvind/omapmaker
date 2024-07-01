mod dfm;
mod geometry;
mod matrix;
mod omap;
mod parser;

use dfm::{Dfm, FieldType};
use geometry::{Contour, Point2D, Point5D, PointCloud5D, Polygon, PolygonTrigger};
use omap::Omap;
use parser::Args;

use clap::Parser;
use kiddo::{immutable::float::kdtree::ImmutableKdTree, SquaredEuclidean};
use las::{point::Classification, Bounds, Read, Reader};
use rand::random;
use std::{fs, path::Path, time::Instant};

fn main() {
    let args = Args::parse();

    let las_file = args.in_file;
    let output_directory = args.output_directory;
    let contour_interval: f64 = if args.form_lines {
        args.contour_interval / 2.
    } else {
        args.contour_interval
    };
    let cell_size = args.grid_size;
    let basemap_interval = args.basemap_contours;

    assert!(contour_interval > 0.);

    let las_path = Path::new(&las_file);
    if !(las_path.extension().unwrap() == "laz" || las_path.extension().unwrap() == "las")
        || !las_path.exists()
    {
        panic!("Invalid input file path");
    }
    let file_stem = Path::new(las_path.file_name().unwrap())
        .file_stem()
        .unwrap()
        .to_str()
        .unwrap();

    if !(output_directory == "./".to_string()) {
        fs::create_dir_all(&output_directory).expect("Could not create output folder");
    }

    let mut las_reader = Reader::from_path(&las_file).expect("Could not read givem laz/las file");

    let header = las_reader.header();
    let las_bounds: Bounds = header.bounds();
    println!("Number of points: {:?}", header.number_of_points());
    println!("Point cloud {:?}", las_bounds);

    let ref_point = Point2D {
        x: (las_bounds.min.x + las_bounds.max.x) / 2.0,
        y: (las_bounds.min.y + las_bounds.max.y) / 2.0,
    };

    println!("Filtering points...");
    let xyzir: PointCloud5D = PointCloud5D::new(
        las_reader
            .points()
            .map(|r| r.unwrap())
            .filter_map(|p| {
                (p.classification == Classification::Ground
                    || p.classification == Classification::Water)
                    .then(|| Point5D {
                        x: p.x + 2. * (random::<f64>() - 0.5) / 1000. - ref_point.x,
                        y: p.y + 2. * (random::<f64>() - 0.5) / 1000. - ref_point.y,
                        z: p.z,
                        i: p.intensity as u32,
                        r: p.return_number as u8,
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

    println!("Building KD-tree...");
    let point_cloud: ImmutableKdTree<f64, usize, 2, 32> =
        ImmutableKdTree::new_from_slice(&xyzir.to_2d_slice());

    println!("Computing DFMs...");
    let now = Instant::now();

    let (width, height, map_bounds) = xyzir.get_dfm_dimensions(cell_size);
    let tl: Point2D = Point2D {
        x: map_bounds.min.x,
        y: map_bounds.max.y,
    };
    let convex_hull: Contour = xyzir.bounded_convex_hull(cell_size, &map_bounds);

    let num_neighbours = 32;

    let mut dem: Dfm = Dfm::new(width, height, tl, cell_size);
    let mut grad_dem: Dfm = Dfm::new(width, height, tl, cell_size);
    let mut drm: Dfm = Dfm::new(width, height, tl, cell_size);
    let mut grad_drm: Dfm = Dfm::new(width, height, tl, cell_size);
    let mut dim: Dfm = Dfm::new(width, height, tl, cell_size);
    let mut grad_dim: Dfm = Dfm::new(width, height, tl, cell_size);

    for y in 0..height {
        for x in 0..width {
            let coords: Point2D = dem.index2coord(x, y).unwrap();
            if !convex_hull.contains(&coords).unwrap() {
                continue;
            }

            // slow due to very many lookups
            let nearest_n =
                point_cloud.nearest_n::<SquaredEuclidean>(&[coords.x, coords.y], num_neighbours);
            let neighbours: Vec<usize> = nearest_n.iter().map(|n| n.item).collect();

            // slow due to matrix inversion
            // gradients are almost for free
            let (elev, grad_elev) =
                xyzir.interpolate_field(FieldType::Elevation, &neighbours, &coords, 0.01);
            let (intens, grad_intens) =
                xyzir.interpolate_field(FieldType::Intensity, &neighbours, &coords, 0.1);
            let (rn, grad_rn) =
                xyzir.interpolate_field(FieldType::ReturnNumber, &neighbours, &coords, 0.1);

            dem.field[y][x] = elev;
            grad_dem.field[y][x] = grad_elev;
            dim.field[y][x] = intens;
            grad_dim.field[y][x] = grad_intens;
            drm.field[y][x] = rn;
            grad_drm.field[y][x] = grad_rn;
        }
    }
    let elapsed = now.elapsed();
    println!("Elapsed time in DFM generation: {:?}", elapsed);

    println!("Computing contours...");

    if basemap_interval > 0. {
        println!("Computing basemap contours...");

        let bm_levels = ((las_bounds.max.z - las_bounds.min.z) / basemap_interval).ceil() as u64;

        for i in 0..bm_levels {
            let bm_level = i as f64 * basemap_interval + las_bounds.min.z;

            let bm_contours = dem.marching_squares(bm_level).unwrap();

            for bm_c in bm_contours {}
        }
    }

    println!("Computing yellow...");
    let return_contours: Vec<Contour> = drm.marching_squares(1.2).unwrap();
    let return_polygons: Vec<Polygon> =
        Polygon::from_contours(return_contours, &convex_hull, PolygonTrigger::Below, 225.);

    if args.write_tiff {
        println!("Writing gridded Las-fields and their gradients to Tiff files...");
        dem.write_to_tiff(format!("dem_{}", &file_stem), &output_directory);
        grad_dem.write_to_tiff(format!("grad_dem_{}", &file_stem), &output_directory);
        dim.write_to_tiff(format!("dim_{}", &file_stem), &output_directory);
        grad_dim.write_to_tiff(format!("grad_dim_{}", &file_stem), &output_directory);
        drm.write_to_tiff(format!("drm_{}", &file_stem), &output_directory);
        grad_drm.write_to_tiff(format!("grad_drm_{}", &file_stem), &output_directory);
    }
}
