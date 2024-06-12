mod geometry;
mod dxf_writer;
mod dfm;
mod matrix;
mod parser;

use geometry::{Contour, Polygon, PolygonTrigger};
use dfm::Dfm;
use parser::Args;

use clap::Parser;
use kiddo::{SquaredEuclidean, immutable::float::kdtree::ImmutableKdTree};
use las::{Bounds, point::Classification, Read, Reader};
use rand::random;
use std::{fs, fs::File, io::BufWriter, path::Path, time::Instant};

fn main(){
    let args = Args::parse();

    let las_file = args.in_file.clone();
    let output_directory = args.output_directory.clone();
    let contour_interval: f64 = if args.form_lines {args.contour_interval/2.} else {args.contour_interval};
    let cell_size = args.grid_size;
    let basemap_interval = args.basemap_contours;

    assert!(contour_interval > 0.);

    let las_path = Path::new(&las_file);
    if !( las_path.extension().unwrap() == "laz" || las_path.extension().unwrap() == "las" ) || !las_path.exists() {
        panic!("Invalid input file path");
    }
    let file_stem = las_path.file_name().unwrap().file_stem().unwrap().to_str().unwrap();

    fs::create_dir_all(&output_directory).expect("Could not create output folder");

    let mut las_reader = Reader::from_path(&las_file).expect("Could not read laz/las file");

    let header = las_reader.header();
    let las_bounds: Bounds = header.bounds();
    println!("Number of points: {:?}", header.number_of_points());
    println!("Point cloud {:?}", las_bounds);

    let ref_point = Point2D{ x: ( las_bounds.min.x + las_bounds.max.x ) / 2.0, y: ( las_bounds.min.y + las_bounds.max.y ) / 2.0 };

    println!("Filtering points...");
    let xyzir: PointCloud5D = PointCloud5D::from(las_reader.points()
                                .map(|r| r.unwrap())
                                .filter_map(|p| (p.classification == Classification::Ground || p.classification == Classification::Water)
                                .then(|| Point5D{x: p.x + 2.*(random::<f64>()-0.5)/1000. - ref_point.x, y: p.y + 2.*(random::<f64>()-0.5)/1000. - ref_point.y, z: p.z, i: p.intensity as f64, r: p.return_number as f64})) // add noise on the order of mm for KD-tree stability
                                .collect(), &las_bounds);
    let num_ground_points: usize = xyzir.len();
    println!("Number of ground points: {}", num_ground_points);

    let sqm: f64 = (las_bounds.max.x-las_bounds.min.x)*(las_bounds.max.y-las_bounds.min.y);
    println!("Area: {:.3} sqkm", sqm / 1_000_000.);

    let density: f64 = num_ground_points as f64 / sqm;
    println!("Ground point density: {:.2} points/sqm", density);

    println!("Building KD-tree...");
    let point_cloud: ImmutableKdTree<f64, usize, 2, 32> = ImmutableKdTree::new_from_slice(&xyzir.to_2D_slice());

    println!("Computing DFMs...");
    let now = Instant::now();

    let (width, height, map_bounds) = xyzir.get_dem_dimensions(cell_size);
    let tl: Point2D = Point2D{x: map_bounds.min.x, y: map_bounds.max.y};
    let convex_hull: Contour = xyzir.convex_hull(&map_bounds, cell_size);

    let num_neighbours = 32;

    let mut dem: Dfm = Dfm::new(width, height, tl, cell_size);
    let mut drm: Dfm = Dfm::new(width, height, tl, cell_size);
    let mut dim: Dfm = Dfm::new(width, height, tl, cell_size);

    for y in 0..height{
        for x in 0..width{
            let coords: Point2D = dem.index2coord(x, y).unwrap();
            if !convex_hull.contains(&coords){
                continue;
            }

            // slow due to very many lookups
            let nearest_n = point_cloud.nearest_n::<SquaredEuclidean>(&coords, num_neighbours);
            let neighbours: Vec<usize> = nearest_n.iter().map(|n| n.item).collect();

            // slow due to matrix inversion
            let elev: f64 = xyzir.interpolate_field(FieldType::Elevation, &neighbours, &coords, 0.01);
            let intens: f64 = xyzir.interpolate_field(FieldType::Intensity, &neighbours, &coords, 0.1);
            let rn: f64 = xyzir.interpolate_field(FieldType::ReturnNumber, &neighbours, &coords, 0.1);

            dem.field[y][x] = elev;
            drm.field[y][x] = rn;
            dim.field[y][x] = intens;
        }
    }
    let elapsed = now.elapsed();
    println!("Elapsed time in DFM generation: {:?}", elapsed);

    println!("Computing contours...");
    /*
    let mut restored_dem: Dfm = Dfm::new(width, height);

    let mut contours: Vec<Contour> = Vec::new();
    let mut level: f64 = (z_min / contour_interval).floor() * contour_interval;
    while level <= z_max{
        contours.append(marching_squares(&dem, level, &map_bounds, cell_size));
        level += contour_interval;
    }
    let triangulation: Cdt = Cdt::new(&contours, &map_bounds);

    for y in 0..height{
        for x in 0..width{
            let coords: Vertex = index2coord(x, y, &map_bounds, cell_size, height);

            restored_dem[y][x] = triangulation.interpolate_value(coords);
        }
    }

    */
    
    if basemap_interval > 0.{
        println!("Computing basemap...");
        let bf = File::create(&Path::new(&format!("{}/basemap_{}.dxf", output_directory, file_stem))).expect("Unable to create file");
        let mut bf = BufWriter::new(bf);

        dxf_write_header(&mut bf, &map_bounds);
        let mut level: f64 = (las_bounds.min.z / basemap_interval).floor() * basemap_interval;
        while level <= las_bounds.max.z{
            let contours: Vec<Contour> = marching_squares(&dem, level, &map_bounds, cell_size);
            for contour in contours{
                dxf_write_polyline(&mut bf, &contour);
                for vertex in contour.vertices{
                    dxf_write_vertex(&mut bf, &vertex);
                }
                dxf_write_end_sequence(&mut bf);
            }
            level += basemap_interval;
        }
        dxf_write_end_file(&mut bf);
    }


    let f = File::create(&Path::new(&format!("{}/{}.dxf", output_directory, file_stem))).expect("Unable to create file");
    let mut f = BufWriter::new(f);

    dxf_write_header(&mut f, &map_bounds);
    /*
    let mut level: f64 = (z_min / contour_interval).floor() * contour_interval;
    while level <= z_max{
        let contours: Vec<Contour> = marching_squares(&restored_dem, level, &map_bounds, cell_size);
        for contour in contours{
            dxf_write_polyline(&mut f, &contour);
            for vertex in contour.vertices{
                dxf_write_vertex(&mut f, &vertex);
            }
            dxf_write_end_sequence(&mut f);
        }
        level += contour_interval;
    }
    */
    
    println!("Computing yellow...");
    let return_contours: Vec<Contour> = marching_squares(&drm, 1.2, &map_bounds, cell_size);
    let return_polygons: Vec<Polygon> = polygons_from_contours(return_contours, convex_hull.clone(), PolygonTrigger::Below, 403, 225.);

    for polygon in return_polygons{
        dxf_write_polygon(&mut f, &polygon);
        dxf_write_polygon_part(&mut f, &polygon.boundary);
        for vertex in polygon.boundary.vertices{
            dxf_write_vertex(&mut f, &vertex);
        }
        for hole in polygon.holes{
            dxf_write_polygon_part(&mut f, &hole);
            for hole_vertex in hole.vertices{
                dxf_write_vertex(&mut f, &hole_vertex);
            }
        }
        dxf_write_end_sequence(&mut f);
    }
    /*
    println!("Computing Intensity...");
    let intensity_contours: Vec<Contour> = marching_squares(&dim, mean_intens, &map_bounds, cell_size);
    let intensity_polygons: Vec<Polygon> = polygons_from_contours(intensity_contours, convex_hull.clone(), PolygonTrigger::Above, 214, 225.);

    for polygon in intensity_polygons{
        dxf_write_polygon(&mut f, &polygon);
        dxf_write_polygon_part(&mut f, &polygon.boundary);
        for vertex in polygon.boundary.vertices{
            dxf_write_vertex(&mut f, &vertex);
        }
        for hole in polygon.holes{
            dxf_write_polygon_part(&mut f, &hole);
            for hole_vertex in hole.vertices{
                dxf_write_vertex(&mut f, &hole_vertex);
            }
        }
        dxf_write_end_sequence(&mut f);
    }
    */
    dxf_write_end_file(&mut f);

    if args.write_tiff{
        println!("Writing gridded Las-fields to Tiff files...");
        write_gtiff(FieldType::ReturnNumber, &output_directory, &file_stem, drm, width, height, cell_size, &map_bounds);
        write_gtiff(FieldType::Intensity, &output_directory, &file_stem, dim, width, height, cell_size, &map_bounds);
        write_gtiff(FieldType::Elevation, &output_directory, &file_stem, dem, width, height, cell_size, &map_bounds);
    }
}