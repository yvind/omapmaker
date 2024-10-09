use crate::geometry::{Point2D, Rectangle};

use kiddo::{immutable::float::kdtree::ImmutableKdTree, SquaredEuclidean};
use las::{Bounds, Builder, Point, Reader, Writer};
use std::{
    ffi::OsString,
    fs,
    path::{Path, PathBuf},
};

pub fn prepare_laz(
    input: PathBuf,
    margin: f64,
    tile_size: f64,
) -> (Vec<Vec<usize>>, Vec<PathBuf>, Point2D, OsString) {
    let filestem = Path::new(input.file_name().unwrap())
        .file_stem()
        .unwrap()
        .to_owned();

    if input.is_dir() {
        let (a, b, c) = multiple_files(input, margin, tile_size);
        (a, b, c, filestem)
    } else if input.is_file() {
        let (a, b, c) = single_file(input, margin, tile_size);
        (a, b, c, filestem)
    } else {
        panic!("Given input is not a recognizable file or directory")
    }
}

fn single_file(
    input: PathBuf,
    margin: f64,
    tile_size: f64,
) -> (Vec<Vec<usize>>, Vec<PathBuf>, Point2D) {
    let mut las_reader = Reader::from_path(&input).unwrap_or_else(|_| {
        panic!(
            "Could not read given laz/las file with path: {}",
            input.to_string_lossy()
        )
    });

    let header = las_reader.header().clone().into_raw().unwrap();
    let bounds = Rectangle {
        min: Point2D {
            x: header.min_x,
            y: header.min_y,
        },
        max: Point2D {
            x: header.max_x,
            y: header.max_y,
        },
    };

    let mut tiled_file = input.with_extension(""); // new PathBuf wo the extension from input path

    fs::create_dir_all(&tiled_file)
        .unwrap_or_else(|_| panic!("Could not create tile folder for {:?}", tiled_file));
    tiled_file.push("temp.txt"); // just beacause PathBuf::set_file_name() otherwise removes the dir name

    let unique_tile_size = tile_size - 2. * margin;

    let x_range = bounds.max.x - bounds.min.x;
    let y_range = bounds.max.y - bounds.min.y;

    let num_x_tiles = (x_range / unique_tile_size).ceil() as usize;
    let num_y_tiles = (y_range / unique_tile_size).ceil() as usize;

    let mut point_buckets: Vec<Vec<Point>> = vec![
        Vec::with_capacity(
            header.number_of_point_records as usize / (num_x_tiles * num_y_tiles)
        );
        num_x_tiles * num_y_tiles
    ];

    let mut bb: Vec<Rectangle> = Vec::with_capacity(num_x_tiles * num_y_tiles);

    for yi in 0..num_y_tiles {
        for xi in 0..num_x_tiles {
            let mut new_bounds = Rectangle::default();

            if yi == 0 {
                // no neighbour above
                new_bounds.max.y = bounds.max.y;
                new_bounds.min.y = new_bounds.max.y - tile_size;
            } else if yi == num_y_tiles - 1 {
                // no neigbour below
                new_bounds.min.y = bounds.min.y;
                new_bounds.max.y = new_bounds.min.y + tile_size;
            } else {
                new_bounds.max.y = bounds.max.y - (tile_size - 2. * margin) * yi as f64 + margin;
                new_bounds.min.y = new_bounds.max.y - tile_size;
            }
            if xi == 0 {
                // no neighbour to the left
                new_bounds.min.x = bounds.min.x;
                new_bounds.max.x = new_bounds.min.x + tile_size;
            } else if xi == num_x_tiles - 1 {
                // no neigbour to the right
                new_bounds.max.x = bounds.max.x;
                new_bounds.min.x = new_bounds.max.x - tile_size;
            } else {
                new_bounds.min.x = bounds.min.x + (tile_size - 2. * margin) * xi as f64 - margin;
                new_bounds.max.x = new_bounds.min.x + tile_size;
            }

            bb.push(new_bounds);
        }
    }

    for point in las_reader.points().map(|p| p.unwrap()) {
        for (i, b) in bb.iter().enumerate() {
            if b.contains(&point) {
                point_buckets[i].push(point.clone());
            }
        }
    }

    for yi in 0..num_y_tiles {
        for xi in 0..num_x_tiles {
            tiled_file.set_file_name(format!("{}_{}.laz", xi, yi));

            let points = &point_buckets[yi * num_x_tiles + xi];
            if points.len() < 10 {
                continue;
            }

            let new_bounds = &bb[yi * num_x_tiles + xi];

            let mut new_header = header.clone();
            new_header.max_x = new_bounds.max_x;
            new_header.max_y = new_bounds.max_y;
            new_header.min_x = new_bounds.min_x;
            new_header.min_y = new_bounds.min_y;

            new_header.version.minor = 4;
            new_header.number_of_point_records = points.len() as u32;

            let builder = Builder::new(new_header).unwrap();

            let mut las_writer =
                Writer::from_path(tiled_file.clone(), builder.into_header().unwrap())
                    .expect("Could not tile las file");

            points.iter().for_each(|p| {
                las_writer
                    .write_point(p.clone())
                    .expect("Could not write point to laz")
            });
        }
    }

    let mut las_reader = Reader::from_path(&input).unwrap_or_else(|_| {
        panic!(
            "Could not read given laz/las file with path: {}",
            input.to_string_lossy()
        )
    });
    let bounds = las_reader.header().bounds();

    // in input folder create a dir called {lasfile_name} and inside store tiled laz files with names xi-yi.laz
    let mut tiled_file = PathBuf::from(input.file_stem().unwrap());
    fs::create_dir_all(&tiled_file)
        .unwrap_or_else(|_| panic!("Could not create tile folder for {:?}", tiled_file));

    let unique_tile_size = tile_size - 2. * margin;

    let x_range = bounds.max.x - bounds.min.x;
    let y_range = bounds.max.y - bounds.min.y;

    let num_x_tiles = (x_range / unique_tile_size).ceil() as usize;
    let num_y_tiles = (y_range / unique_tile_size).ceil() as usize;

    for yi in 0..num_y_tiles {
        for xi in 0..num_x_tiles {
            tiled_file.set_file_name(format!("{}_{}.laz", xi, yi));

            let mut new_header = las_reader.header().clone();
            let mut new_bounds = new_header.bounds();

            if yi == 0 {
                // no neighbour above
                new_bounds.max.y = bounds.max.y;
                new_bounds.min.y = new_bounds.max.y - tile_size;
            } else if yi == num_y_tiles - 1 {
                // no neigbour below
                new_bounds.min.y = bounds.min.y;
                new_bounds.max.y = new_bounds.min.y + tile_size;
            } else {
                new_bounds.max.y = bounds.max.y - (tile_size - 2. * margin) * yi as f64 - margin;
                new_bounds.min.y = new_bounds.max.y - tile_size;
            }
            if xi == 0 {
                // no neighbour to the left
                new_bounds.min.x = bounds.min.x;
                new_bounds.max.x = new_bounds.min.x + tile_size;
            } else if xi == num_x_tiles - 1 {
                // no neigbour to the right
                new_bounds.max.x = bounds.max.x;
                new_bounds.min.x = new_bounds.max.x - tile_size;
            } else {
                new_bounds.min.x = bounds.min.x + (tile_size - 2. * margin) * xi as f64 - margin;
                new_bounds.max.x = new_bounds.min.x + tile_size;
            }

            //new_header.bounds = new_bounds;
            let mut las_writer =
                Writer::from_path(tiled_file.clone(), new_header).expect("Could not tile las file");

            for point in las_reader.points().map(|p| p.unwrap()) {
                if point.x >= new_bounds.min.x
                    && point.y >= new_bounds.min.y
                    && point.x <= new_bounds.max.x
                    && point.y <= new_bounds.max.y
                {
                    las_writer
                        .write_point(point)
                        .expect("Could not write point to laz");
                }
            }
        }
    }

    (
        vec![vec![0]],
        vec![input],
        Point2D::new(
            ((bounds.min.x + bounds.max.x) / 20.).round() * 10.,
            ((bounds.min.y + bounds.max.y) / 20.).round() * 10.,
        ),
    )
}

fn multiple_files(
    input: PathBuf,
    margin: f64,
    tile_size: f64,
) -> (Vec<Vec<usize>>, Vec<PathBuf>, Point2D) {
    let mut tile_centers = Vec::new();
    let mut tile_bounds = Vec::new();
    let mut tile_names = Vec::new();

    for entry in fs::read_dir(input).expect("Could not read given laz-directory") {
        let las_path = entry
            .expect("Could not access files in the laz-directory")
            .path();

        if let Ok(las_reader) = Reader::from_path(&las_path) {
            let b = las_reader.header().bounds();
            tile_centers.push([(b.min.x + b.max.x) / 2., (b.min.y + b.max.y) / 2.]);
            tile_bounds.push(b);
            tile_names.push(las_path);
        }
    }

    let neighbours = neighbouring_tiles(&tile_centers, &tile_bounds, margin);

    let mut ref_point = Point2D::new(0., 0.);
    for c in tile_centers.iter() {
        ref_point.x += c[0];
        ref_point.y += c[1];
    }
    ref_point.x = (ref_point.x / (10. * tile_centers.len() as f64)).round() * 10.;
    ref_point.y = (ref_point.y / (10. * tile_centers.len() as f64)).round() * 10.;

    (neighbours, tile_names, ref_point)
}

fn neighbouring_tiles(
    tile_centers: &[[f64; 2]],
    tile_bounds: &[Bounds],
    margin: f64,
) -> Vec<Vec<usize>> {
    let tree: ImmutableKdTree<f64, usize, 2, 32> = ImmutableKdTree::new_from_slice(tile_centers);

    let mut tile_neighbours = vec![];
    for (i, point) in tile_centers.iter().enumerate() {
        let nn = tree.nearest_n::<SquaredEuclidean>(point, 9);
        let mut neighbours: Vec<usize> = nn.iter().map(|n| n.item).collect();

        neighbours.retain(|&e| boxes_touch(&tile_bounds[i], &tile_bounds[e], margin));

        tile_neighbours.push(neighbours);
    }
    tile_neighbours
}

fn boxes_touch(box1: &Bounds, box2: &Bounds, margin: f64) -> bool {
    !(box1.max.x < box2.min.x - margin
        || box1.min.x > box2.max.x + margin
        || box1.max.y < box2.min.y - margin
        || box1.min.y > box2.max.y + margin)
}
