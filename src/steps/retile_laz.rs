use crate::geometry::{Point2D, Rectangle};

use las::{raw, Builder, Point, Reader, Writer};
use std::{fs, path::PathBuf};

pub fn retile_laz(
    neighbour_map: &[Option<usize>; 9],
    paths: &Vec<PathBuf>,
    tile_size: f64,
    margin: f64,
) -> Vec<PathBuf> {
    if paths.len() == 1 {
        single_file(&(paths[0]), tile_size, margin)
    } else {
        multiple_files(neighbour_map, paths, tile_size, margin)
    }
}

fn single_file(input: &PathBuf, tile_size: f64, margin: f64) -> Vec<PathBuf> {
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

    let (bb, num_x_tiles, num_y_tiles) = retile_bounds(&bounds, tile_size, margin);

    let mut point_buckets: Vec<Vec<Point>> = vec![
        Vec::with_capacity(
            header.number_of_point_records as usize / (num_x_tiles * num_y_tiles)
        );
        num_x_tiles * num_y_tiles
    ];

    for point in las_reader.points().map(|p| p.unwrap()) {
        for (i, b) in bb.iter().enumerate() {
            if b.contains(&point) {
                point_buckets[i].push(point.clone());
            }
        }
    }

    let paths = write_tiles_to_file(
        tiled_file,
        point_buckets,
        bb,
        num_x_tiles,
        num_y_tiles,
        &header,
    );
    paths
}

fn multiple_files(
    neighbour_map: &[Option<usize>; 9],
    paths: &Vec<PathBuf>,
    tile_size: f64,
    margin: f64,
) -> Vec<PathBuf> {
    // read the laz to be re-tiled, must be readable by now
    let ci = neighbour_map[0].unwrap();
    let mut las_reader = Reader::from_path(&paths[ci]).unwrap();

    let header = las_reader.header().clone().into_raw().unwrap();
    let mut bounds = Rectangle {
        min: Point2D {
            x: header.min_x,
            y: header.min_y,
        },
        max: Point2D {
            x: header.max_x,
            y: header.max_y,
        },
    };

    let mut push_bounds = Rectangle::default();
    for (i, v) in neighbour_map.iter().enumerate() {
        match v {
            None => continue,
            Some(_) => match i {
                0 => continue,
                1 => {
                    push_bounds.min.x = -margin;
                    push_bounds.max.y = margin;
                }
                2 => push_bounds.max.y = margin,
                3 => {
                    push_bounds.max.x = margin;
                    push_bounds.max.y = margin;
                }
                4 => push_bounds.max.x = margin,
                5 => {
                    push_bounds.max.x = margin;
                    push_bounds.min.y = -margin;
                }
                6 => push_bounds.min.y = -margin,
                7 => {
                    push_bounds.min.x = -margin;
                    push_bounds.min.y = -margin;
                }
                8 => push_bounds.min.x = -margin,
                _ => panic!("logic fail in laz neighbour calculation"),
            },
        }
    }
    bounds = bounds + push_bounds;

    let (bb, num_x_tiles, num_y_tiles) = retile_bounds(&bounds, tile_size, margin);

    let mut point_buckets: Vec<Vec<Point>> = vec![
        Vec::with_capacity(
            header.number_of_point_records as usize / (num_x_tiles * num_y_tiles)
        );
        num_x_tiles * num_y_tiles
    ];

    // read points from main file into buckets
    for point in las_reader.points().map(|p| p.unwrap()) {
        for (i, b) in bb.iter().enumerate() {
            if b.contains(&point) {
                point_buckets[i].push(point.clone());
            }
        }
    }
    drop(las_reader);

    // read points from neighbour files into buckets
    for w_ni in neighbour_map.iter().skip(1) {
        match w_ni {
            None => continue,
            Some(ni) => {
                let mut las_reader = Reader::from_path(&paths[*ni]).unwrap();

                for point in las_reader.points().map(|p| p.unwrap()) {
                    for (i, b) in bb.iter().enumerate() {
                        if b.contains(&point) {
                            point_buckets[i].push(point.clone());
                        }
                    }
                }
            }
        }
    }

    let ct = neighbour_map[0].unwrap();
    let tiled_file = paths[ct].with_extension(""); // new PathBuf wo the extension from input path

    let tile_paths = write_tiles_to_file(
        tiled_file,
        point_buckets,
        bb,
        num_x_tiles,
        num_y_tiles,
        &header,
    );
    tile_paths
}

fn write_tiles_to_file(
    mut tile_path: PathBuf,
    point_buckets: Vec<Vec<Point>>,
    bb: Vec<Rectangle>,
    num_x_tiles: usize,
    num_y_tiles: usize,
    header: &raw::Header,
) -> Vec<PathBuf> {
    let mut paths = Vec::with_capacity(num_x_tiles * num_y_tiles);
    for yi in 0..num_y_tiles {
        for xi in 0..num_x_tiles {
            tile_path.set_file_name(format!("{}_{}.laz", xi, yi));

            let points = &point_buckets[yi * num_x_tiles + xi];
            if points.len() < 10 {
                continue;
            }
            paths.push(tile_path.clone());

            let new_bounds = &bb[yi * num_x_tiles + xi];

            let mut new_header = header.clone();
            new_header.max_x = new_bounds.max.x;
            new_header.max_y = new_bounds.max.y;
            new_header.min_x = new_bounds.min.x;
            new_header.min_y = new_bounds.min.y;

            new_header.version.minor = 4;
            new_header.number_of_point_records = points.len() as u32;

            let builder = Builder::new(new_header).unwrap();

            let mut las_writer =
                Writer::from_path(tile_path.clone(), builder.into_header().unwrap())
                    .expect("Could not tile las file");

            points.iter().for_each(|p| {
                las_writer
                    .write_point(p.clone())
                    .expect("Could not write point to laz")
            });
        }
    }
    paths
}

fn retile_bounds(
    bounds: &Rectangle,
    tile_size: f64,
    margin: f64,
) -> (Vec<Rectangle>, usize, usize) {
    let unique_tile_size = tile_size - 2. * margin;

    let x_range = bounds.max.x - bounds.min.x;
    let y_range = bounds.max.y - bounds.min.y;

    let num_x_tiles = (x_range / unique_tile_size).ceil() as usize;
    let num_y_tiles = (y_range / unique_tile_size).ceil() as usize;

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
    (bb, num_x_tiles, num_y_tiles)
}
