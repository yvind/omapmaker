use crate::{
    geometry::{Point2D, Rectangle},
    TILE_SIZE_USIZE,
};

use las::{point::Classification, raw, Builder, Point, Reader, Writer};
use std::{
    fs,
    path::PathBuf,
    sync::{Arc, Mutex},
    thread,
};

use std::time::Instant;

use crate::{NEIGHBOUR_MARGIN, NEIGHBOUR_MARGIN_USIZE, TILE_SIZE};

pub fn retile_laz(
    num_threads: usize,
    neighbour_map: &[Option<usize>; 9],
    paths: &[PathBuf],
) -> (Vec<PathBuf>, Vec<Rectangle>) {
    assert!(paths.len() > 0);
    if paths.len() == 1 {
        single_file(num_threads, &(paths[0]))
    } else {
        multiple_files(num_threads, neighbour_map, paths)
    }
}

fn single_file(num_threads: usize, input: &PathBuf) -> (Vec<PathBuf>, Vec<Rectangle>) {
    let now = Instant::now();

    let mut las_reader = Reader::from_path(input).unwrap_or_else(|_| {
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

    let tiled_file = input.with_extension(""); // new PathBuf wo the extension from input path

    fs::create_dir_all(&tiled_file)
        .unwrap_or_else(|_| panic!("Could not create tile folder for {:?}", tiled_file));

    let (bb, cb, num_x_tiles, num_y_tiles) = retile_bounds(&bounds, &Rectangle::default());

    println!(
        "Bounds retiled, time including opening file etc: {:?}",
        now.elapsed()
    );
    let now = Instant::now();

    let mut point_buckets: Vec<Vec<Point>> = vec![
        Vec::with_capacity(
            header.number_of_point_records as usize / (num_x_tiles * num_y_tiles)
        );
        num_x_tiles * num_y_tiles
    ];

    // possible area for multithreading, only worth using rayon's into_par_iter when more than approx 1/2 million points
    for point in las_reader.points().map(|p| p.unwrap()) {
        for (i, b) in bb.iter().enumerate() {
            if b.contains(&point) {
                point_buckets[i].push(point.clone());
            }
        }
    }

    println!("points divided into tiles, time: {:?}", now.elapsed());
    let now = Instant::now();

    let p = write_tiles_to_file(
        num_threads,
        tiled_file,
        point_buckets,
        bb,
        num_x_tiles,
        num_y_tiles,
        header,
    );
    println!("written, time: {:?}", now.elapsed());
    (p, cb)
}

fn multiple_files(
    num_threads: usize,
    neighbour_map: &[Option<usize>; 9],
    paths: &[PathBuf],
) -> (Vec<PathBuf>, Vec<Rectangle>) {
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
                    push_bounds.min.x = -NEIGHBOUR_MARGIN;
                    push_bounds.max.y = NEIGHBOUR_MARGIN;
                }
                2 => push_bounds.max.y = NEIGHBOUR_MARGIN,
                3 => {
                    push_bounds.max.x = NEIGHBOUR_MARGIN;
                    push_bounds.max.y = NEIGHBOUR_MARGIN;
                }
                4 => push_bounds.max.x = NEIGHBOUR_MARGIN,
                5 => {
                    push_bounds.max.x = NEIGHBOUR_MARGIN;
                    push_bounds.min.y = -NEIGHBOUR_MARGIN;
                }
                6 => push_bounds.min.y = -NEIGHBOUR_MARGIN,
                7 => {
                    push_bounds.min.x = -NEIGHBOUR_MARGIN;
                    push_bounds.min.y = -NEIGHBOUR_MARGIN;
                }
                8 => push_bounds.min.x = -NEIGHBOUR_MARGIN,
                _ => panic!("logic fail in laz neighbour calculation"),
            },
        }
    }
    bounds = &bounds + &push_bounds;

    let (bb, cb, num_x_tiles, num_y_tiles) = retile_bounds(&bounds, &push_bounds);

    let mut point_buckets: Vec<Vec<Point>> = vec![
        Vec::with_capacity(
            header.number_of_point_records as usize / (num_x_tiles * num_y_tiles)
        );
        num_x_tiles * num_y_tiles
    ];

    // possible area for multithreading
    // read points from main file into buckets
    for point in las_reader.points().filter_map(Result::ok) {
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

                for point in las_reader.points().filter_map(Result::ok) {
                    // only edge boxes should be considered
                    for (i, b) in bb.iter().enumerate() {
                        if b.contains(&point) {
                            point_buckets[i].push(point.clone());
                        }
                    }
                }
            }
        }
    }

    let tiled_file = paths[ci].with_extension(""); // new PathBuf wo the extension from input path

    fs::create_dir_all(&tiled_file)
        .unwrap_or_else(|_| panic!("Could not create tile folder for {:?}", tiled_file));

    let p = write_tiles_to_file(
        num_threads,
        tiled_file,
        point_buckets,
        bb,
        num_x_tiles,
        num_y_tiles,
        header,
    );

    (p, cb)
}

fn write_tiles_to_file(
    num_threads: usize,
    mut tile_path: PathBuf,
    point_buckets: Vec<Vec<Point>>,
    bb: Vec<Rectangle>,
    num_x_tiles: usize,
    num_y_tiles: usize,
    header: raw::Header,
) -> Vec<PathBuf> {
    let paths = Arc::new(Mutex::new(Vec::with_capacity(num_x_tiles * num_y_tiles)));

    tile_path.push("temp.txt"); // just beacause PathBuf::set_file_name() otherwise removes the dir name

    let point_buckets = Arc::new(point_buckets);
    let bb = Arc::new(bb);

    let mut thread_handles = Vec::with_capacity(num_threads);
    for ti in 0..num_threads {
        let mut tile_path = tile_path.clone();
        let point_buckets = point_buckets.clone();
        let bb = bb.clone();
        let header = header.clone();
        let paths = paths.clone();

        thread_handles.push(thread::spawn(move || {
            let mut yi = ti;

            while yi < num_y_tiles {
                for xi in 0..num_x_tiles {
                    tile_path.set_file_name(format!("{}_{}.las", xi, yi));

                    let points = &point_buckets[yi * num_x_tiles + xi];
                    // skip tiles with too few ground points
                    if points
                        .iter()
                        .filter(|p| p.classification == Classification::Ground)
                        .count()
                        < TILE_SIZE_USIZE
                    {
                        continue;
                    }

                    {
                        paths.lock().unwrap().push(tile_path.clone());
                    }

                    let tile_bounds = &bb[yi * num_x_tiles + xi];

                    let mut tile_header = header.clone();
                    tile_header.max_x = tile_bounds.max.x;
                    tile_header.max_y = tile_bounds.max.y;
                    tile_header.min_x = tile_bounds.min.x;
                    tile_header.min_y = tile_bounds.min.y;

                    tile_header.version.minor = 4;
                    tile_header.number_of_point_records = points.len() as u32;

                    let builder = Builder::new(tile_header).unwrap();

                    let mut las_writer =
                        Writer::from_path(tile_path.clone(), builder.into_header().unwrap())
                            .expect("Could not write the retiled las/laz");

                    points.iter().for_each(|p| {
                        las_writer
                            .write_point(p.clone())
                            .expect("Could not write point to retiled las/laz")
                    });
                }
                yi += num_threads;
            }
        }))
    }
    for t in thread_handles {
        t.join().unwrap();
    }
    Arc::<Mutex<Vec<PathBuf>>>::into_inner(paths)
        .unwrap()
        .into_inner()
        .unwrap()
}

fn retile_bounds(
    bounds: &Rectangle,
    neighbour_bounds: &Rectangle,
) -> (Vec<Rectangle>, Vec<Rectangle>, usize, usize) {
    let x_range = bounds.max.x - bounds.min.x;
    let y_range = bounds.max.y - bounds.min.y;

    let num_x_tiles =
        ((x_range - NEIGHBOUR_MARGIN) / (TILE_SIZE - NEIGHBOUR_MARGIN)).ceil() as usize;
    let num_y_tiles =
        ((y_range - NEIGHBOUR_MARGIN) / (TILE_SIZE - NEIGHBOUR_MARGIN)).ceil() as usize;

    let first_last_margin_x =
        (-x_range + 2. * TILE_SIZE + ((num_x_tiles - 2) * TILE_SIZE_USIZE) as f64
            - ((num_x_tiles - 3) * NEIGHBOUR_MARGIN_USIZE) as f64)
            / 2.;
    let first_last_margin_y =
        (-y_range + 2. * TILE_SIZE + ((num_x_tiles - 2) * TILE_SIZE_USIZE) as f64
            - ((num_x_tiles - 3) * NEIGHBOUR_MARGIN_USIZE) as f64)
            / 2.;

    let mut bb: Vec<Rectangle> = Vec::with_capacity(num_x_tiles * num_y_tiles);
    let mut cut_bounds: Vec<Rectangle> = Vec::with_capacity(num_x_tiles * num_y_tiles);

    for yi in 0..num_y_tiles {
        for xi in 0..num_x_tiles {
            let mut tile_bounds = Rectangle::default();
            let mut inner_bounds = Rectangle::default();

            if yi == 0 {
                // no neighbour above
                tile_bounds.max.y = bounds.max.y;
                tile_bounds.min.y = tile_bounds.max.y - TILE_SIZE;

                inner_bounds.max.y = bounds.max.y - neighbour_bounds.max.y;
                inner_bounds.min.y = tile_bounds.min.y + first_last_margin_y / 2.;
            } else if yi == num_y_tiles - 1 {
                // no neigbour below
                tile_bounds.min.y = bounds.min.y;
                tile_bounds.max.y = tile_bounds.min.y + TILE_SIZE;

                inner_bounds.min.y = bounds.min.y - neighbour_bounds.min.y;
                inner_bounds.max.y = tile_bounds.max.y - first_last_margin_y / 2.;
            } else {
                tile_bounds.max.y = bounds.max.y - (TILE_SIZE_USIZE * yi) as f64
                    + first_last_margin_y
                    + (NEIGHBOUR_MARGIN_USIZE * (yi - 1)) as f64;
                tile_bounds.min.y = tile_bounds.max.y - TILE_SIZE;

                inner_bounds.max.y = tile_bounds.max.y - NEIGHBOUR_MARGIN / 2.;
                inner_bounds.max.y = tile_bounds.min.y + NEIGHBOUR_MARGIN / 2.;
            }
            if xi == 0 {
                // no neighbour to the left
                tile_bounds.min.x = bounds.min.x;
                tile_bounds.max.x = tile_bounds.min.x + TILE_SIZE;

                inner_bounds.min.x = bounds.min.x - neighbour_bounds.min.x;
                inner_bounds.max.x = tile_bounds.max.x - first_last_margin_x / 2.;
            } else if xi == num_x_tiles - 1 {
                // no neigbour to the right
                tile_bounds.max.x = bounds.max.x;
                tile_bounds.min.x = tile_bounds.max.x - TILE_SIZE;

                inner_bounds.max.x = bounds.max.x - neighbour_bounds.max.x;
                inner_bounds.min.x = tile_bounds.min.x + first_last_margin_x / 2.;
            } else {
                tile_bounds.min.x =
                    bounds.max.x + (TILE_SIZE_USIZE * xi) as f64 + first_last_margin_x
                        - (NEIGHBOUR_MARGIN_USIZE * (xi - 1)) as f64;
                tile_bounds.max.x = tile_bounds.min.x + TILE_SIZE;

                inner_bounds.min.x = tile_bounds.min.x + NEIGHBOUR_MARGIN / 2.;
                inner_bounds.max.x = tile_bounds.max.x - NEIGHBOUR_MARGIN / 2.;
            }

            bb.push(tile_bounds);
            cut_bounds.push(inner_bounds);
        }
    }
    (bb, cut_bounds, num_x_tiles, num_y_tiles)
}
