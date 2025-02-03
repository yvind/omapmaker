#![allow(clippy::too_many_arguments)]

use crate::{geometry::PointTrait, MIN_NEIGHBOUR_MARGIN, TILE_SIZE, TILE_SIZE_USIZE};

use std::{
    fs,
    path::PathBuf,
    sync::{Arc, Mutex},
    thread,
};

use geo::{Contains, Coord, Rect};
use las::{point::Classification, raw, Builder, Point, Reader, Writer};

pub fn retile_laz(
    num_threads: usize,
    neighbour_map: &[Option<usize>; 9],
    paths: Arc<Vec<PathBuf>>,
) -> (Vec<PathBuf>, Vec<Rect>) {
    assert!(!paths.is_empty());

    // read the laz to be re-tiled, must be readable by now
    let ci = neighbour_map[0].unwrap();
    let header;
    {
        let las_reader = Reader::from_path(&paths[ci]).unwrap();
        header = las_reader.header().clone().into_raw().unwrap();
    }
    let bounds = Rect::new(
        Coord {
            x: header.min_x,
            y: header.min_y,
        },
        Coord {
            x: header.max_x,
            y: header.max_y,
        },
    );

    let mut min = Coord::zero();
    let mut max = Coord::zero();
    for (i, v) in neighbour_map.iter().enumerate() {
        match v {
            None => continue,
            Some(_) => match i {
                0 => continue,
                1 => {
                    min.x = -MIN_NEIGHBOUR_MARGIN;
                    max.y = MIN_NEIGHBOUR_MARGIN;
                }
                2 => max.y = MIN_NEIGHBOUR_MARGIN,
                3 => {
                    max.x = MIN_NEIGHBOUR_MARGIN;
                    max.y = MIN_NEIGHBOUR_MARGIN;
                }
                4 => max.x = MIN_NEIGHBOUR_MARGIN,
                5 => {
                    max.x = MIN_NEIGHBOUR_MARGIN;
                    min.y = -MIN_NEIGHBOUR_MARGIN;
                }
                6 => min.y = -MIN_NEIGHBOUR_MARGIN,
                7 => {
                    min.x = -MIN_NEIGHBOUR_MARGIN;
                    min.y = -MIN_NEIGHBOUR_MARGIN;
                }
                8 => min.x = -MIN_NEIGHBOUR_MARGIN,
                _ => panic!("logic fail in laz neighbour calculation"),
            },
        }
    }
    let push_bounds = Rect::new(min, max);
    let bounds = Rect::new(
        bounds.min() + push_bounds.min(),
        bounds.max() + push_bounds.max(),
    );
    let (bb, cb, num_x_tiles, num_y_tiles) = retile_bounds(&bounds, &push_bounds);
    {
        //pb.lock().unwrap().inc(1);
    }

    let point_buckets = Arc::new(Mutex::new(vec![
        Vec::with_capacity(
            header.number_of_point_records as usize / (num_x_tiles * num_y_tiles)
        );
        num_x_tiles * num_y_tiles
    ]));

    let bb = Arc::new(bb);
    let mut thread_handles = Vec::with_capacity(num_threads);
    let tni = Arc::new(Mutex::new(num_threads));
    for ti in 0..num_threads {
        let tni = tni.clone();
        let point_buckets = point_buckets.clone();
        let bb = bb.clone();
        let neighbour_map = *neighbour_map;
        let paths = paths.clone();
        thread_handles.push(thread::spawn(move || {
            let mut neighbour_index = ti;
            while neighbour_index < neighbour_map.len() {
                if let Some(pi) = neighbour_map[neighbour_index] {
                    let mut las_reader = Reader::from_path(&paths[pi]).unwrap();

                    let mut thread_buckets = vec![
                        Vec::with_capacity(
                            header.number_of_point_records as usize / (num_x_tiles * num_y_tiles)
                        );
                        num_x_tiles * num_y_tiles
                    ];

                    for point in las_reader.points().filter_map(Result::ok) {
                        for (i, b) in bb.iter().enumerate() {
                            if b.contains(&point.coords()) {
                                thread_buckets[i].push(point.clone());
                            }
                        }
                    }
                    let mut buckets = point_buckets.lock().unwrap();
                    for (i, t_bucket) in thread_buckets.into_iter().enumerate() {
                        if !t_bucket.is_empty() {
                            buckets[i].extend(t_bucket);
                        }
                    }
                }
                {
                    // aquire mutex and get next free index
                    let mut tni_lock = tni.lock().unwrap();
                    neighbour_index = *tni_lock;
                    *tni_lock += 1;
                } // release mutex
            }
            //pb.lock().unwrap().inc(2);
        }));
    }
    for t in thread_handles {
        t.join().unwrap();
    }
    // remove the mutex from point buckets
    let point_buckets = Arc::new(
        Arc::<Mutex<Vec<Vec<Point>>>>::into_inner(point_buckets)
            .unwrap()
            .into_inner()
            .unwrap(),
    );

    let tiled_file = paths[ci].with_extension(""); // new PathBuf wo the extension from input path

    fs::create_dir_all(&tiled_file)
        .unwrap_or_else(|_| panic!("Could not create tile folder for {:?}", tiled_file));

    write_tiles_to_file(
        num_threads,
        tiled_file,
        point_buckets,
        bb,
        cb,
        num_x_tiles,
        num_y_tiles,
        header,
    )
}

fn write_tiles_to_file(
    num_threads: usize,
    mut tile_path: PathBuf,
    point_buckets: Arc<Vec<Vec<Point>>>,
    bb: Arc<Vec<Rect>>,
    mut cb: Vec<Rect>,
    num_x_tiles: usize,
    num_y_tiles: usize,
    header: raw::Header,
) -> (Vec<PathBuf>, Vec<Rect>) {
    let paths = Arc::new(Mutex::new(vec![
        PathBuf::default();
        num_x_tiles * num_y_tiles
    ]));

    tile_path.push("temp.txt"); // just beacause PathBuf::set_file_name() otherwise removes the dir name

    let remove_index = Arc::new(Mutex::new(Vec::with_capacity(cb.len())));

    let mut thread_handles = Vec::with_capacity(num_threads);
    for ti in 0..num_threads {
        let mut tile_path = tile_path.clone();
        let point_buckets = point_buckets.clone();
        let bb = bb.clone();
        let header = header.clone();
        let paths = paths.clone();
        let remove_index = remove_index.clone();

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
                        remove_index.lock().unwrap().push(yi * num_x_tiles + xi);
                        continue;
                    }

                    {
                        paths.lock().unwrap()[yi * num_x_tiles + xi].push(tile_path.clone());
                    }
                    let tile_bounds = &bb[yi * num_x_tiles + xi];

                    let mut tile_header = header.clone();
                    tile_header.max_x = tile_bounds.max().x;
                    tile_header.max_y = tile_bounds.max().y;
                    tile_header.min_x = tile_bounds.min().x;
                    tile_header.min_y = tile_bounds.min().y;

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
            //pb.lock().unwrap().inc(1);
        }))
    }
    for t in thread_handles {
        t.join().unwrap();
    }

    let mut paths = Arc::<Mutex<Vec<PathBuf>>>::into_inner(paths)
        .unwrap()
        .into_inner()
        .unwrap();

    let mut remove_index = Arc::<Mutex<Vec<usize>>>::into_inner(remove_index)
        .unwrap()
        .into_inner()
        .unwrap();

    remove_index.sort_unstable_by(|a, b| b.cmp(a));
    for i in remove_index {
        cb.remove(i);
        paths.remove(i);
    }

    assert_eq!(paths.len(), cb.len());
    (paths, cb)
}

fn retile_bounds(
    bounds: &Rect,
    neighbour_file_margin: &Rect,
) -> (Vec<Rect>, Vec<Rect>, usize, usize) {
    let x_range = bounds.max().x - bounds.min().x;
    let y_range = bounds.max().y - bounds.min().y;

    let num_x_tiles = ((x_range - MIN_NEIGHBOUR_MARGIN) / (TILE_SIZE - MIN_NEIGHBOUR_MARGIN))
        .ceil()
        .max(2.0) as usize;
    let num_y_tiles = ((y_range - MIN_NEIGHBOUR_MARGIN) / (TILE_SIZE - MIN_NEIGHBOUR_MARGIN))
        .ceil()
        .max(2.0) as usize;

    let neighbour_margin_x =
        ((num_x_tiles * TILE_SIZE_USIZE) as f64 - x_range) / (num_x_tiles - 1) as f64;
    let neighbour_margin_y =
        ((num_y_tiles * TILE_SIZE_USIZE) as f64 - y_range) / (num_y_tiles - 1) as f64;

    let mut bb: Vec<Rect> = Vec::with_capacity(num_x_tiles * num_y_tiles);
    let mut cut_bounds: Vec<Rect> = Vec::with_capacity(num_x_tiles * num_y_tiles);

    for yi in 0..num_y_tiles {
        for xi in 0..num_x_tiles {
            let mut tile_min = Coord::zero();
            let mut tile_max = Coord::zero();

            let mut inner_min = Coord::zero();
            let mut inner_max = Coord::zero();

            if yi == 0 {
                // no neighbour above
                tile_max.y = bounds.max().y;
                tile_min.y = tile_max.y - TILE_SIZE;

                inner_max.y = bounds.max().y - neighbour_file_margin.max().y;
                inner_min.y = tile_min.y + neighbour_margin_y / 2.;
            } else if yi == num_y_tiles - 1 {
                // no neigbour below
                tile_min.y = bounds.min().y;
                tile_max.y = tile_min.y + TILE_SIZE;

                inner_min.y = bounds.min().y - neighbour_file_margin.min().y;
                inner_max.y = tile_max.y - neighbour_margin_y / 2.;
            } else {
                tile_max.y = bounds.max().y - (TILE_SIZE - neighbour_margin_y) * yi as f64;
                tile_min.y = tile_max.y - TILE_SIZE;

                inner_max.y = tile_max.y - neighbour_margin_y / 2.;
                inner_min.y = tile_min.y + neighbour_margin_y / 2.;
            }
            if xi == 0 {
                // no neighbour to the left
                tile_min.x = bounds.min().x;
                tile_max.x = tile_min.x + TILE_SIZE;

                inner_min.x = bounds.min().x - neighbour_file_margin.min().x;
                inner_max.x = tile_max.x - neighbour_margin_x / 2.;
            } else if xi == num_x_tiles - 1 {
                // no neigbour to the right
                tile_max.x = bounds.max().x;
                tile_min.x = tile_max.x - TILE_SIZE;

                inner_max.x = bounds.max().x - neighbour_file_margin.max().x;
                inner_min.x = tile_min.x + neighbour_margin_x / 2.;
            } else {
                tile_min.x = bounds.min().x + (TILE_SIZE - neighbour_margin_x) * xi as f64;
                tile_max.x = tile_min.x + TILE_SIZE;

                inner_min.x = tile_min.x + neighbour_margin_x / 2.;
                inner_max.x = tile_max.x - neighbour_margin_x / 2.;
            }

            bb.push(Rect::new(tile_min, tile_max));
            cut_bounds.push(Rect::new(inner_min, inner_max));
        }
    }
    (bb, cut_bounds, num_x_tiles, num_y_tiles)
}
