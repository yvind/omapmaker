#![allow(clippy::too_many_arguments)]

use crate::{MIN_NEIGHBOUR_MARGIN, TILE_SIZE, TILE_SIZE_USIZE};

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
    let header = {
        let las_reader = Reader::from_path(&paths[ci]).unwrap();
        las_reader.header().clone().into_raw().unwrap()
    };
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
                            if b.contains(&Coord {
                                x: point.x,
                                y: point.y,
                            }) {
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
    (Vec::new(), Vec::new())
}

pub fn retile_bounds(
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
