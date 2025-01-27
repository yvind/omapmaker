use kiddo::{immutable::float::kdtree::ImmutableKdTree, SquaredEuclidean};
use las::Reader;

use std::{fs, path::PathBuf};

use crate::geometry::MapRect;

use geo::{Coord, Rect};

pub fn map_laz(input: PathBuf) -> (Vec<[Option<usize>; 9]>, Vec<PathBuf>, Coord) {
    if input.is_file() {
        single_file(input)
    } else {
        // check if dir contains more than 1 las/laz file
        let mut paths = lidar_in_directory(input);
        if paths.is_empty() {
            panic!("No laz/las files were found in the given directory");
        } else if paths.len() == 1 {
            single_file(paths.swap_remove(0))
        } else {
            multiple_files(paths)
        }
    }
}

fn lidar_in_directory(input: PathBuf) -> Vec<PathBuf> {
    let extensions = ["las", "laz"];

    let paths = fs::read_dir(input)
        .expect("Could not read input directory")
        .filter_map(Result::ok)
        .filter(|entry| {
            entry.path().is_file()
                && (entry.path().extension().and_then(|e| e.to_str()) == Some(extensions[0])
                    || entry.path().extension().and_then(|e| e.to_str()) == Some(extensions[1]))
        })
        .map(|lf| lf.path())
        .collect();
    paths
}

fn single_file(input: PathBuf) -> (Vec<[Option<usize>; 9]>, Vec<PathBuf>, Coord) {
    let las_reader = Reader::from_path(&input).unwrap_or_else(|_| {
        panic!(
            "Could not read given laz/las file with path: {}",
            &input.to_string_lossy()
        )
    });
    let bounds = las_reader.header().bounds();

    let ref_point = Coord {
        x: ((bounds.max.x + bounds.min.x) / 20.).round() * 10.,
        y: ((bounds.max.y + bounds.min.y) / 20.).round() * 10.,
    };

    (
        vec![[Some(0), None, None, None, None, None, None, None, None]],
        vec![input],
        ref_point,
    )
}

fn multiple_files(paths: Vec<PathBuf>) -> (Vec<[Option<usize>; 9]>, Vec<PathBuf>, Coord) {
    let mut tile_centers = Vec::with_capacity(paths.len());
    let mut tile_bounds = Vec::with_capacity(paths.len());
    let mut tile_names = Vec::with_capacity(paths.len());

    for las_path in paths {
        if let Ok(las_reader) = Reader::from_path(&las_path) {
            let b = las_reader.header().bounds();
            tile_centers.push([(b.min.x + b.max.x) / 2., (b.min.y + b.max.y) / 2.]);
            tile_bounds.push(Rect::from_bounds(b));
            tile_names.push(las_path);
        }
    }

    if tile_names.is_empty() {
        panic!("Unable to read the las/laz-files found in the input directory");
    } else if tile_names.len() == 1 {
        let mut center_point = tile_centers.swap_remove(0); // round ref_point to nearest 10m
        center_point[0] = (center_point[0] / 10.).round() * 10.;
        center_point[1] = (center_point[1] / 10.).round() * 10.;

        return (
            vec![[Some(0), None, None, None, None, None, None, None, None]],
            tile_names,
            Coord::from(center_point),
        );
    }

    let neighbours = neighbouring_tiles(&tile_centers, &tile_bounds);

    let mut ref_point: Coord<f64> = Coord::default();
    tile_centers.iter().for_each(|tc| {
        ref_point.x += tc[0];
        ref_point.y += tc[1]
    });
    ref_point.x = (ref_point.x / (10 * tile_centers.len()) as f64).round() * 10.;
    ref_point.y = (ref_point.y / (10 * tile_centers.len()) as f64).round() * 10.;

    (neighbours, tile_names, ref_point)
}

fn neighbouring_tiles(tile_centers: &[[f64; 2]], tile_bounds: &[Rect]) -> Vec<[Option<usize>; 9]> {
    let tree: ImmutableKdTree<f64, usize, 2, 32> = ImmutableKdTree::new_from_slice(tile_centers);

    let mut avg_tile_size = 0.;
    tile_bounds
        .iter()
        .for_each(|r| avg_tile_size += r.max().x - r.min().x + r.max().y - r.min().y);
    avg_tile_size /= (2 * tile_bounds.len()) as f64;

    let margin = 0.1 * avg_tile_size;

    let mut tile_neighbours = Vec::with_capacity(tile_centers.len());
    for (i, point) in tile_centers.iter().enumerate() {
        let bounds = &tile_bounds[i];

        let nn = tree.nearest_n::<SquaredEuclidean>(point, 9);
        let mut neighbours_index: Vec<usize> = nn.iter().map(|n| n.item).collect();

        neighbours_index.retain(|&e| tile_bounds[i].touch_margin(&tile_bounds[e], margin));

        let mut orderd_neighbours = [Some(i), None, None, None, None, None, None, None, None];
        for ni in neighbours_index.iter().skip(1) {
            if let Some(j) = get_neighbour_side(bounds, tile_centers[*ni]) {
                orderd_neighbours[j] = Some(*ni)
            }
        }

        tile_neighbours.push(orderd_neighbours);
    }
    tile_neighbours
}

fn get_neighbour_side(bounds: &Rect, tile_center: [f64; 2]) -> Option<usize> {
    if tile_center[0] < bounds.min().x && tile_center[1] > bounds.max().y {
        return Some(1);
    }
    if tile_center[0] > bounds.max().x && tile_center[1] > bounds.max().y {
        return Some(3);
    }
    if tile_center[0] > bounds.max().x && tile_center[1] < bounds.min().y {
        return Some(5);
    }
    if tile_center[0] < bounds.min().x && tile_center[1] < bounds.min().y {
        return Some(7);
    }
    if tile_center[1] > bounds.max().y {
        return Some(2);
    }
    if tile_center[0] > bounds.max().x {
        return Some(4);
    }
    if tile_center[1] < bounds.min().y {
        return Some(6);
    }
    if tile_center[0] < bounds.min().x {
        return Some(8);
    }
    None
}
