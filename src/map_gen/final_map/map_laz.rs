#![allow(clippy::type_complexity)]

use geo::{Coord, Intersects, Polygon, Rect};
use las::Reader;

use std::path::PathBuf;

use crate::geometry::MapRect;
use crate::neighbors::Neighborhood;
use crate::{Error, Result};

pub fn map_laz(
    paths: &Vec<PathBuf>,
    polygon_filter: &Option<Polygon>,
) -> Result<(Vec<PathBuf>, Vec<Neighborhood>, Vec<Rect>, Coord, f64)> {
    let mut tile_centers = Vec::with_capacity(paths.len());
    let mut las_paths = Vec::with_capacity(paths.len());
    let mut avg_elevation = 0.;
    let mut tile_bounds = Vec::with_capacity(paths.len());

    for path in paths {
        if let Ok(las_reader) = Reader::from_path(path) {
            let b = las_reader.header().bounds();

            if let Some(polygon) = polygon_filter {
                let rect = Rect::from_bounds(b);
                if !polygon.intersects(&rect) {
                    continue;
                }
            }

            las_paths.push(path.clone());
            avg_elevation += (b.min.z + b.max.z) / 2.;
            tile_centers.push([(b.min.x + b.max.x) / 2., (b.min.y + b.max.y) / 2.]);
            tile_bounds.push(Rect::from_bounds(b));
        }
    }

    if las_paths.is_empty() {
        return Err(Error::MapAreaDistinctFromLidarArea);
    }

    if tile_centers.len() == 1 {
        let mut center_point = tile_centers.swap_remove(0); // round ref_point to nearest 10m
        center_point[0] = (center_point[0] / 10.).round() * 10.;
        center_point[1] = (center_point[1] / 10.).round() * 10.;
        avg_elevation = (avg_elevation / 10.).round() * 10.;

        return Ok((
            las_paths,
            vec![Default::default()],
            tile_bounds,
            Coord::from(center_point),
            avg_elevation,
        ));
    }

    let neighbors = Neighborhood::neighboring_tiles(&tile_centers, &tile_bounds);

    let mut ref_point: Coord<f64> = Coord::default();
    tile_centers.iter().for_each(|tc| {
        ref_point.x += tc[0];
        ref_point.y += tc[1]
    });
    ref_point.x = (ref_point.x / (10 * tile_centers.len()) as f64).round() * 10.;
    ref_point.y = (ref_point.y / (10 * tile_centers.len()) as f64).round() * 10.;
    avg_elevation = (avg_elevation / (10 * tile_centers.len()) as f64).round() * 10.;

    Ok((las_paths, neighbors, tile_bounds, ref_point, avg_elevation))
}
