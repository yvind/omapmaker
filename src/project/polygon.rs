use geo::{Coord, LineString, Polygon};
use proj4rs::{transform::transform, Proj};

pub fn from_walkers_map_coords(epsg: Option<u16>, line: LineString) -> Option<Polygon> {
    if line.0.is_empty() {
        return None;
    }
    if epsg.is_none() {
        return Some(Polygon::new(line, vec![]));
    }
    let epsg = epsg.unwrap();

    let global_proj = Proj::from_epsg_code(4326).unwrap();
    let local_proj = Proj::from_epsg_code(epsg).unwrap();

    // proj4rs uses radians, but walkers uses degrees. Conversion needed
    let mut points: Vec<(f64, f64)> = line
        .0
        .into_iter()
        .map(|c| (c.x.to_radians(), c.y.to_radians()))
        .collect();

    transform(&global_proj, &local_proj, points.as_mut_slice()).unwrap();

    let line = LineString::new(
        points
            .into_iter()
            .map(|t| Coord { x: t.0, y: t.1 })
            .collect(),
    );

    Some(Polygon::new(line, vec![]))
}
