use geo::{LineString, Polygon};
use proj_core::Transform;

pub fn from_walkers_map_coords(epsg: Option<u16>, line: LineString) -> Option<Polygon> {
    if line.0.is_empty() {
        return None;
    }
    if epsg.is_none() {
        return Some(Polygon::new(line, vec![]));
    }
    let epsg = epsg.unwrap();

    let transform = Transform::from_epsg(4326, epsg as u32).unwrap();

    let transformed_line = transform.convert_geometry(line).unwrap();

    Some(Polygon::new(transformed_line, vec![]))
}
