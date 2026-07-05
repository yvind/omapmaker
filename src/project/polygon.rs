use geo::{LineString, Polygon};
use proj_core::{CrsDef, Transform};

pub fn from_walkers_map_coords(crs: Option<CrsDef>, line: LineString) -> Option<Polygon> {
    if line.0.is_empty() {
        return None;
    }
    let Some(crs) = crs else {
        return Some(Polygon::new(line, vec![]));
    };

    let transform = Transform::from_epsg(4326, crs.epsg()).unwrap();

    let transformed_line = transform.convert_geometry(line).unwrap();

    Some(Polygon::new(transformed_line, vec![]))
}
