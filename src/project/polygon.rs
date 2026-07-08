use proj_core::{CrsDef, Transform};

pub fn from_walkers_map_coords(
    crs: Option<CrsDef>,
    line: geo::LineString,
) -> crate::Result<Option<geo::Polygon>> {
    if line.0.is_empty() {
        return Ok(None);
    }
    let Some(crs) = crs else {
        return Ok(Some(geo::Polygon::new(line, vec![])));
    };

    let transform = Transform::from_epsg(4326, crs.epsg())?;

    let transformed_line = transform.convert_geometry(line)?;

    Ok(Some(geo::Polygon::new(transformed_line, vec![])))
}
