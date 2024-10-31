use super::{LineString, MapLineString, MultiPolygon};
pub use geo::MultiLineString;

pub trait MapMultiLineString {
    fn from_polygons(polys: MultiPolygon) -> MultiLineString;
    fn fix_ends_to_line(&mut self, hull: &LineString, epsilon: f64);
}

impl MapMultiLineString for MultiLineString {
    fn from_polygons(polys: MultiPolygon) -> MultiLineString {
        let mut result = Vec::with_capacity(polys.0.len());

        for poly in polys.into_iter() {
            let (ext, inn) = poly.into_inner();
            result.push(ext);
            result.extend(inn);
        }
        MultiLineString::new(result)
    }

    fn fix_ends_to_line(&mut self, hull: &LineString, epsilon: f64) {
        for c in self.iter_mut() {
            c.fix_ends_to_line(hull, epsilon).unwrap();
        }
    }
}
