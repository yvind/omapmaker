use super::{LineString, MultiPolygon};
pub use geo::MultiLineString;

pub trait MapMultiLineString {
    fn from_polygons(polys: MultiPolygon) -> MultiLineString;
    fn clip(self, overlay: &LineString) -> MultiLineString;
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

    fn clip(self, overlay: &LineString) -> MultiLineString {
        self
    }
}
