use geo::{MapCoords, MapCoordsInPlace};
use omap::{
    Omap,
    objects::{AreaObject, BezierPath, BezierPolygon, LineObject, PointObject},
};

use super::{
    InternalMap, MapObject, Symbol,
    object::{PRESERVE_CONTOUR_GEOMETRY_TAG, STABLE_CONTOUR_SEAM_TAG},
};
use crate::parameters::{GeometryParameters, Scale};

impl InternalMap {
    pub fn into_omap(
        mut self,
        meters_above_sea: f64,
        geo_params: &GeometryParameters,
    ) -> crate::Result<Omap> {
        let crs = self
            .crs
            .as_ref()
            .map(|crs| omap::geo_referencing::CrsType::Epsg(crs.epsg() as u16))
            .unwrap_or(omap::geo_referencing::CrsType::Local);

        let mut omap = match self.scale {
            Scale::S10_000 => {
                Omap::default_10_000_geo_referenced(self.ref_point, crs, meters_above_sea)?
            }
            Scale::S15_000 => {
                Omap::default_15_000_geo_referenced(self.ref_point, crs, meters_above_sea)?
            }
        };
        let transform = omap.geo_referencing.create_transform();

        for (sym, objects) in self.objects.drain() {
            let bezier_error = geo_params
                .bezier_error_for_symbol(sym)
                .map(|e| self.scale.meters_to_paper_mm(e));

            for object in objects {
                let omap_object: omap::objects::MapObject = match object {
                    MapObject::Area {
                        mut object,
                        symbol,
                        tags,
                    } => {
                        object.map_coords_in_place(|c| c + self.ref_point);

                        let geometry = transform.to_map_polygon(object);
                        let geometry = if let Some(bezier) = bezier_error {
                            match BezierPolygon::fit_polygon(geometry.clone(), bezier.try_into()?) {
                                Ok(s) => s,
                                Err(_) => geometry.into(),
                            }
                        } else {
                            geometry.into()
                        };

                        let mut area = AreaObject::new(
                            Symbol::Area(symbol)
                                .get_omap_symbol_ref(&omap.symbols)
                                .and_then(|s| s.try_into().ok()),
                            geometry,
                        );
                        *area.tags_mut() = tags;
                        area.into()
                    }
                    MapObject::Line {
                        object,
                        symbol,
                        mut tags,
                    } => {
                        tags.remove(PRESERVE_CONTOUR_GEOMETRY_TAG);
                        tags.remove(STABLE_CONTOUR_SEAM_TAG);
                        let object = object.map_coords(|c| c + self.ref_point);

                        let geometry = transform.to_map_linestring(object);
                        let geometry = if let Some(bezier) = bezier_error {
                            match BezierPath::fit_line_string(geometry.clone(), bezier.try_into()?)
                            {
                                Ok(s) => s,
                                Err(_) => geometry.into(),
                            }
                        } else {
                            geometry.into()
                        };

                        let mut line = LineObject::new(
                            Symbol::Line(symbol)
                                .get_omap_symbol_ref(&omap.symbols)
                                .and_then(|s| s.as_line_path()),
                            geometry,
                        );
                        *line.tags_mut() = tags;
                        line.into()
                    }
                    MapObject::Point {
                        object,
                        symbol,
                        rotation,
                        tags,
                    } => {
                        let object = object.map_coords(|c| c + self.ref_point);
                        let mut point = PointObject::new(
                            Symbol::Point(symbol)
                                .get_omap_symbol_ref(&omap.symbols)
                                .and_then(|s| s.as_point()),
                            transform.to_map_point(object),
                        );
                        point.rotation = rotation;
                        *point.tags_mut() = tags;
                        point.into()
                    }
                };
                omap.parts.get_mut(0).unwrap().add_object(omap_object);
            }
        }

        Ok(omap)
    }
}
