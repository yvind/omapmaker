use crate::Result;
use std::collections::{hash_map::Keys, HashMap};

use eframe::{
    egui::{self, Color32, Stroke},
    emath,
};
use geo::Coord;
use log::{log, Level};
use proj4rs::{transform::transform, Proj};

use omap::{objects::MapObject, symbols::*, Omap};

use super::*;

trait Drawable {
    /// converting a symbol to something drawable to screen
    /// needs to know what crs to unproject to lat/lon
    fn into_drawable_geometry(
        self,
        ref_point: Coord,
        crs: Option<u16>,
        bezier_error: Option<f64>,
    ) -> Result<DrawableGeometry>;
}

impl Drawable for MapObject {
    fn into_drawable_geometry(
        self,
        ref_point: Coord,
        crs: Option<u16>,
        bezier_error: Option<f64>,
    ) -> Result<DrawableGeometry> {
        let dg = match self {
            MapObject::AreaObject(area_object) => DrawableGeometry::Polygon(
                DrawablePolygonObject::from_geo(area_object.polygon, ref_point, crs, bezier_error)?,
            ),
            MapObject::LineObject(line_object) => DrawableGeometry::Line(
                DrawableLineObject::from_geo(line_object.line, ref_point, crs, bezier_error)?,
            ),
            MapObject::PointObject(point_object) => {
                DrawableGeometry::Point(DrawablePointObject::from_geo(
                    point_object.point,
                    point_object.rotation,
                    ref_point,
                    crs,
                )?)
            }
            MapObject::TextObject(text_object) => DrawableGeometry::Text(
                DrawableTextObject::from_geo(text_object.point, text_object.text, ref_point, crs)?,
            ),
        };
        Ok(dg)
    }
}
pub struct DrawableOmap {
    hull: Vec<walkers::Position>,
    map_objects: HashMap<Symbol, Vec<DrawableGeometry>>,
}

impl DrawableOmap {
    pub fn keys(&self) -> Keys<'_, Symbol, Vec<DrawableGeometry>> {
        self.map_objects.keys()
    }

    pub fn from_omap(omap: Omap, hull: geo::LineString, bezier_error: Option<f64>) -> Self {
        let ref_point = omap.get_ref_point();
        let crs = omap.get_crs();

        let global_hull = if let Some(epsg) = crs {
            let wgs = Proj::from_epsg_code(4326).unwrap();
            let local = Proj::from_epsg_code(epsg).unwrap();

            let mut points: Vec<(f64, f64)> = hull
                .0
                .into_iter()
                .map(|c| (c.x + ref_point.x, c.y + ref_point.y))
                .collect();

            transform(&local, &wgs, points.as_mut_slice()).unwrap();

            points
                .into_iter()
                .map(|t| walkers::pos_from_lon_lat(t.0.to_degrees(), t.1.to_degrees()))
                .collect()
        } else {
            hull.0
                .into_iter()
                .map(|c| walkers::pos_from_lon_lat(c.x + ref_point.x, c.y + ref_point.y))
                .collect()
        };

        DrawableOmap {
            hull: global_hull,
            map_objects: Self::into_drawable(omap.objects, ref_point, crs, bezier_error),
        }
    }

    fn into_drawable(
        mut omap_objs: HashMap<Symbol, Vec<MapObject>>,
        ref_point: Coord,
        crs: Option<u16>,
        bezier_error: Option<f64>,
    ) -> HashMap<Symbol, Vec<DrawableGeometry>> {
        let mut drawable_objs = HashMap::with_capacity(omap_objs.len());
        for (symbol, objs) in omap_objs.drain() {
            let bezier = match symbol {
                // basemap should never be converted to beziers
                Symbol::Line(LineSymbol::BasemapContour)
                | Symbol::Line(LineSymbol::NegBasemapContour) => None,
                _ => bezier_error,
            };

            drawable_objs.insert(
                symbol,
                objs.into_iter()
                    .filter_map(|o| match o.into_drawable_geometry(ref_point, crs, bezier) {
                        Ok(o) => Some(o),
                        Err(e) => {
                            log!(
                                Level::Warn,
                                "Unable to convert a map object to drawable map object with error {e}"
                            );
                            None
                        }
                    })
                    .collect(),
            );
        }

        drawable_objs
    }

    pub fn update(&mut self, mut other: Self) {
        // assumes that the omap used for any update and only differs in the contained map_objects
        for (key, objs) in other.map_objects.drain() {
            if objs.is_empty() {
                let _ = self.map_objects.remove(&key);
            } else {
                let _ = self.map_objects.insert(key, objs);
            }
        }
    }

    pub fn draw(
        &self,
        ui: &mut egui::Ui,
        projector: &walkers::Projector,
        visibilities: &HashMap<Symbol, bool>,
        opacity: f32,
    ) {
        // project the hull:
        let points = self
            .hull
            .clone()
            .into_iter()
            .map(|p| projector.project(p))
            .collect();

        // not necessarily a convex polygon, but close
        ui.painter().add(egui::Shape::convex_polygon(
            points,
            Color32::WHITE.gamma_multiply(opacity), //.gamma_multiply(0.8),
            Stroke::new(2., Color32::RED),
        ));

        for symbol in Symbol::draw_order() {
            if let Some(vis) = visibilities.get(&symbol) {
                if !vis {
                    continue;
                }
            } else {
                continue;
            }

            if let Some((special, mut stroke)) = symbol.stroke(
                projector.scale_pixel_per_meter(projector.unproject(emath::Pos2::new(0.5, 0.5))),
            ) {
                stroke.color = stroke.color.gamma_multiply(opacity);

                if let Some(objs) = self.map_objects.get(&symbol) {
                    for obj in objs {
                        obj.draw(ui, projector, stroke, special);
                    }
                }
            }
        }
    }
}
