use crate::Result;
use std::collections::{HashMap, hash_map::Keys};

use eframe::{
    egui::{self, Color32, Stroke},
    emath,
};
use geo::Coord;
use log::{Level, log};
use proj_core::{CrsDef, Transform};

use super::*;
use crate::{
    map_gen::egui_map::{AreaSymbol, LineSymbol, MapObject, Symbol, TempMap},
    parameters::GeometryParameters,
};

trait Drawable {
    /// converting a symbol to something drawable to screen
    /// needs to know what crs to unproject to lat/lon
    fn into_drawable_geometry(
        self,
        ref_point: Coord,
        crs: Option<CrsDef>,
        bezier_error: Option<f64>,
    ) -> Result<DrawableGeometry>;
}

impl Drawable for MapObject {
    fn into_drawable_geometry(
        self,
        ref_point: Coord,
        crs: Option<CrsDef>,
        bezier_error: Option<f64>,
    ) -> Result<DrawableGeometry> {
        let dg = match self {
            MapObject::Area {
                object,
                symbol: _,
                tags: _,
            } => DrawableGeometry::Polygon(DrawablePolygonObject::from_geo(
                object,
                ref_point,
                crs,
                bezier_error,
            )?),
            MapObject::Line {
                object,
                symbol: _,
                tags: _,
            } => DrawableGeometry::Line(DrawableLineObject::from_geo(
                object,
                ref_point,
                crs,
                bezier_error,
            )?),
            MapObject::Point {
                object: point_object,
                rotation,
                symbol: _,
                tags: _,
            } => DrawableGeometry::Point(DrawablePointObject::from_geo(
                point_object,
                rotation,
                ref_point,
                crs,
            )?),
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

    pub fn from_temp_map(
        tmap: TempMap,
        hull: geo::LineString,
        geometry: &GeometryParameters,
    ) -> Result<Self> {
        let ref_point = tmap.ref_point;

        let global_hull = if let Some(epsg) = &tmap.crs {
            let transform = Transform::from_epsg(epsg.epsg(), 4326)?;

            let points: Vec<(f64, f64)> = hull
                .0
                .into_iter()
                .map(|c| (c.x + ref_point.x, c.y + ref_point.y))
                .collect();

            let transformed_points = transform.convert_batch(&points)?;

            transformed_points
                .into_iter()
                .map(|t| walkers::lon_lat(t.0, t.1))
                .collect()
        } else {
            hull.0
                .into_iter()
                .map(|c| walkers::lon_lat(c.x + ref_point.x, c.y + ref_point.y))
                .collect()
        };

        Ok(DrawableOmap {
            hull: global_hull,
            map_objects: Self::into_drawable(tmap.objects, ref_point, tmap.crs, geometry),
        })
    }

    fn into_drawable(
        mut omap_objs: HashMap<Symbol, Vec<MapObject>>,
        ref_point: Coord,
        crs: Option<CrsDef>,
        geometry: &GeometryParameters,
    ) -> HashMap<Symbol, Vec<DrawableGeometry>> {
        let mut drawable_objs = HashMap::with_capacity(omap_objs.len());
        for (symbol, objs) in omap_objs.drain() {
            let bezier = match symbol {
                // basemap should never be converted to beziers
                Symbol::Line(LineSymbol::BasemapContour)
                | Symbol::Line(LineSymbol::NegBasemapContour) => None,
                _ => geometry.bezier_error_for_symbol(symbol),
            };

            drawable_objs.insert(
                symbol,
                objs.into_iter()
                    .filter_map(|o| match o.into_drawable_geometry(ref_point, crs.clone(), bezier) {
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
        projector: &walkers::ScreenProjector,
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

        let show_white_forest = visibilities
            .get(&Symbol::Area(AreaSymbol::WhiteForest))
            .copied()
            .unwrap_or(true);
        let fill = if show_white_forest {
            Color32::WHITE.gamma_multiply(opacity)
        } else {
            Color32::TRANSPARENT
        };

        // not necessarily a convex polygon, but close
        ui.painter().add(egui::Shape::convex_polygon(
            points,
            fill,
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
