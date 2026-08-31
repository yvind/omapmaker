use std::collections::HashMap;

use super::{AreaSymbol, LineSymbol, PointSymbol, Symbol};

pub enum MapObject {
    Area {
        object: geo::Polygon,
        symbol: AreaSymbol,
        tags: HashMap<String, String>,
    },
    Line {
        object: geo::LineString,
        symbol: LineSymbol,
        tags: HashMap<String, String>,
    },
    Point {
        object: geo::Point,
        symbol: PointSymbol,
        rotation: f64,
        tags: HashMap<String, String>,
    },
}

pub(super) const PRESERVE_CONTOUR_GEOMETRY_TAG: &str = "_omapmaker_preserve_contour_geometry";
pub(super) const STABLE_CONTOUR_SEAM_TAG: &str = "_omapmaker_stable_contour_seam";

impl MapObject {
    pub fn get_symbol(&self) -> Symbol {
        match self {
            MapObject::Area {
                object: _,
                symbol,
                tags: _,
            } => Symbol::Area(*symbol),
            MapObject::Line {
                object: _,
                symbol,
                tags: _,
            } => Symbol::Line(*symbol),
            MapObject::Point {
                object: _,
                symbol,
                rotation: _,
                tags: _,
            } => Symbol::Point(*symbol),
        }
    }

    pub fn change_symbol(&mut self, symbol: impl Into<Symbol>) -> anyhow::Result<()> {
        let symbol = symbol.into();
        match (self, symbol) {
            (
                MapObject::Area {
                    object: _,
                    symbol,
                    tags: _,
                },
                Symbol::Area(area_symbol),
            ) => *symbol = area_symbol,
            (
                MapObject::Line {
                    object: _,
                    symbol,
                    tags: _,
                },
                Symbol::Line(line_symbol),
            ) => *symbol = line_symbol,
            (
                MapObject::Point {
                    object: _,
                    symbol,
                    rotation: _,
                    tags: _,
                },
                Symbol::Point(point_symbol),
            ) => *symbol = point_symbol,
            _ => return Err(anyhow::anyhow!("Incompatible symbol exchange")),
        }
        Ok(())
    }

    pub fn add_elevation_tag(&mut self, elevation: f32) {
        let key = "elev".to_string();
        let value = elevation.to_string();

        match self {
            MapObject::Area {
                object: _,
                symbol: _,
                tags,
            } => {
                tags.insert(key, value);
            }
            MapObject::Line {
                object: _,
                symbol: _,
                tags,
            } => {
                tags.insert(key, value);
            }
            MapObject::Point {
                object: _,
                symbol: _,
                rotation: _,
                tags,
            } => {
                tags.insert(key, value);
            }
        }
    }

    pub fn preserve_contour_geometry(&mut self) {
        if let MapObject::Line { tags, .. } = self {
            tags.insert(PRESERVE_CONTOUR_GEOMETRY_TAG.to_string(), String::new());
        }
    }

    pub fn stabilize_contour_seam(&mut self) {
        if let MapObject::Line { tags, .. } = self {
            tags.insert(STABLE_CONTOUR_SEAM_TAG.to_string(), String::new());
        }
    }
}
