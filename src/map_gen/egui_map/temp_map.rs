use std::collections::HashMap;

use geo::{Coord, LineString};
use proj_core::CrsDef;

use crate::parameters::Scale;

#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, PartialOrd, Ord)]
pub enum Symbol {
    Area(AreaSymbol),
    Line(LineSymbol),
    Point(PointSymbol),
}

impl From<AreaSymbol> for Symbol {
    fn from(value: AreaSymbol) -> Self {
        Symbol::Area(value)
    }
}

impl From<LineSymbol> for Symbol {
    fn from(value: LineSymbol) -> Self {
        Symbol::Line(value)
    }
}

impl From<PointSymbol> for Symbol {
    fn from(value: PointSymbol) -> Self {
        Symbol::Point(value)
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, PartialOrd, Ord)]
pub enum AreaSymbol {
    RoughOpenLand,
    OpenLand,
    SandyGround,
    BareRock,
    LightGreen,
    MediumGreen,
    DarkGreen,
    Marsh,
    PrivateArea,
    PavedAreaWithBoundary,
    ShallowWaterWithSolidBankLine,
    UncrossableWaterWithBankLine,
    GiganticBoulder,
    Building,
    OutOfBounds,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, PartialOrd, Ord)]
pub enum LineSymbol {
    BasemapContour,
    FormLine,
    Contour,
    IndexContour,
    NegBasemapContour,
    SmallCrossableWatercourse,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, PartialOrd, Ord)]
pub enum PointSymbol {
    SlopeLineFormLine,
    SlopeLineContour,
    DotKnoll,
    ElongatedDotKnoll,
    UDepression,
    SmallBoulder,
    LargeBoulder,
}

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

    pub fn add_elevation_tag(&mut self, elevation: f64) {
        let key = "Elevation".to_string();
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
}

pub struct TempMap {
    pub ref_point: geo::Coord,
    pub scale: Scale,
    pub crs: Option<CrsDef>,
    pub objects: HashMap<Symbol, Vec<MapObject>>,
}

impl TempMap {
    pub fn new(ref_point: geo::Coord, scale: Scale, crs: Option<CrsDef>) -> Self {
        TempMap {
            ref_point,
            scale,
            crs,
            objects: HashMap::new(),
        }
    }

    pub fn add_object(&mut self, map_object: MapObject) {
        let symbol = map_object.get_symbol();

        if let Some(vec) = self.objects.get_mut(&symbol) {
            vec.push(map_object);
        } else {
            self.objects.insert(symbol, vec![map_object]);
        }
    }

    pub fn reserve_capacity(&mut self, symbol: impl Into<Symbol>, additional: usize) {
        let symbol = symbol.into();
        if let Some(vec) = self.objects.get_mut(&symbol) {
            vec.reserve(additional);
        } else {
            self.objects.insert(symbol, Vec::with_capacity(additional));
        }
    }

    pub fn remove_empty_keys(&mut self) {
        self.objects.retain(|_, v| !v.is_empty());
    }

    pub fn mark_basemap_depressions(&mut self) {
        let basemap = self
            .objects
            .get_mut(&Symbol::Line(LineSymbol::BasemapContour));

        if basemap.is_none() {
            return;
        }
        let basemap = basemap.unwrap();

        let mut neg_basemap = Vec::new();

        let mut i = 0;
        while i < basemap.len() {
            if let MapObject::Line {
                object,
                symbol: _,
                tags: _,
            } = &basemap[i]
            {
                if object.is_closed() && line_string_signed_area(&object) < 0. {
                    let mut neg = basemap.swap_remove(i);

                    let _ = neg.change_symbol(LineSymbol::NegBasemapContour);

                    neg_basemap.push(neg);
                } else {
                    i += 1;
                }
            }
        }

        if let Some(existing_neg) = self
            .objects
            .get_mut(&Symbol::Line(LineSymbol::NegBasemapContour))
        {
            existing_neg.extend(neg_basemap);
        } else {
            let _ = self
                .objects
                .insert(Symbol::Line(LineSymbol::NegBasemapContour), neg_basemap);
        }
    }

    /// Turn small contour loops to dotknolls and depressions and remove the smallest ones
    /// dot_knolls smaller than (min+max)/2 + min will never be drawn as elongated
    pub fn make_dotknolls_and_depressions(
        &mut self,
        min_area: f64,
        max_area: f64,
        elongated_aspect: f64,
    ) {
        let keys = [
            Symbol::Line(LineSymbol::Contour),
            Symbol::Line(LineSymbol::FormLine),
            Symbol::Line(LineSymbol::IndexContour),
        ];

        let min_elongated_area = (max_area + min_area) / 2. + min_area;

        for key in keys {
            let contours = self.objects.get_mut(&key);

            if contours.is_none() {
                continue;
            }

            let contours = contours.unwrap();
            let mut small_loops = Vec::with_capacity(contours.len());

            let mut i = 0;
            while i < contours.len() {
                let contour_object = &contours[i];
                if let MapObject::Line {
                    object,
                    symbol,
                    tags,
                } = contour_object
                {
                    if object.is_closed() {
                        let area = line_string_signed_area(&object);

                        if area.abs() <= max_area {
                            small_loops.push(contours.swap_remove(i));
                        } else {
                            i += 1;
                        }
                    } else {
                        i += 1;
                    }
                } else {
                    panic!("Non-line object under contour symbol in objects hashmap");
                }
            }

            for small_loop in small_loops {
                if let MapObject::Line {
                    object,
                    symbol,
                    tags,
                } = &small_loop
                {
                    let area = line_string_signed_area(&object);

                    // ignore too small loops
                    if area.abs() < min_area {
                        continue;
                    }

                    let (aspect, mid_point, rotation) =
                        line_string_aspect_midpoint_rotation(&object);

                    let map_object = if area < 0. {
                        MapObject::Point {
                            object: geo::Point(mid_point),
                            symbol: PointSymbol::UDepression,
                            rotation,
                            tags: HashMap::new(),
                        }
                    } else if aspect < elongated_aspect || area < min_elongated_area {
                        MapObject::Point {
                            object: geo::Point(mid_point),
                            symbol: PointSymbol::DotKnoll,
                            rotation,
                            tags: HashMap::new(),
                        }
                    } else {
                        MapObject::Point {
                            object: geo::Point(mid_point),
                            symbol: PointSymbol::ElongatedDotKnoll,
                            rotation,
                            tags: HashMap::new(),
                        }
                    };
                    self.add_object(map_object);
                }
            }
        }
    }
}

fn line_string_signed_area(line: &LineString) -> f64 {
    if line.0.len() < 3 {
        return 0.;
    }
    let mut area: f64 = 0.;
    for i in 0..line.0.len() - 1 {
        area += line.0[i].x * line.0[i + 1].y - line.0[i].y * line.0[i + 1].x;
    }
    0.5 * area
}

fn line_string_aspect_midpoint_rotation(line: &LineString) -> (f64, Coord, f64) {
    let mut midpoint = Coord::zero();

    let len_f64 = line.0.len() as f64;
    for c in line.0.iter() {
        midpoint = midpoint + *c;
    }
    midpoint = midpoint / len_f64;

    // Calculate second moments
    let mu20 = line
        .0
        .iter()
        .map(|p| (p.x - midpoint.x).powi(2))
        .sum::<f64>()
        / len_f64;
    let mu02 = line
        .0
        .iter()
        .map(|p| (p.y - midpoint.y).powi(2))
        .sum::<f64>()
        / len_f64;
    let mu11 = line
        .0
        .iter()
        .map(|p| (p.x - midpoint.x) * (p.y - midpoint.y))
        .sum::<f64>()
        / len_f64;

    // Calculate elongation using eigenvalues of the covariance matrix
    let temp = ((mu20 - mu02).powi(2) + 4.0 * mu11.powi(2)).sqrt();
    let lambda1 = (mu20 + mu02 + temp) / 2.0;
    let lambda2 = (mu20 + mu02 - temp) / 2.0;

    // Handle potential numerical issues
    const EPS: f64 = 1000. * f64::EPSILON;
    if lambda2.abs() <= EPS {
        // colinear points
        if mu11.abs() <= EPS {
            // horizontal or vertical
            return (
                f64::INFINITY,
                midpoint,
                if mu20 > mu02 {
                    0.0
                } else {
                    std::f64::consts::FRAC_PI_2
                },
            );
        } else {
            // Diagonal line
            let angle = 0.5 * f64::atan2(2.0 * mu11, mu20 - mu02);
            return (f64::INFINITY, midpoint, normalize_angle(angle));
        }
    }

    let elongation = lambda1 / lambda2;

    // Calculate the angle of the major axis
    // The eigenvector for the larger eigenvalue gives the major axis direction
    let angle = if mu11.abs() <= EPS {
        // Principal axes are aligned with coordinate axes
        if mu20 >= mu02 {
            0.0
        } else {
            std::f64::consts::FRAC_PI_2
        }
    } else {
        // General case: use eigenvector of larger eigenvalue
        // For 2x2 symmetric matrix, eigenvector is [mu11, lambda1 - mu20]
        f64::atan2(lambda1 - mu20, mu11) + std::f64::consts::FRAC_PI_2
    };

    (elongation, midpoint, normalize_angle(angle))
}

fn normalize_angle(angle: f64) -> f64 {
    let mut normalized = angle % std::f64::consts::PI;
    if normalized < 0.0 {
        normalized += std::f64::consts::PI;
    }
    normalized
}
