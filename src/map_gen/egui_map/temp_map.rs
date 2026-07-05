use std::collections::HashMap;

use geo::{MapCoords, MapCoordsInPlace};
use omap::{
    NonNegativeF64, Omap,
    objects::{AreaObject, LineObject, PointObject},
    symbols::{WeakAreaPathSymbol, WeakLinePathSymbol},
};
use proj_core::CrsDef;
use rstar::{PointDistance, RTree, primitives::GeomWithData};

use crate::parameters::Scale;

#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, PartialOrd, Ord)]
pub enum Symbol {
    Area(AreaSymbol),
    Line(LineSymbol),
    Point(PointSymbol),
}

impl std::fmt::Display for Symbol {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            Symbol::Area(area_symbol) => write!(f, "{:?}", area_symbol),
            Symbol::Line(line_symbol) => write!(f, "{:?}", line_symbol),
            Symbol::Point(point_symbol) => write!(f, "{:?}", point_symbol),
        }
    }
}

impl Symbol {
    pub fn get_omap_symbol<'a>(
        &self,
        symbol_set: &'a omap::symbols::SymbolSet,
    ) -> Option<&'a omap::symbols::Symbol> {
        let code = match self {
            Symbol::Area(area_symbol) => area_symbol.get_code(),
            Symbol::Line(line_symbol) => line_symbol.get_code(),
            Symbol::Point(point_symbol) => point_symbol.get_code(),
        };

        symbol_set.get_symbol_by_code(code)
    }
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
    WhiteForest,
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

impl AreaSymbol {
    pub fn get_code(&self) -> omap::Code {
        match self {
            AreaSymbol::RoughOpenLand => omap::Code::new(403, 0, 0),
            AreaSymbol::OpenLand => omap::Code::new(401, 0, 0),
            AreaSymbol::SandyGround => omap::Code::new(213, 0, 0),
            AreaSymbol::BareRock => omap::Code::new(214, 0, 0),
            AreaSymbol::LightGreen => omap::Code::new(406, 0, 0),
            AreaSymbol::MediumGreen => omap::Code::new(408, 0, 0),
            AreaSymbol::DarkGreen => omap::Code::new(410, 0, 0),
            AreaSymbol::Marsh => omap::Code::new(308, 0, 0),
            AreaSymbol::PrivateArea => omap::Code::new(520, 0, 0),
            AreaSymbol::PavedAreaWithBoundary => omap::Code::new(501, 0, 0),
            AreaSymbol::ShallowWaterWithSolidBankLine => omap::Code::new(302, 0, 0),
            AreaSymbol::UncrossableWaterWithBankLine => omap::Code::new(301, 0, 0),
            AreaSymbol::GiganticBoulder => omap::Code::new(206, 0, 0),
            AreaSymbol::Building => omap::Code::new(521, 0, 0),
            AreaSymbol::OutOfBounds => omap::Code::new(709, 0, 0),
            AreaSymbol::WhiteForest => omap::Code::new(405, 0, 0),
        }
    }
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

impl LineSymbol {
    pub fn get_code(&self) -> omap::Code {
        match self {
            LineSymbol::BasemapContour => omap::Code::new(101, 2, 0),
            LineSymbol::FormLine => omap::Code::new(103, 0, 0),
            LineSymbol::Contour => omap::Code::new(101, 0, 0),
            LineSymbol::IndexContour => omap::Code::new(102, 0, 0),
            LineSymbol::NegBasemapContour => omap::Code::new(101, 3, 0),
            LineSymbol::SmallCrossableWatercourse => omap::Code::new(305, 0, 0),
        }
    }
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

impl PointSymbol {
    pub fn get_code(&self) -> omap::Code {
        match self {
            PointSymbol::SlopeLineFormLine => omap::Code::new(103, 1, 0),
            PointSymbol::SlopeLineContour => omap::Code::new(101, 1, 0),
            PointSymbol::DotKnoll => omap::Code::new(109, 0, 0),
            PointSymbol::ElongatedDotKnoll => omap::Code::new(110, 0, 0),
            PointSymbol::UDepression => omap::Code::new(111, 0, 0),
            PointSymbol::SmallBoulder => omap::Code::new(204, 0, 0),
            PointSymbol::LargeBoulder => omap::Code::new(205, 0, 0),
        }
    }
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

struct MergeLine {
    object: geo::LineString,
    symbol: LineSymbol,
    tags: HashMap<String, String>,
}

impl MergeLine {
    fn elevation_key(&self) -> Option<i64> {
        self.tags
            .get("Elevation")
            .and_then(|elevation| elevation.parse::<f64>().ok())
            .map(|elevation| (elevation * 100.).round() as i64)
    }

    fn start_point(&self) -> [f64; 2] {
        let start = self.object.0[0];
        [start.x, start.y]
    }

    fn end_point(&self) -> [f64; 2] {
        let end = self.object.0[self.object.0.len() - 1];
        [end.x, end.y]
    }

    fn into_map_object(self) -> MapObject {
        MapObject::Line {
            object: self.object,
            symbol: self.symbol,
            tags: self.tags,
        }
    }
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

    pub fn into_omap(
        mut self,
        meters_above_sea: f64,
        bezier_error: Option<f64>,
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
        let transform = omap.geo_referencing.get_transform();

        for (_, objects) in self.objects.drain() {
            for object in objects {
                let omap_object: omap::objects::MapObject = match object {
                    MapObject::Area {
                        mut object,
                        symbol,
                        tags,
                    } => {
                        object.map_coords_in_place(|c| c + self.ref_point);
                        let mut area = AreaObject::new(
                            WeakAreaPathSymbol::try_from(
                                Symbol::Area(symbol)
                                    .get_omap_symbol(&omap.symbols)
                                    .ok_or_else(|| omap::Error::MissingSymbolId)?
                                    .downgrade(),
                            )?,
                            transform.to_map_polygon(object),
                        );
                        area.tags = tags;
                        area.into()
                    }
                    MapObject::Line {
                        object,
                        symbol,
                        tags,
                    } => {
                        let object = object.map_coords(|c| c + self.ref_point);
                        let mut line = LineObject::new(
                            WeakLinePathSymbol::try_from(
                                Symbol::Line(symbol)
                                    .get_omap_symbol(&omap.symbols)
                                    .ok_or_else(|| omap::Error::MissingSymbolId)?
                                    .downgrade(),
                            )?,
                            transform.to_map_linestring(object),
                        );
                        line.tags = tags;
                        line.write_as_bezier = if let Some(err) = bezier_error
                            && !matches!(
                                symbol,
                                LineSymbol::BasemapContour | LineSymbol::NegBasemapContour
                            ) {
                            NonNegativeF64::try_from(err).ok()
                        } else {
                            None
                        };
                        line.into()
                    }
                    MapObject::Point {
                        object,
                        symbol,
                        rotation,
                        tags,
                    } => {
                        let object = object.map_coords(|c| c + self.ref_point);
                        let omap_symbol = Symbol::Point(symbol)
                            .get_omap_symbol(&omap.symbols)
                            .ok_or_else(|| omap::Error::MissingSymbolId)?;
                        let symbol = match omap_symbol {
                            omap::symbols::Symbol::Point(symbol) => std::rc::Rc::downgrade(symbol),
                            _ => Err(omap::Error::MissingSymbolId)?,
                        };
                        let mut point = PointObject::new(symbol, transform.to_map_point(object));
                        point.rotation = rotation;
                        point.tags = tags;
                        point.into()
                    }
                };
                omap.parts.0[0].add_object(omap_object);
            }
        }

        Ok(omap)
    }

    pub fn mark_basemap_depressions(&mut self) {
        let basemap = self
            .objects
            .get_mut(&Symbol::Line(LineSymbol::BasemapContour));

        let Some(basemap) = basemap else {
            return;
        };

        let mut neg_basemap = Vec::new();

        let mut i = 0;
        while i < basemap.len() {
            if let MapObject::Line {
                object,
                symbol: _,
                tags: _,
            } = &basemap[i]
            {
                if object.is_closed() && line_string_signed_area(object) < 0. {
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

            let Some(contours) = contours else {
                continue;
            };
            let mut small_loops = Vec::with_capacity(contours.len());

            let mut i = 0;
            while i < contours.len() {
                let contour_object = &contours[i];
                if let MapObject::Line {
                    object,
                    symbol: _,
                    tags: _,
                } = contour_object
                {
                    if object.is_closed() {
                        let area = line_string_signed_area(object);

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
                    symbol: _,
                    tags: _,
                } = &small_loop
                {
                    let area = line_string_signed_area(object);

                    // ignore too small loops
                    if area.abs() < min_area {
                        continue;
                    }

                    let (aspect, mid_point, rotation) =
                        line_string_aspect_midpoint_rotation(object);

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

    /// Merge line objects that are tip to tail.
    /// Line ends (directed) of the same symbol that are less than `delta`
    /// units apart are merged. Elevation tags are respected and only elements
    /// with equal elevation tags can be merged.
    pub fn merge_lines(&mut self, delta: f64) {
        for (key, map_objects) in self.objects.iter_mut() {
            if !matches!(key, Symbol::Line(_)) {
                continue;
            }
            let delta = delta * delta;

            let mut unclosed_objects = Vec::with_capacity(map_objects.len());

            let mut i = 0;
            while i < map_objects.len() {
                if let MapObject::Line {
                    object,
                    symbol: _,
                    tags: _,
                } = &map_objects[i]
                {
                    if !object.is_closed() && object.0.len() >= 2 {
                        let MapObject::Line {
                            object,
                            symbol,
                            tags,
                        } = map_objects.swap_remove(i)
                        else {
                            unreachable!("checked line object before swap_remove");
                        };
                        unclosed_objects.push(MergeLine {
                            object,
                            symbol,
                            tags,
                        });
                    } else {
                        i += 1;
                    }
                } else {
                    i += 1;
                }
            }

            let mut unclosed_object_groups = HashMap::<Option<i64>, Vec<MergeLine>>::new();
            for unclosed_object in unclosed_objects {
                unclosed_object_groups
                    .entry(unclosed_object.elevation_key())
                    .or_default()
                    .push(unclosed_object);
            }

            for (_, mut unclosed_objects) in unclosed_object_groups {
                let (line_ends, line_starts): (Vec<_>, Vec<_>) = unclosed_objects
                    .iter()
                    .enumerate()
                    .map(|(i, o)| (GeomWithData::new(o.end_point(), i), o.start_point()))
                    .collect();

                // detect the merges needed
                let end_tree = RTree::bulk_load(line_ends);

                let mut merges = Vec::with_capacity(line_starts.len());
                for (start_i, line_start) in line_starts.iter().enumerate() {
                    if let Some(nn) = end_tree.nearest_neighbor(*line_start)
                        && nn.distance_2(line_start) <= delta
                    {
                        merges.push((start_i, nn.data));
                    }
                }

                // start doing merges keeping track of the moved objects
                while let Some(merge) = merges.pop() {
                    if merge.0 == merge.1 {
                        let mut line = unclosed_objects.swap_remove(merge.0);
                        line.object.close();

                        map_objects.push(line.into_map_object());
                    } else {
                        // merge
                        let part2 = unclosed_objects.swap_remove(merge.0);

                        let part1 = if merge.1 >= unclosed_objects.len() {
                            &mut unclosed_objects[merge.0]
                        } else {
                            &mut unclosed_objects[merge.1]
                        };

                        let _ = part1.object.0.pop();
                        part1.object.0.extend(part2.object.0);
                    }
                    // update map
                    let mut i = 0;
                    while i < merges.len() {
                        let other_merge = &mut merges[i];

                        // find merges made impossible
                        if other_merge.1 == merge.1 || other_merge.0 == merge.0 {
                            let _ = merges.swap_remove(i);
                            continue;
                        } else {
                            i += 1;
                        }

                        // update map as merge.0 is now called merge.1
                        if other_merge.0 == merge.0 {
                            other_merge.0 = merge.1
                        }
                        if other_merge.1 == merge.0 {
                            other_merge.1 = merge.1
                        }

                        // correct map for swap remove moving object
                        if other_merge.0 >= unclosed_objects.len() {
                            other_merge.0 = merge.0;
                        }
                        if other_merge.1 >= unclosed_objects.len() {
                            other_merge.1 = merge.0;
                        }
                    }
                }
                let unclosed = unclosed_objects.into_iter().map(|mut line_object| {
                    // check if it is almost closed
                    let start = line_object.object.0[0];
                    let end = line_object.object.0[line_object.object.0.len() - 1];

                    if (start.x - end.x).powi(2) + (start.y - end.y).powi(2) <= delta {
                        line_object.object.close();
                    }

                    line_object.into_map_object()
                });

                map_objects.extend(unclosed);
            }
        }
    }
}

fn line_string_signed_area(line: &geo::LineString) -> f64 {
    if line.0.len() < 3 {
        return 0.;
    }
    let mut area: f64 = 0.;
    for i in 0..line.0.len() - 1 {
        area += line.0[i].x * line.0[i + 1].y - line.0[i].y * line.0[i + 1].x;
    }
    0.5 * area
}

fn line_string_aspect_midpoint_rotation(line: &geo::LineString) -> (f64, geo::Coord, f64) {
    let mut midpoint = geo::Coord::zero();

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
