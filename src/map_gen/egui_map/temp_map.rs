use std::collections::HashMap;

use geo::{Area, BooleanOps, BoundingRect, Buffer, Intersects, MapCoords, MapCoordsInPlace};
use omap::{
    Omap,
    objects::{AreaObject, BezierPath, BezierPolygon, LineObject, PointObject},
    symbols::{WeakAreaPathSymbol, WeakLinePathSymbol},
};
use proj_core::CrsDef;
use rstar::{AABB, PointDistance, RTree, RTreeObject, primitives::GeomWithData};

use crate::parameters::{GeometryParameters, Scale};

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
    ) -> anyhow::Result<Option<&'a omap::symbols::Symbol>> {
        let code = match self {
            Symbol::Area(area_symbol) => area_symbol.get_code(),
            Symbol::Line(line_symbol) => line_symbol.get_code(),
            Symbol::Point(point_symbol) => point_symbol.get_code(),
        };

        Ok(symbol_set.symbol_by_code(code)?)
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
            AreaSymbol::WhiteForest => omap::Code::new(405, 0, 0),
        }
    }

    pub fn min_size(&self, scale: &Scale) -> f64 {
        let a = match self {
            AreaSymbol::WhiteForest => 64.,
            AreaSymbol::RoughOpenLand => 225.,
            AreaSymbol::OpenLand => 64.,
            AreaSymbol::SandyGround => 225.,
            AreaSymbol::BareRock => 225.,
            AreaSymbol::LightGreen => 225.,
            AreaSymbol::MediumGreen => 110.,
            AreaSymbol::DarkGreen => 64.,
            AreaSymbol::Marsh => 45.,
            AreaSymbol::PrivateArea => 225.,
            AreaSymbol::PavedAreaWithBoundary => 225.,
            AreaSymbol::ShallowWaterWithSolidBankLine => 64.,
            AreaSymbol::UncrossableWaterWithBankLine => 64.,
            AreaSymbol::GiganticBoulder => 67.,
            AreaSymbol::Building => 56.,
        };
        let multiplier = match scale {
            Scale::S10_000 => 4. / 9.,
            Scale::S15_000 => 1.,
        };
        a * multiplier
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
    Cliff,
    ImpassableCliff,
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
            LineSymbol::Cliff => omap::Code::new(202, 0, 0),
            LineSymbol::ImpassableCliff => omap::Code::new(201, 0, 0),
        }
    }

    pub fn min_length(&self, scale: Scale, is_closed: bool) -> f64 {
        let l = match self {
            LineSymbol::BasemapContour => 3.,
            LineSymbol::FormLine => {
                if is_closed {
                    150.
                } else {
                    250.
                }
            }
            LineSymbol::Contour | LineSymbol::IndexContour => {
                if is_closed {
                    120.
                } else {
                    10.
                }
            }
            LineSymbol::NegBasemapContour => 3.,
            LineSymbol::SmallCrossableWatercourse => 15.,
            LineSymbol::Cliff => 9.,
            LineSymbol::ImpassableCliff => 9.,
        };
        let multiplier = match scale {
            Scale::S10_000 => 2. / 3.,
            Scale::S15_000 => 1.,
        };
        l * multiplier
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

const PRESERVE_CONTOUR_GEOMETRY_TAG: &str = "_omapmaker_preserve_contour_geometry";
const STABLE_CONTOUR_SEAM_TAG: &str = "_omapmaker_stable_contour_seam";

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

struct MergeArea {
    object: geo::Polygon,
    symbol: AreaSymbol,
    tags: HashMap<String, String>,
}

#[derive(Clone, Copy)]
struct IndexedPolygonEnvelope {
    envelope: AABB<[f64; 2]>,
    index: usize,
}

impl RTreeObject for IndexedPolygonEnvelope {
    type Envelope = AABB<[f64; 2]>;

    fn envelope(&self) -> Self::Envelope {
        self.envelope
    }
}

impl MergeLine {
    fn elevation_key(&self) -> Option<String> {
        self.tags.get("Elevation").cloned()
    }

    fn requires_exact_contour_merge(&self) -> bool {
        self.tags.contains_key(PRESERVE_CONTOUR_GEOMETRY_TAG)
            || self.tags.contains_key(STABLE_CONTOUR_SEAM_TAG)
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

impl MergeArea {
    fn signed_area(&self) -> f64 {
        self.object.signed_area()
    }

    fn abs_area(&self) -> f64 {
        self.signed_area().abs()
    }

    fn envelope(&self) -> Option<AABB<[f64; 2]>> {
        polygon_envelope(&self.object)
    }

    fn into_map_object(self) -> MapObject {
        MapObject::Area {
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

    pub fn merge_areas(&mut self, symbol: AreaSymbol, delta: f64) -> crate::Result<()> {
        let objects = self.objects.remove(&Symbol::Area(symbol));

        let Some(objects) = objects else {
            return Ok(());
        };

        let areas = objects
            .into_iter()
            .map(|mo| match mo {
                MapObject::Area {
                    object,
                    symbol: _,
                    tags: _,
                } => Ok(object),
                MapObject::Line {
                    object: _,
                    symbol: _,
                    tags: _,
                } => anyhow::bail!("Should not be any Line objects under an Area key"),
                MapObject::Point {
                    object: _,
                    symbol: _,
                    rotation: _,
                    tags: _,
                } => anyhow::bail!("Should not be any Point objects under an Area key"),
            })
            .collect::<anyhow::Result<geo::MultiPolygon>>()?;

        let areas = areas.buffer(delta);
        let areas = geo::unary_union(&areas);
        let areas = areas.buffer(-delta);

        let objects = areas
            .into_iter()
            .map(|p| MapObject::Area {
                object: p,
                symbol,
                tags: Default::default(),
            })
            .collect::<Vec<_>>();

        self.objects.insert(Symbol::Area(symbol), objects);

        Ok(())
    }

    /// Subtract all polygons under `exclusion` from every polygon under
    /// `target`, preserving the target symbol and object tags.
    pub fn subtract_area_symbol(
        &mut self,
        target: AreaSymbol,
        exclusion: AreaSymbol,
    ) -> crate::Result<()> {
        let Some(exclusions) = self.objects.get(&Symbol::Area(exclusion)) else {
            return Ok(());
        };
        let exclusions = exclusions
            .iter()
            .map(|object| match object {
                MapObject::Area { object, .. } => Ok(object.clone()),
                _ => anyhow::bail!("Should not be a non-area object under an Area key"),
            })
            .collect::<crate::Result<geo::MultiPolygon>>()?;
        if exclusions.0.is_empty() {
            return Ok(());
        }
        let Some(targets) = self.objects.get_mut(&Symbol::Area(target)) else {
            return Ok(());
        };
        let mut clipped = Vec::new();
        for object in targets.drain(..) {
            let MapObject::Area {
                object,
                symbol,
                tags,
            } = object
            else {
                anyhow::bail!("Should not be a non-area object under an Area key");
            };
            clipped.extend(object.difference(&exclusions).into_iter().map(|object| {
                MapObject::Area {
                    object,
                    symbol,
                    tags: tags.clone(),
                }
            }));
        }
        *targets = clipped;
        Ok(())
    }

    pub fn merge_and_filter_min_size(
        &mut self,
        symbols: impl IntoIterator<Item = AreaSymbol>,
    ) -> crate::Result<()> {
        let min_areas = symbols
            .into_iter()
            .map(|a| (a, a.min_size(&self.scale)))
            .collect::<Vec<_>>();

        for (symbol, min_area) in min_areas {
            self.merge_and_filter_symbol_min_size(symbol, min_area);
        }

        Ok(())
    }

    pub fn filter_area_min_size(&mut self, symbol: AreaSymbol, minimum_area_m2: f64) {
        if minimum_area_m2.is_finite() && minimum_area_m2 > 0. {
            self.merge_and_filter_symbol_min_size(symbol, minimum_area_m2);
        }
    }

    fn merge_and_filter_symbol_min_size(&mut self, symbol: AreaSymbol, min_area: f64) {
        let Some(map_objects) = self.objects.get_mut(&Symbol::Area(symbol)) else {
            return;
        };

        let mut areas = Vec::with_capacity(map_objects.len());
        let mut others = Vec::new();

        for map_object in map_objects.drain(..) {
            if let MapObject::Area {
                object,
                symbol,
                tags,
            } = map_object
            {
                areas.push(MergeArea {
                    object,
                    symbol,
                    tags,
                });
            } else {
                others.push(map_object);
            }
        }

        merge_small_areas(&mut areas, min_area);

        map_objects.extend(
            areas
                .into_iter()
                .filter(|area| area.abs_area() >= min_area)
                .map(MergeArea::into_map_object),
        );
        map_objects.extend(others);
    }

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
                            WeakAreaPathSymbol::try_from(
                                Symbol::Area(symbol)
                                    .get_omap_symbol(&omap.symbols)?
                                    .ok_or_else(|| omap::Error::MissingSymbolId)?
                                    .downgrade(),
                            )?,
                            geometry,
                        );
                        area.tags = tags;
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
                            WeakLinePathSymbol::try_from(
                                Symbol::Line(symbol)
                                    .get_omap_symbol(&omap.symbols)?
                                    .ok_or_else(|| omap::Error::MissingSymbolId)?
                                    .downgrade(),
                            )?,
                            geometry,
                        );
                        line.tags = tags;
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
                            .get_omap_symbol(&omap.symbols)?
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
                omap.parts.get_mut(0).unwrap().add_object(omap_object);
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
                    i += 1;
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

            let mut unclosed_object_groups =
                HashMap::<(Option<String>, bool), Vec<MergeLine>>::new();
            for unclosed_object in unclosed_objects {
                unclosed_object_groups
                    .entry((
                        unclosed_object.elevation_key(),
                        unclosed_object.requires_exact_contour_merge(),
                    ))
                    .or_default()
                    .push(unclosed_object);
            }

            for ((_, exact_contour_merge), mut unclosed_objects) in unclosed_object_groups {
                // Preserved contour geometry may only join at effectively
                // identical endpoints. This stitches exact tile cuts without
                // bridging a deliberately pruned form-line gap.
                let merge_delta = if exact_contour_merge { 1e-16 } else { delta };
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
                        && nn.distance_2(line_start) <= merge_delta
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

                    if (start.x - end.x).powi(2) + (start.y - end.y).powi(2) <= merge_delta {
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

fn merge_small_areas(areas: &mut Vec<MergeArea>, min_area: f64) {
    let mut active = vec![true; areas.len()];
    let mut candidate_lookup = small_area_merge_candidates(areas, min_area);

    while let Some((small_index, target_index)) =
        find_small_area_merge(areas, &active, &candidate_lookup, min_area)
    {
        let union = areas[target_index].object.union(&areas[small_index].object);
        if union.0.len() == 1 {
            areas[target_index].object = union.0.into_iter().next().expect("checked union length");
            active[small_index] = false;

            let absorbed_candidates = std::mem::take(&mut candidate_lookup[small_index]);
            candidate_lookup[target_index].extend(absorbed_candidates);
        }
    }

    let mut active = active.into_iter();
    areas.retain(|_| active.next().unwrap_or(false));
}

fn small_area_merge_candidates(areas: &[MergeArea], min_area: f64) -> Vec<Vec<usize>> {
    let indexed_polygons = areas
        .iter()
        .enumerate()
        .filter_map(|(index, area)| {
            Some(IndexedPolygonEnvelope {
                envelope: area.envelope()?,
                index,
            })
        })
        .collect::<Vec<_>>();

    let tree = RTree::bulk_load(indexed_polygons);
    let mut candidate_lookup = vec![Vec::new(); areas.len()];

    for (small_index, small_area) in areas.iter().enumerate() {
        let small_abs_area = small_area.abs_area();
        if small_abs_area >= min_area {
            continue;
        }

        let Some(envelope) = small_area.envelope() else {
            continue;
        };

        for candidate in tree.locate_in_envelope_intersecting(envelope) {
            if candidate.index == small_index {
                continue;
            }

            let candidate_area = areas[candidate.index].abs_area();
            if candidate_area < small_abs_area {
                continue;
            }

            if small_area.object.intersects(&areas[candidate.index].object)
                && small_area
                    .object
                    .union(&areas[candidate.index].object)
                    .0
                    .len()
                    == 1
            {
                candidate_lookup[small_index].push(candidate.index);
            }
        }
    }

    candidate_lookup
}

fn find_small_area_merge(
    areas: &[MergeArea],
    active: &[bool],
    candidate_lookup: &[Vec<usize>],
    min_area: f64,
) -> Option<(usize, usize)> {
    for (small_index, small_area) in areas.iter().enumerate() {
        if !active[small_index] {
            continue;
        }

        let small_abs_area = small_area.abs_area();
        if small_abs_area >= min_area {
            continue;
        }

        let mut best_target = None;
        let mut best_area = 0.;
        for &candidate_index in &candidate_lookup[small_index] {
            if !active[candidate_index] || candidate_index == small_index {
                continue;
            }

            let candidate_area = areas[candidate_index].abs_area();
            if candidate_area < small_abs_area || candidate_area <= best_area {
                continue;
            }

            if small_area.object.intersects(&areas[candidate_index].object)
                && small_area
                    .object
                    .union(&areas[candidate_index].object)
                    .0
                    .len()
                    == 1
            {
                best_area = candidate_area;
                best_target = Some(candidate_index);
            }
        }

        if let Some(target_index) = best_target {
            return Some((small_index, target_index));
        }
    }

    None
}

fn polygon_envelope(polygon: &geo::Polygon) -> Option<AABB<[f64; 2]>> {
    let rect = polygon.bounding_rect()?;
    Some(AABB::from_corners(
        [rect.min().x, rect.min().y],
        [rect.max().x, rect.max().y],
    ))
}

#[cfg(test)]
mod tests {
    use super::*;

    fn seam_stable_formline_at(points: Vec<geo::Coord>, elevation: f32) -> MapObject {
        let mut object = MapObject::Line {
            object: geo::LineString::new(points),
            symbol: LineSymbol::FormLine,
            tags: HashMap::new(),
        };
        object.add_elevation_tag(elevation);
        object.stabilize_contour_seam();
        object
    }

    fn seam_stable_formline(points: Vec<geo::Coord>) -> MapObject {
        seam_stable_formline_at(points, 2.5)
    }

    #[test]
    fn marsh_subtraction_removes_all_open_water_overlap() {
        let mut map = TempMap::new(geo::coord! { x: 0., y: 0. }, Scale::S15_000, None);
        let marsh = geo::Rect::new(geo::coord! { x: 0., y: 0. }, geo::coord! { x: 10., y: 10. })
            .to_polygon();
        let water = geo::Rect::new(geo::coord! { x: 5., y: 0. }, geo::coord! { x: 15., y: 10. })
            .to_polygon();
        map.add_object(MapObject::Area {
            object: marsh,
            symbol: AreaSymbol::Marsh,
            tags: HashMap::new(),
        });
        map.add_object(MapObject::Area {
            object: water.clone(),
            symbol: AreaSymbol::UncrossableWaterWithBankLine,
            tags: HashMap::new(),
        });

        map.subtract_area_symbol(AreaSymbol::Marsh, AreaSymbol::UncrossableWaterWithBankLine)
            .unwrap();

        let [MapObject::Area { object, .. }] =
            map.objects[&Symbol::Area(AreaSymbol::Marsh)].as_slice()
        else {
            panic!("expected one clipped marsh polygon");
        };
        assert_eq!(object.intersection(&water).unsigned_area(), 0.);
        assert!((object.unsigned_area() - 50.).abs() < 1e-9);
    }

    #[test]
    fn seam_stable_formlines_merge_only_at_identical_endpoints() {
        let mut map = TempMap::new(geo::coord! { x: 0., y: 0. }, Scale::S15_000, None);
        map.add_object(seam_stable_formline(vec![
            geo::coord! { x: 0., y: 0. },
            geo::coord! { x: 1., y: 0. },
        ]));
        map.add_object(seam_stable_formline(vec![
            geo::coord! { x: 1., y: 0. },
            geo::coord! { x: 2., y: 0. },
        ]));
        map.add_object(seam_stable_formline(vec![
            geo::coord! { x: 2.001, y: 0. },
            geo::coord! { x: 3., y: 0. },
        ]));

        map.merge_lines(10.);

        let lines = &map.objects[&Symbol::Line(LineSymbol::FormLine)];
        assert_eq!(lines.len(), 2);
        assert!(lines.iter().any(|object| {
            matches!(
                object,
                MapObject::Line { object, .. }
                    if object.0 == vec![
                        geo::coord! { x: 0., y: 0. },
                        geo::coord! { x: 1., y: 0. },
                        geo::coord! { x: 2., y: 0. },
                    ]
            )
        }));
    }

    #[test]
    fn exact_contour_stitching_is_independent_of_tile_order() {
        let stitch = |reverse: bool| {
            let mut segments = vec![
                vec![geo::coord! { x: 0., y: 0. }, geo::coord! { x: 1., y: 0. }],
                vec![geo::coord! { x: 1., y: 0. }, geo::coord! { x: 2., y: 0. }],
                vec![geo::coord! { x: 2., y: 0. }, geo::coord! { x: 3., y: 0. }],
            ];
            if reverse {
                segments.reverse();
            }
            let mut map = TempMap::new(geo::coord! { x: 0., y: 0. }, Scale::S15_000, None);
            for segment in segments {
                map.add_object(seam_stable_formline(segment));
            }
            map.merge_lines(10.);
            let [MapObject::Line { object, .. }] =
                map.objects[&Symbol::Line(LineSymbol::FormLine)].as_slice()
            else {
                panic!("expected one stitched line");
            };
            object.clone()
        };

        assert_eq!(stitch(false), stitch(true));
    }

    #[test]
    fn contour_merge_requires_exact_elevation_and_matching_orientation() {
        let line = |points, elevation| seam_stable_formline_at(points, elevation);

        let mut different_elevations =
            TempMap::new(geo::coord! { x: 0., y: 0. }, Scale::S15_000, None);
        different_elevations.add_object(line(
            vec![geo::coord! { x: 0., y: 0. }, geo::coord! { x: 1., y: 0. }],
            2.501,
        ));
        different_elevations.add_object(line(
            vec![geo::coord! { x: 1., y: 0. }, geo::coord! { x: 2., y: 0. }],
            2.504,
        ));
        different_elevations.merge_lines(10.);
        assert_eq!(
            different_elevations.objects[&Symbol::Line(LineSymbol::FormLine)].len(),
            2
        );

        let mut opposite_orientation =
            TempMap::new(geo::coord! { x: 0., y: 0. }, Scale::S15_000, None);
        opposite_orientation.add_object(line(
            vec![geo::coord! { x: 0., y: 0. }, geo::coord! { x: 1., y: 0. }],
            2.5,
        ));
        opposite_orientation.add_object(line(
            vec![geo::coord! { x: 2., y: 0. }, geo::coord! { x: 1., y: 0. }],
            2.5,
        ));
        opposite_orientation.merge_lines(10.);
        assert_eq!(
            opposite_orientation.objects[&Symbol::Line(LineSymbol::FormLine)].len(),
            2
        );
    }
}
