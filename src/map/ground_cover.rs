use geo::{BooleanOps, Intersects};

use super::{AreaSymbol, InternalMap, MapObject, Symbol};

/// Shared edges must not be moved independently by the OMAP curve fitter.
pub(super) const PRESERVE_AREA_GEOMETRY_TAG: &str = "omapmaker:preserve_area_geometry";

impl InternalMap {
    /// Resolve once, after tile stitching and minimum-area filtering. Snapshot
    /// the original shades so a 406+409 overlap is promoted exactly once.
    pub fn resolve_ground_cover_conflicts(&mut self) {
        let areas = |symbol| {
            let polygons = self
                .objects
                .get(&Symbol::Area(symbol))
                .into_iter()
                .flatten()
                .filter_map(|object| match object {
                    MapObject::Area { object, .. } => Some(object),
                    _ => None,
                });
            geo::unary_union(polygons)
        };
        let undergrowth = areas(AreaSymbol::Undergrowth);
        if undergrowth.0.is_empty() {
            return;
        }
        let fields = areas(AreaSymbol::BoulderField);
        let light = areas(AreaSymbol::LightGreen);
        let medium = areas(AreaSymbol::MediumGreen);
        let dark = areas(AreaSymbol::DarkGreen);
        let darker = medium.union(&dark);
        let greens = light.union(&darker);

        let promote_light = light.intersection(&undergrowth).difference(&darker);
        let promote_medium = medium.intersection(&undergrowth).difference(&dark);
        let rocky_undergrowth = fields.intersection(&undergrowth).difference(&greens);

        // Preserve both sides of the shared 409/green boundaries on export.
        for symbol in [
            AreaSymbol::LightGreen,
            AreaSymbol::MediumGreen,
            AreaSymbol::DarkGreen,
        ] {
            for object in self
                .objects
                .get_mut(&Symbol::Area(symbol))
                .into_iter()
                .flatten()
            {
                if let MapObject::Area { object, tags, .. } = object
                    && object.intersects(&undergrowth)
                {
                    tags.insert(PRESERVE_AREA_GEOMETRY_TAG.into(), "true".into());
                }
            }
        }
        for (polygons, symbol) in [
            (promote_light, AreaSymbol::MediumGreen),
            (promote_medium, AreaSymbol::DarkGreen),
            (rocky_undergrowth, AreaSymbol::LightGreen),
        ] {
            for object in polygons {
                self.add_object(MapObject::Area {
                    object,
                    symbol,
                    tags: [(PRESERVE_AREA_GEOMETRY_TAG.into(), "true".into())].into(),
                });
            }
        }
        let exclusions = greens.union(&fields);
        if let Some(objects) = self.objects.get_mut(&Symbol::Area(AreaSymbol::Undergrowth)) {
            let mut clipped = Vec::new();
            for object in objects.drain(..) {
                if let MapObject::Area {
                    object,
                    symbol,
                    tags,
                } = object
                {
                    clipped.extend(object.difference(&exclusions).into_iter().map(|object| {
                        MapObject::Area {
                            object,
                            symbol,
                            tags: tags.clone(),
                        }
                    }));
                }
            }
            *objects = clipped;
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::parameters::Scale;
    use geo::Area;

    fn rect(map: &mut InternalMap, symbol: AreaSymbol, x: f64, width: f64) {
        map.add_object(MapObject::Area {
            object: geo::Rect::new((x, 0.), (x + width, 10.)).to_polygon(),
            symbol,
            tags: Default::default(),
        });
    }

    fn areas(map: &InternalMap, symbol: AreaSymbol) -> geo::MultiPolygon {
        geo::unary_union(
            map.objects
                .get(&Symbol::Area(symbol))
                .into_iter()
                .flatten()
                .filter_map(|o| match o {
                    MapObject::Area { object, .. } => Some(object),
                    _ => None,
                }),
        )
    }

    #[test]
    fn undergrowth_promotes_each_shade_once_and_preserves_yellow() {
        let mut map = InternalMap::new(geo::Coord::zero(), Scale::S10_000, None);
        rect(&mut map, AreaSymbol::Undergrowth, 0., 50.);
        rect(&mut map, AreaSymbol::RoughOpenLand, 0., 50.);
        rect(&mut map, AreaSymbol::LightGreen, 10., 30.);
        rect(&mut map, AreaSymbol::MediumGreen, 20., 20.);
        rect(&mut map, AreaSymbol::DarkGreen, 30., 10.);
        map.resolve_ground_cover_conflicts();
        assert_eq!(areas(&map, AreaSymbol::Undergrowth).unsigned_area(), 200.);
        assert_eq!(areas(&map, AreaSymbol::RoughOpenLand).unsigned_area(), 500.);
        let medium = areas(&map, AreaSymbol::MediumGreen);
        assert_eq!(medium.unsigned_area(), 300.);
        let dark = areas(&map, AreaSymbol::DarkGreen);
        assert_eq!(dark.unsigned_area(), 200.);
        assert_eq!(
            dark.intersection(&geo::Rect::new((10., 0.), (20., 10.)).to_polygon())
                .unsigned_area(),
            0.
        );
        for shade in [
            AreaSymbol::LightGreen,
            AreaSymbol::MediumGreen,
            AreaSymbol::DarkGreen,
        ] {
            assert_eq!(
                areas(&map, shade)
                    .intersection(&areas(&map, AreaSymbol::Undergrowth))
                    .unsigned_area(),
                0.
            );
        }
        map.resolve_ground_cover_conflicts();
        assert_eq!(areas(&map, AreaSymbol::DarkGreen).unsigned_area(), 200.);
    }

    #[test]
    fn boulder_field_replaces_hatching_with_green_even_without_forest_shading() {
        let mut map = InternalMap::new(geo::Coord::zero(), Scale::S10_000, None);
        rect(&mut map, AreaSymbol::Undergrowth, 0., 40.);
        rect(&mut map, AreaSymbol::BoulderField, 0., 30.);
        rect(&mut map, AreaSymbol::LightGreen, 10., 10.);
        rect(&mut map, AreaSymbol::MediumGreen, 20., 10.);
        map.resolve_ground_cover_conflicts();
        assert_eq!(areas(&map, AreaSymbol::Undergrowth).unsigned_area(), 100.);
        assert_eq!(areas(&map, AreaSymbol::BoulderField).unsigned_area(), 300.);
        assert_eq!(areas(&map, AreaSymbol::LightGreen).unsigned_area(), 200.);
        assert_eq!(areas(&map, AreaSymbol::MediumGreen).unsigned_area(), 200.);
        assert_eq!(areas(&map, AreaSymbol::DarkGreen).unsigned_area(), 100.);
        assert_eq!(
            areas(&map, AreaSymbol::Undergrowth)
                .intersection(&areas(&map, AreaSymbol::BoulderField))
                .unsigned_area(),
            0.
        );
    }
}
