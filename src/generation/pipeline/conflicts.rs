use crate::map::{AreaSymbol, MapObject};
use geo::{Area, BooleanOps};

pub(super) fn resolve_building_conflicts(objects: &mut Vec<MapObject>) {
    let footprints = geo::MultiPolygon::new(
        objects
            .iter()
            .filter_map(|object| match object {
                MapObject::Area {
                    object,
                    symbol: AreaSymbol::Building,
                    ..
                } => Some(object.clone()),
                _ => None,
            })
            .collect(),
    );
    if footprints.0.is_empty() {
        return;
    }

    let mut resolved = Vec::with_capacity(objects.len());
    for object in objects.drain(..) {
        match object {
            MapObject::Area {
                object,
                symbol:
                    symbol @ (AreaSymbol::RoughOpenLand
                    | AreaSymbol::LightGreen
                    | AreaSymbol::MediumGreen
                    | AreaSymbol::DarkGreen
                    | AreaSymbol::Marsh
                    | AreaSymbol::PavedAreaWithBoundary),
                tags,
            } => resolved.extend(object.difference(&footprints).into_iter().map(|object| {
                MapObject::Area {
                    object,
                    symbol,
                    tags: tags.clone(),
                }
            })),
            MapObject::Area {
                object,
                symbol: AreaSymbol::GiganticBoulder,
                tags,
            } => {
                let overlap = object.intersection(&footprints).unsigned_area();
                if overlap < object.unsigned_area() * 0.95 {
                    resolved.push(MapObject::Area {
                        object,
                        symbol: AreaSymbol::GiganticBoulder,
                        tags,
                    });
                }
            }
            MapObject::Area {
                object,
                symbol: AreaSymbol::UncrossableWaterWithBankLine,
                tags,
            } => {
                let overlap = object.intersection(&footprints).unsigned_area();
                if overlap > 1. && overlap > object.unsigned_area() * 0.25 {
                    log::warn!(
                        "A detected building overlaps {:.1} m² of mapped water; keeping both for review",
                        overlap
                    );
                }
                resolved.push(MapObject::Area {
                    object,
                    symbol: AreaSymbol::UncrossableWaterWithBankLine,
                    tags,
                });
            }
            _ => resolved.push(object),
        }
    }
    *objects = resolved;
}
