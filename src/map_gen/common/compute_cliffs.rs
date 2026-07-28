use std::collections::HashMap;

use crate::{
    geometry::MapMultiPolygon,
    map_gen::egui_map::{AreaSymbol, LineSymbol, MapObject},
    parameters::{BufferRule, MapParameters},
    raster::{Dfm, dfm::Slope},
};

use geo::{BooleanOps, Simplify};

pub fn compute_cliffs(
    slope: &Dfm<Slope>,
    convex_hull: &geo::Polygon,
    cut_overlay: &geo::Polygon,
    params: &MapParameters,
    buffer_rules: &[BufferRule],
) -> Vec<MapObject> {
    let cliff_contours = slope.marching_squares(params.cliff.cliff);

    let mut cliff_polygons = geo::MultiPolygon::from_contours(cliff_contours, convex_hull, false);

    cliff_polygons = cliff_polygons.simplify(crate::SIMPLIFICATION_DIST);

    for buffer in buffer_rules.iter() {
        cliff_polygons = cliff_polygons.apply_buffer_rule(buffer);
    }

    cliff_polygons = cut_overlay.intersection(&cliff_polygons);

    let (small_cliff_lines, large_cliff_lines, cliff_polygons) = if params.cliff.collapse {
        let (mut small_cliff_lines, mut cliff_polygons) =
            cliff_polygons.collapse(params.cliff.collapse_amount_small_cliff as f64);
        small_cliff_lines =
            small_cliff_lines.simplify(params.cliff.collapse_amount_small_cliff as f64 / 4.);
        cliff_polygons = cliff_polygons.simplify(crate::SIMPLIFICATION_DIST);

        let (mut large_cliff_lines, mut cliff_polygons) =
            cliff_polygons.collapse(params.cliff.collapse_amount_large_cliff as f64);
        large_cliff_lines =
            large_cliff_lines.simplify(params.cliff.collapse_amount_large_cliff as f64 / 4.);

        cliff_polygons = cliff_polygons.simplify(crate::SIMPLIFICATION_DIST);
        (small_cliff_lines, large_cliff_lines, cliff_polygons)
    } else {
        (
            geo::MultiLineString::empty(),
            geo::MultiLineString::empty(),
            cliff_polygons,
        )
    };

    let num_polys = cliff_polygons.0.len();
    let num_lines = small_cliff_lines.0.len() + large_cliff_lines.0.len();

    let mut objects = Vec::with_capacity(num_polys + num_lines);

    for polygon in cliff_polygons.into_iter() {
        let cliff_object = MapObject::Area {
            object: polygon,
            symbol: AreaSymbol::GiganticBoulder,
            tags: HashMap::new(),
        };
        objects.push(cliff_object);
    }

    for line in small_cliff_lines.into_iter() {
        let cliff_object = MapObject::Line {
            object: line,
            symbol: LineSymbol::Cliff,
            tags: HashMap::new(),
        };
        objects.push(cliff_object);
    }

    for line in large_cliff_lines.into_iter() {
        let cliff_object = MapObject::Line {
            object: line,
            symbol: LineSymbol::ImpassableCliff,
            tags: HashMap::new(),
        };
        objects.push(cliff_object);
    }

    objects
}
