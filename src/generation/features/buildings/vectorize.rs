use std::collections::HashMap;

use geo::{Area, BooleanOps, Simplify};

use crate::{
    geometry::MapMultiPolygon,
    map::{AreaSymbol, MapObject},
    parameters::{BuildingGeometryParameters, BuildingParameters},
};

use super::{BuildingDetection, regularization};

pub fn building_objects(
    detection: &BuildingDetection,
    convex_hull: &geo::Polygon,
    cut_overlay: &geo::Polygon,
    parameters: &BuildingParameters,
    geometry_parameters: &BuildingGeometryParameters,
) -> Vec<MapObject> {
    debug_assert!(
        detection
            .candidate_id
            .grid
            .same_layout(&detection.accepted_mask.grid)
            && detection
                .plane_rejected_mask
                .grid
                .same_layout(&detection.accepted_mask.grid),
        "building diagnostics must use one grid"
    );
    let contours = detection.accepted_mask.marching_squares(0.5);
    let mut traced = geo::MultiPolygon::from_contours(contours, convex_hull, false);
    for rule in &geometry_parameters.buffer_rules {
        traced = traced.apply_buffer_rule(rule);
    }
    traced = close_small_holes(traced, geometry_parameters.maximum_hole_area_m2);
    let mut polygons = geo::MultiPolygon::new(
        traced
            .into_iter()
            .map(|polygon| regularization::regularize_building_footprint(&polygon, parameters))
            .collect(),
    )
    .simplify(crate::SIMPLIFICATION_DIST);
    polygons = cut_overlay.intersection(&polygons);
    polygons
        .into_iter()
        .filter(|polygon| polygon.unsigned_area() > 0.)
        .map(|polygon| MapObject::Area {
            object: polygon,
            symbol: AreaSymbol::Building,
            tags: HashMap::new(),
        })
        .collect()
}

/// Close small gaps after buffering so footprint regularization only sees the
/// rings that should remain in the final building geometry.
fn close_small_holes(polygons: geo::MultiPolygon, maximum_hole_area_m2: f64) -> geo::MultiPolygon {
    if !maximum_hole_area_m2.is_finite() || maximum_hole_area_m2 <= 0. {
        return polygons;
    }

    geo::MultiPolygon::new(
        polygons
            .into_iter()
            .map(|polygon| {
                let (exterior, interiors) = polygon.into_inner();
                let interiors = interiors
                    .into_iter()
                    .filter(|interior| {
                        geo::Polygon::new(interior.clone(), Vec::new()).unsigned_area()
                            > maximum_hole_area_m2
                    })
                    .collect();
                geo::Polygon::new(exterior, interiors)
            })
            .collect(),
    )
}

#[cfg(test)]
mod tests {
    use geo::{Area, polygon};

    use super::close_small_holes;

    #[test]
    fn closes_only_building_holes_within_the_configured_area() {
        let small_hole = polygon![
            (x: 1., y: 1.),
            (x: 2., y: 1.),
            (x: 2., y: 2.),
            (x: 1., y: 2.),
        ];
        let large_hole = polygon![
            (x: 4., y: 4.),
            (x: 7., y: 4.),
            (x: 7., y: 7.),
            (x: 4., y: 7.),
        ];
        let source = geo::MultiPolygon::new(vec![geo::Polygon::new(
            polygon![
                (x: 0., y: 0.),
                (x: 10., y: 0.),
                (x: 10., y: 10.),
                (x: 0., y: 10.),
            ]
            .exterior()
            .clone(),
            vec![small_hole.exterior().clone(), large_hole.exterior().clone()],
        )]);

        assert_eq!(close_small_holes(source.clone(), 0.), source);
        let cleaned = close_small_holes(source, 4.);
        assert_eq!(cleaned.0[0].interiors(), &[large_hole.exterior().clone()]);
        assert_eq!(cleaned.unsigned_area(), 91.);
    }
}
