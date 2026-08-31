mod detection;
mod regularization;
mod surface_fit;
mod vectorize;

use crate::raster::{
    BuildingCandidateId, BuildingProbability, Dfm, ElevatedPointCount, HeightAboveGroundMax,
    HeightAboveGroundMean, PlanarPointFraction, PlaneResidual, SurfaceNormalX, SurfaceNormalY,
    SurfaceNormalZ,
};

pub(crate) use detection::detect_buildings;
pub(crate) use surface_fit::compute_building_surface_fit;
pub(crate) use vectorize::building_objects;

const FIT_ELEVATION_CEILING_M: f64 = 100.;
const MINIMUM_RANSAC_SAMPLE_SIZE: usize = 3;

#[cfg(test)]
use crate::{
    geometry::PointCloud,
    map::{AreaSymbol, MapObject},
    parameters::{BuildingClassificationEvidence, BuildingParameters},
    raster::Elevation,
};
#[cfg(test)]
use las::point::Classification;

/// Candidate-local RANSAC diagnostics. The expensive fit cache tracks only
/// candidate discovery and plane-model parameters, not acceptance scoring.
pub struct BuildingSurfaceFit {
    pub height_mean: Dfm<HeightAboveGroundMean>,
    pub height_max: Dfm<HeightAboveGroundMax>,
    pub elevated_point_count: Dfm<ElevatedPointCount>,
    pub planar_point_fraction: Dfm<PlanarPointFraction>,
    pub plane_residual: Dfm<PlaneResidual>,
    pub normal_x: Dfm<SurfaceNormalX>,
    pub normal_y: Dfm<SurfaceNormalY>,
    pub normal_z: Dfm<SurfaceNormalZ>,
    vegetation_fraction: Box<[f32]>,
    class_6_fraction: Box<[f32]>,
}

/// Cheap threshold-dependent products. Candidate IDs are assigned in raster
/// order, making the result independent of Rayon thread count.
pub struct BuildingDetection {
    pub probability: Dfm<BuildingProbability>,
    pub candidate_id: Dfm<BuildingCandidateId>,
    /// Non-vegetation elevated candidates that lacked enough planar support.
    /// The independent cliff detector can confirm the rocky subset as boulders.
    pub plane_rejected_mask: Dfm<BuildingProbability>,
    accepted_mask: Dfm<BuildingProbability>,
}

impl BuildingDetection {
    /// Accepted building cells for hard exclusion in other feature detectors.
    pub fn accepted_mask(&self) -> &Dfm<BuildingProbability> {
        &self.accepted_mask
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::raster::DfmGrid;
    use las::{Bounds, Vector};

    fn bounds() -> Bounds {
        Bounds {
            min: Vector {
                x: 0.,
                y: 0.,
                z: 0.,
            },
            max: Vector {
                x: 30.,
                y: 30.,
                z: 20.,
            },
        }
    }

    fn synthetic_roof(reverse: bool) -> (Dfm<Elevation>, PointCloud) {
        let grid = DfmGrid::new(31, 31, 1., geo::coord! { x: 0., y: 30. }).unwrap();
        let mut dem = Dfm::<Elevation>::new(grid);
        dem.field.fill(0.);
        let mut points = Vec::new();
        for y in 8..=18 {
            for x in 5..=16 {
                for offset in [-0.2, 0.2] {
                    let z = 5. + 0.08 * x as f64 + 0.03 * y as f64;
                    let mut point =
                        crate::geometry::PointLaz::new(x as f64 + offset, 30. - y as f64, z);
                    point.0.return_number = 1;
                    point.0.number_of_returns = 1;
                    points.push(point);
                }
            }
        }
        // A similarly elevated but volume-like, multi-return tree patch.
        for y in 8..=14 {
            for x in 22..=26 {
                let canopy_top = 7. + ((x * 11 + y * 7) % 9) as f64 * 0.45;
                for (return_number, level) in [(1, 3.), (2, 6.), (3, canopy_top)] {
                    let mut point = crate::geometry::PointLaz::new(x as f64, 30. - y as f64, level);
                    point.0.number_of_returns = 3;
                    point.0.return_number = return_number;
                    points.push(point);
                }
            }
        }
        if reverse {
            points.reverse();
        }
        (dem, PointCloud::new(points, bounds()))
    }

    #[test]
    fn plane_fit_recovers_a_sloped_roof_and_rejects_volume_returns() {
        let (dem, cloud) = synthetic_roof(false);
        let params = BuildingParameters {
            minimum_building_area_m2: 10.,
            ..Default::default()
        };
        let fit = compute_building_surface_fit(&cloud, &dem, &params).unwrap();
        let roof = 12 * dem.width() + 10;
        let tree = 11 * dem.width() + 24;
        assert!(fit.plane_residual.field[roof] < 0.05);
        assert!(fit.normal_z.field[roof] > 0.98);
        assert_eq!(fit.elevated_point_count.field[tree], 0.);
    }

    #[test]
    fn detector_emits_one_building_and_is_input_order_deterministic() {
        let (dem, cloud) = synthetic_roof(false);
        let (_, reversed) = synthetic_roof(true);
        let mut params = crate::parameters::MapParameters::default();
        params.building.minimum_building_area_m2 = 20.;
        params.building.confidence_threshold = 0.6;
        let fit = compute_building_surface_fit(&cloud, &dem, &params.building).unwrap();
        let reversed_fit = compute_building_surface_fit(&reversed, &dem, &params.building).unwrap();
        let detection = detect_buildings(&fit, &params.building);
        let reversed_detection = detect_buildings(&reversed_fit, &params.building);
        assert_eq!(
            detection.candidate_id.field,
            reversed_detection.candidate_id.field
        );
        assert_eq!(
            detection.accepted_mask.field,
            reversed_detection.accepted_mask.field
        );

        let hull = geo::Rect::new(
            geo::coord! { x: -1., y: -1. },
            geo::coord! { x: 31., y: 31. },
        )
        .to_polygon();
        let objects = building_objects(
            &detection,
            &hull,
            &hull,
            &params.building,
            &params.geometry.buildings.buffer_rules,
        );
        assert_eq!(objects.len(), 1);
        assert!(matches!(
            objects[0],
            MapObject::Area {
                symbol: AreaSymbol::Building,
                ..
            }
        ));
    }

    #[test]
    fn authoritative_class_6_accepts_a_nonplanar_component() {
        let (dem, mut cloud) = synthetic_roof(false);
        for point in &mut cloud.points {
            if point.x() >= 22. {
                point.0.classification = Classification::Building;
            }
        }
        let ignored_params = BuildingParameters {
            minimum_building_area_m2: 10.,
            class_6_evidence: BuildingClassificationEvidence::Ignore,
            ..Default::default()
        };
        let fit = compute_building_surface_fit(&cloud, &dem, &ignored_params).unwrap();
        let ignored = detect_buildings(&fit, &ignored_params);
        let authoritative_params = BuildingParameters {
            class_6_evidence: BuildingClassificationEvidence::Authoritative,
            ..ignored_params
        };
        let authoritative_fit =
            compute_building_surface_fit(&cloud, &dem, &authoritative_params).unwrap();
        let authoritative = detect_buildings(&authoritative_fit, &authoritative_params);

        let tree = 11 * dem.width() + 24;
        assert_eq!(ignored.accepted_mask.field[tree], 0.);
        assert_eq!(authoritative.accepted_mask.field[tree], 1.);
    }

    #[test]
    fn ransac_accepts_a_two_plane_roof_and_ignores_non_last_returns() {
        let grid = DfmGrid::new(31, 31, 1., geo::coord! { x: 0., y: 30. }).unwrap();
        let mut dem = Dfm::<Elevation>::new(grid);
        dem.field.fill(0.);
        let mut points = Vec::new();
        for y in 8..=18 {
            for x in 5..=19 {
                for offset in [-0.2, 0.2] {
                    let ridge_distance = (x as f64 - 12.).abs();
                    let mut roof = crate::geometry::PointLaz::new(
                        x as f64 + offset,
                        30. - y as f64,
                        7. - 0.18 * ridge_distance,
                    );
                    roof.0.return_number = 1;
                    roof.0.number_of_returns = 1;
                    points.push(roof);
                }

                // Neither point may influence the fit: the first is not a last
                // return and the last lies below the roof-height threshold.
                let mut canopy_first = crate::geometry::PointLaz::new(
                    x as f64,
                    30. - y as f64,
                    12. + ((x + y) % 5) as f64,
                );
                canopy_first.0.return_number = 1;
                canopy_first.0.number_of_returns = 2;
                points.push(canopy_first);
                let mut canopy_last = crate::geometry::PointLaz::new(x as f64, 30. - y as f64, 1.);
                canopy_last.0.return_number = 2;
                canopy_last.0.number_of_returns = 2;
                points.push(canopy_last);
            }
        }
        let cloud = PointCloud::new(points, bounds());
        let params = BuildingParameters {
            maximum_plane_residual_m: 0.05,
            minimum_planar_point_fraction: 0.9,
            minimum_building_area_m2: 40.,
            confidence_threshold: 0.55,
            ..Default::default()
        };
        let fit = compute_building_surface_fit(&cloud, &dem, &params).unwrap();
        let center = 12 * dem.width() + 12;
        assert!(fit.planar_point_fraction.field[center] > 0.95);
        assert!(
            fit.plane_residual.field[center] < 0.03,
            "residual was {}",
            fit.plane_residual.field[center]
        );
        let detection = detect_buildings(&fit, &params);
        assert_eq!(detection.accepted_mask.field[center], 1.);
    }

    #[test]
    fn nonplanar_candidate_is_exposed_for_boulder_review_but_trees_are_not() {
        let grid = DfmGrid::new(31, 31, 1., geo::coord! { x: 0., y: 30. }).unwrap();
        let mut dem = Dfm::<Elevation>::new(grid);
        dem.field.fill(0.);
        let make_cloud = |vegetation: bool| {
            let mut points = Vec::new();
            for y in 8..=16 {
                for x in 8..=16 {
                    let z = 4. + ((x * 17 + y * 31) % 11) as f64 * 0.23;
                    let mut point = crate::geometry::PointLaz::new(x as f64, 30. - y as f64, z);
                    point.0.return_number = 1;
                    point.0.number_of_returns = 1;
                    if vegetation {
                        point.0.classification = Classification::HighVegetation;
                    }
                    points.push(point);
                }
            }
            PointCloud::new(points, bounds())
        };
        let params = BuildingParameters {
            maximum_plane_residual_m: 0.02,
            minimum_planar_point_fraction: 0.8,
            minimum_building_area_m2: 20.,
            minimum_plane_inliers: 10,
            maximum_roof_planes: 1,
            ..Default::default()
        };
        let center = 12 * dem.width() + 12;

        let fit = compute_building_surface_fit(&make_cloud(false), &dem, &params).unwrap();
        let detection = detect_buildings(&fit, &params);
        assert_eq!(detection.accepted_mask.field[center], 0.);
        assert_eq!(detection.plane_rejected_mask.field[center], 1.);

        let tree_fit = compute_building_surface_fit(&make_cloud(true), &dem, &params).unwrap();
        let tree_detection = detect_buildings(&tree_fit, &params);
        assert_eq!(tree_detection.accepted_mask.field[center], 0.);
        assert_eq!(tree_detection.plane_rejected_mask.field[center], 0.);
    }

    #[test]
    fn raster_stair_steps_regularize_to_the_roof_direction() {
        let grid = DfmGrid::new(61, 61, 0.5, geo::coord! { x: 0., y: 15. }).unwrap();
        let mut probability = Dfm::<BuildingProbability>::new(grid.clone());
        let mut candidate_id = Dfm::<BuildingCandidateId>::new(grid.clone());
        let mut accepted_mask = Dfm::<BuildingProbability>::new(grid);
        probability.field.fill(0.);
        candidate_id.field.fill(0.);
        accepted_mask.field.fill(0.);

        let roof_angle = 17_f64.to_radians();
        for y in 0..accepted_mask.height() {
            for x in 0..accepted_mask.width() {
                let coordinate = accepted_mask.index2coord(y, x);
                let dx = coordinate.x - 15.;
                let dy = coordinate.y;
                let along = dx * roof_angle.cos() + dy * roof_angle.sin();
                let across = -dx * roof_angle.sin() + dy * roof_angle.cos();
                if along.abs() <= 6. && across.abs() <= 3. {
                    let index = y * accepted_mask.width() + x;
                    probability.field[index] = 1.;
                    candidate_id.field[index] = 1.;
                    accepted_mask.field[index] = 1.;
                }
            }
        }
        let mut plane_rejected_mask = Dfm::<BuildingProbability>::new_like(&accepted_mask);
        plane_rejected_mask.field.fill(0.);
        let detection = BuildingDetection {
            probability,
            candidate_id,
            accepted_mask,
            plane_rejected_mask,
        };
        let hull = geo::Rect::new(
            geo::coord! { x: -1., y: -16. },
            geo::coord! { x: 31., y: 16. },
        )
        .to_polygon();
        let parameters = BuildingParameters {
            regularization_simplification_tolerance_m: 0.6,
            regularization_maximum_boundary_displacement_m: 1.,
            regularization_minimum_iou: 0.75,
            ..Default::default()
        };

        let objects = building_objects(&detection, &hull, &hull, &parameters, &[]);
        assert_eq!(objects.len(), 1);
        let MapObject::Area { object, .. } = &objects[0] else {
            panic!("building detector emitted a non-area object");
        };
        let longest_edge = object
            .exterior()
            .lines()
            .max_by(|first, second| {
                first
                    .dx()
                    .hypot(first.dy())
                    .total_cmp(&second.dx().hypot(second.dy()))
            })
            .unwrap();
        let regularized_direction = longest_edge
            .dy()
            .atan2(longest_edge.dx())
            .rem_euclid(std::f64::consts::FRAC_PI_2);
        assert!((regularized_direction - roof_angle).abs().to_degrees() < 2.);
    }
}
