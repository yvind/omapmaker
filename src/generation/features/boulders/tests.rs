use super::*;
use super::{candidate::classify, evidence::is_last};
use crate::raster::DfmGrid;
use crate::{geometry::PointLaz, map::PointSymbol};
use las::point::Classification;
use las::{Bounds, Vector};

fn rock(height: f64, ground_classified: bool, slope: f64) -> (Dfm<Elevation>, PointCloud) {
    let grid = DfmGrid::new(81, 81, 0.5, geo::coord! { x: 0., y: 40. }).unwrap();
    let mut dem = Dfm::<Elevation>::new(grid);
    for y in 0..dem.height() {
        for x in 0..dem.width() {
            let p = dem.index2coord(y, x);
            dem[(y, x)] = (slope * p.x) as f32;
        }
    }
    let mut points = Vec::new();
    for y in 38..=42 {
        for x in 38..=42 {
            let p = dem.index2coord(y, x);
            // A compact rounded rock with several points on each cell.
            let rock_height =
                height * (1. - 0.12 * ((x as f64 - 40.).powi(2) + (y as f64 - 40.).powi(2)));
            if ground_classified {
                dem[(y, x)] += rock_height as f32;
            }
            for offset in [-0.08, 0.08] {
                let mut point =
                    PointLaz::new(p.x + offset, p.y, slope * (p.x + offset) + rock_height);
                if !ground_classified {
                    point.0.classification = Classification::Unclassified;
                }
                points.push(point);
            }
        }
    }
    add_ground(&dem, &mut points, (38, 42, 38, 42), slope);
    let bounds = Bounds {
        min: Vector {
            x: 0.,
            y: 0.,
            z: 0.,
        },
        max: Vector {
            x: 40.,
            y: 40.,
            z: 40.,
        },
    };
    (dem, PointCloud::new(points, bounds))
}

fn add_ground(
    dem: &Dfm<Elevation>,
    points: &mut Vec<PointLaz>,
    footprint: (usize, usize, usize, usize),
    slope: f64,
) {
    let (left, right, top, bottom) = footprint;
    for y in 26..=54 {
        for x in 26..=54 {
            if (left..=right).contains(&x) && (top..=bottom).contains(&y) {
                continue;
            }
            let p = dem.index2coord(y, x);
            points.push(PointLaz::new(p.x, p.y, slope * p.x));
        }
    }
}

fn block(half_width: usize, ground_classified: bool) -> (Dfm<Elevation>, PointCloud) {
    let (mut dem, mut cloud) = rock(3., false, 0.);
    cloud.points.clear();
    let left = 40 - half_width;
    let right = 40 + half_width;
    for y in left..=right {
        for x in left..=right {
            let p = dem.index2coord(y, x);
            if ground_classified {
                dem[(y, x)] = 3.;
            }
            for offset in [-0.08, 0.08] {
                let mut point = PointLaz::new(p.x + offset, p.y, 3.);
                if !ground_classified {
                    point.0.classification = Classification::Unclassified;
                }
                cloud.points.push(point);
            }
        }
    }
    add_ground(&dem, &mut cloud.points, (left, right, left, right), 0.);
    (dem, cloud)
}

// Keep the algorithm fixtures permissive so their return, surface and terrain
// checks are exercised independently. Production defaults are tested below.
fn permissive_parameters() -> BoulderParameters {
    BoulderParameters {
        maximum_height_to_diameter: 2.5,
        minimum_top_fraction: 0.25,
        maximum_nonterminal_fraction: 0.25,
        minimum_side_slope_degrees: 35.,
        ..Default::default()
    }
}

fn detect(dem: &Dfm<Elevation>, cloud: &PointCloud) -> Vec<MapObject> {
    detect_with_params(dem, cloud, &permissive_parameters())
}

fn detect_with_params(
    dem: &Dfm<Elevation>,
    cloud: &PointCloud,
    params: &BoulderParameters,
) -> Vec<MapObject> {
    let evidence = compute_boulder_evidence(cloud, dem);
    let hull = geo::Rect::new((0., 0.), (40., 40.)).to_polygon();
    boulder_objects(&evidence, cloud, dem, &hull, &hull, params)
}

fn point_result(objects: &[MapObject]) -> (geo::Point, PointSymbol, f64) {
    assert_eq!(
        objects.len(),
        1,
        "expected one point boulder, got {}",
        objects.len()
    );
    let MapObject::Point {
        object,
        symbol,
        tags,
        ..
    } = &objects[0]
    else {
        panic!("boulder detector emitted an area or line")
    };
    (*object, *symbol, tags["boulder_height_m"].parse().unwrap())
}

#[test]
fn conservative_defaults_reject_ambiguous_shapes_but_keep_broad_solid_blocks() {
    let params = BoulderParameters::default();
    for (dem, cloud) in [rock(3., false, 0.), block(2, false)] {
        assert_eq!(detect(&dem, &cloud).len(), 1);
        assert!(detect_with_params(&dem, &cloud, &params).is_empty());
    }

    let (dem, mut cloud) = block(3, false);
    assert_eq!(detect_with_params(&dem, &cloud, &params).len(), 1);
    // One passing echo per five last returns was tolerated by the older
    // settings, but is too much evidence of foliage for the new defaults.
    let surface = cloud
        .points
        .iter()
        .take_while(|point| point.0.z > 1.)
        .step_by(2)
        .cloned()
        .collect::<Vec<_>>();
    for mut point in surface {
        for offset in [-0.08, 0.08, 0.24] {
            let mut last = point.clone();
            last.0.x += offset;
            cloud.points.push(last);
        }
        point.0.return_number = 1;
        point.0.number_of_returns = 2;
        point.0.z += 0.1;
        cloud.points.push(point);
    }
    assert_eq!(detect(&dem, &cloud).len(), 1);
    assert!(detect_with_params(&dem, &cloud, &params).is_empty());
}

#[test]
fn top_coverage_cannot_be_increased_by_discarding_shoulders() {
    let (dem, mut cloud) = block(3, false);
    for (cell, pair) in cloud.points[..98].chunks_mut(2).enumerate() {
        if cell / 7 == 0 || (cell / 7 == 6 && (1..=5).contains(&(cell % 7))) {
            for point in pair {
                point.0.z = 2.;
            }
        }
    }
    // Isolate broadness from the abrupt shoulder's surface roughness.
    let mut params = BoulderParameters {
        maximum_surface_roughness_m: 3.,
        ..Default::default()
    };
    assert!(detect_with_params(&dem, &cloud, &params).is_empty());
    params.minimum_top_fraction = 0.6;
    let objects = detect_with_params(&dem, &cloud, &params);
    assert_eq!(point_result(&objects).1, PointSymbol::LargeBoulder);
    let MapObject::Point { tags, .. } = &objects[0] else {
        unreachable!()
    };
    let fraction = tags["boulder_top_fraction"].parse::<f64>().unwrap();
    assert!(fraction > 0.6 && fraction < 0.8);
}

#[test]
fn a_top_covering_exactly_eighty_percent_of_the_footprint_is_accepted() {
    let (dem, _) = block(2, false);
    let mut heights = std::collections::HashMap::new();
    for y in 38..42 {
        for x in 38..43 {
            let corner = (y == 38 || y == 41) && (x == 38 || x == 42);
            heights.insert(y * dem.width() + x, if corner { 2. } else { 3. });
        }
    }
    let params = BoulderParameters {
        maximum_surface_roughness_m: 3.,
        ..Default::default()
    };
    assert!(surface::shape(&heights, &dem, 3., &params).is_ok());
    heights.insert(40 * dem.width() + 40, 2.);
    assert!(matches!(
        surface::shape(&heights, &dem, 3., &params),
        Err(Rejection::Pointed)
    ));
}

#[test]
fn very_large_boulders_with_long_steep_sides_have_no_upper_size_limit() {
    let params = BoulderParameters::default();
    for (scale, rounded) in [(1, false), (1, true), (2, false), (2, true)] {
        let extent = 40. * scale as f64;
        let peak = 18. * scale as f64;
        let grid = DfmGrid::new(
            80 * scale + 1,
            80 * scale + 1,
            0.5,
            geo::coord! { x: 0., y: extent },
        )
        .unwrap();
        let mut dem = Dfm::<Elevation>::new(grid);
        dem.field.fill(0.);
        let mut points = Vec::new();
        for y in 0..dem.height() {
            for x in 0..dem.width() {
                let p = dem.index2coord(y, x);
                let radius = (p.x - extent * 0.5).abs().max((p.y - extent * 0.5).abs());
                // Broad flat or rounded top, then steep sides reaching ground
                // 4 or 8 m away: beyond the former fixed 3 m search limit.
                let dome = if rounded {
                    1. - 0.15 * (radius / (10. * scale as f64)).min(1.).powi(2)
                } else {
                    1.
                };
                let height = peak
                    * dome
                    * (1. - (radius - 10. * scale as f64).max(0.) / (4. * scale as f64)).max(0.);
                let mut point = PointLaz::new(p.x, p.y, height);
                point.0.classification = if height > 0. {
                    Classification::Unclassified
                } else {
                    Classification::Ground
                };
                points.push(point);
            }
        }
        let bounds = Bounds {
            min: Vector {
                x: 0.,
                y: 0.,
                z: 0.,
            },
            max: Vector {
                x: extent,
                y: extent,
                z: peak,
            },
        };
        let cloud = PointCloud::new(points, bounds);
        let evidence = compute_boulder_evidence(&cloud, &dem);
        let hull = geo::Rect::new((0., 0.), (extent, extent)).to_polygon();
        let objects = boulder_objects(&evidence, &cloud, &dem, &hull, &hull, &params);
        let (_, symbol, height) = point_result(&objects);
        assert_eq!(symbol, PointSymbol::LargeBoulder);
        assert!((height - peak).abs() < 0.01);
    }
}

#[test]
fn unclassified_boulders_are_detected_on_slopes() {
    for slope in [0., 0.125, -0.2] {
        let (dem, cloud) = rock(3., false, slope);
        let (point, symbol, height) = point_result(&detect(&dem, &cloud));
        assert_eq!(symbol, PointSymbol::LargeBoulder);
        assert!((point.x() - 20.).abs() < 0.6 && (point.y() - 20.).abs() < 0.6);
        assert!(height > 2. && height <= 3. + 1e-6);
    }
}

#[test]
fn ground_returns_never_propose_support_or_position_a_boulder() {
    let (dem, mut cloud) = rock(3., false, 0.);
    let expected = point_result(&detect(&dem, &cloud));
    let mut ground_spike = cloud.points[24].clone();
    ground_spike.0.classification = Classification::Ground;
    ground_spike.0.z = 8.;
    cloud.points.push(ground_spike);
    assert_eq!(point_result(&detect(&dem, &cloud)), expected);
    for point in &mut cloud.points {
        point.0.classification = Classification::Ground;
    }
    let evidence = compute_boulder_evidence(&cloud, &dem);
    assert!(evidence.layer_heights.is_empty());
    assert!(detect(&dem, &cloud).is_empty());
    let (dem, cloud) = rock(3., true, 0.);
    assert!(detect(&dem, &cloud).is_empty());
}

#[test]
fn large_symbol_includes_exactly_two_metres() {
    for (height, expected) in [(1.9, false), (2., true), (2.01, true), (4., true)] {
        let (dem, mut cloud) = rock(height, false, 0.);
        // A flat, multiply sampled top gives an exact threshold fixture.
        for point in &mut cloud.points[..50] {
            point.0.z = height;
        }
        let objects = detect(&dem, &cloud);
        assert_eq!(!objects.is_empty(), expected);
        if expected {
            assert_eq!(point_result(&objects).1, PointSymbol::LargeBoulder);
        }
    }
    let (dem, cloud) = rock(2.1, false, 0.);
    assert_eq!(
        point_result(&detect(&dem, &cloud)).1,
        PointSymbol::LargeBoulder
    );
}

#[test]
fn only_valid_last_returns_can_create_or_support_boulders() {
    let (dem, cloud) = rock(3., false, 0.);
    for (return_number, number_of_returns, expected) in [
        (1, 1, true),
        (2, 2, true),
        (1, 2, false),
        (2, 3, false),
        (0, 0, false),
        (3, 2, false),
    ] {
        let mut cloud = cloud.clone();
        for point in &mut cloud.points {
            point.0.return_number = return_number;
            point.0.number_of_returns = number_of_returns;
        }
        assert_eq!(
            !detect(&dem, &cloud).is_empty(),
            expected,
            "return {return_number}/{number_of_returns}"
        );
    }
    // A DEM bump also needs actual elevated last-return observations.
    let (dem, mut cloud) = rock(3., true, 0.);
    for point in &mut cloud.points {
        point.0.return_number = 1;
        point.0.number_of_returns = 2;
    }
    assert!(detect(&dem, &cloud).is_empty());
}

#[test]
fn multi_return_last_echoes_can_position_an_opaque_boulder() {
    let (dem, mut cloud) = rock(3., false, 0.);
    for point in &mut cloud.points {
        if (point.x() - 20.).abs() < 0.1 && point.y() == 20. {
            point.0.return_number = 2;
            point.0.number_of_returns = 2;
        }
    }
    let (position, symbol, _) = point_result(&detect(&dem, &cloud));
    assert_eq!(symbol, PointSymbol::LargeBoulder);
    assert!(cloud.points.iter().any(|point| point.x() == position.x()
        && point.y() == position.y()
        && point.0.return_number == 2
        && point.0.number_of_returns == 2));
}

#[test]
fn an_opaque_forest_boulder_can_be_entirely_multi_return_last_echoes() {
    let (dem, mut cloud) = rock(3., false, 0.);
    let expected = point_result(&detect(&dem, &cloud));
    let mut canopy = Vec::new();
    for point in &mut cloud.points[..50] {
        point.0.return_number = 2;
        point.0.number_of_returns = 2;
        let mut first = point.clone();
        first.0.return_number = 1;
        first.0.classification = Classification::HighVegetation;
        first.0.z = 12.;
        canopy.push(first);
    }
    cloud.points.extend(canopy);
    assert_eq!(point_result(&detect(&dem, &cloud)), expected);
}

#[test]
fn planar_unclassified_crown_with_last_returns_underneath_is_rejected() {
    let (dem, mut cloud) = rock(3., false, 0.);
    for point in &mut cloud.points[..50] {
        point.0.z = 3.;
    }
    assert_eq!(detect(&dem, &cloud).len(), 1);
    let beneath = cloud.points[..50]
        .iter()
        .map(|top| {
            let mut point = top.clone();
            point.0.z = 0.;
            point
        })
        .collect::<Vec<_>>();
    cloud.points.extend(beneath);
    assert!(detect(&dem, &cloud).is_empty());
}

#[test]
fn a_dense_opaque_broad_crown_is_rejected_by_surface_roughness() {
    let (dem, mut cloud) = rock(3., false, 0.);
    for (cell, pair) in cloud.points[..50].chunks_mut(2).enumerate() {
        let z = if (cell / 5 + cell % 5) % 2 == 0 {
            3.3
        } else {
            2.7
        };
        for point in pair {
            point.0.z = z;
        }
    }
    assert!(detect(&dem, &cloud).is_empty());
    let params = BoulderParameters {
        maximum_surface_roughness_m: 3.,
        ..permissive_parameters()
    };
    let evidence = compute_boulder_evidence(&cloud, &dem);
    let hull = geo::Rect::new((0., 0.), (40., 40.)).to_polygon();
    // The crown satisfies opacity, area, broad top, and steep sides.
    assert_eq!(
        boulder_objects(&evidence, &cloud, &dem, &hull, &hull, &params).len(),
        1
    );
}

#[test]
fn a_smooth_curved_boulder_with_small_measurement_noise_is_accepted() {
    let (dem, mut cloud) = rock(3., false, 0.);
    for (cell, pair) in cloud.points[..50].chunks_mut(2).enumerate() {
        let x = (cell % 5) as f64 - 2.;
        let y = (cell / 5) as f64 - 2.;
        let height = 3. * (1. - (x * x + y * y) * 0.25 / 2.25).sqrt();
        let noise = 0.02 * (cell as f64).sin();
        for point in pair {
            point.0.z = height + noise;
        }
    }
    assert_eq!(
        point_result(&detect(&dem, &cloud)).1,
        PointSymbol::LargeBoulder
    );
}

#[test]
fn a_smooth_opaque_pointed_crown_is_rejected_even_with_a_dense_tip() {
    let (dem, mut cloud) = rock(4., false, 0.);
    for pair in cloud.points[..50].chunks_mut(2) {
        let radius = (((pair[0].x() + pair[1].x()) * 0.5 - 20.).powi(2)
            + (pair[0].y() - 20.).powi(2))
        .sqrt();
        for point in pair {
            point.0.z = 4. - 2. * radius;
        }
    }
    assert!(detect(&dem, &cloud).is_empty());
    let params = BoulderParameters {
        minimum_top_fraction: 0.,
        maximum_surface_roughness_m: 3.,
        ..permissive_parameters()
    };
    let evidence = compute_boulder_evidence(&cloud, &dem);
    let hull = geo::Rect::new((0., 0.), (40., 40.)).to_polygon();
    assert_eq!(
        boulder_objects(&evidence, &cloud, &dem, &hull, &hull, &params).len(),
        1
    );
    for index in 0..100 {
        let mut tip = cloud.points[24].clone();
        tip.0.x += index as f64 * 0.001;
        cloud.points.push(tip);
    }
    assert!(detect(&dem, &cloud).is_empty());
}

#[test]
fn lower_last_returns_on_vertical_sides_are_not_penetration() {
    let (dem, mut cloud) = block(2, false);
    let mut sides = Vec::new();
    for top in &cloud.points[..50] {
        if (top.x() - 20.).abs() < 0.9 && (top.y() - 20.).abs() < 0.9 {
            continue;
        }
        for height in [0.25, 0.75, 1.25, 1.75, 2.25] {
            let mut side = top.clone();
            side.0.z = height;
            sides.push(side);
        }
    }
    cloud.points.extend(sides);
    assert_eq!(
        point_result(&detect(&dem, &cloud)).1,
        PointSymbol::LargeBoulder
    );
}

#[test]
fn a_ground_classified_block_stays_in_the_cliff_stage() {
    use crate::{
        map::{InternalMap, Symbol},
        parameters::{MapParameters, Scale},
    };
    let (dem, cloud) = block(4, true);
    let hull = geo::Rect::new((0., 0.), (40., 40.)).to_polygon();
    let params = MapParameters::default();
    let cliffs = crate::generation::features::compute_cliffs(
        &dem.slope(),
        &dem,
        &hull,
        &hull,
        &params,
        &params.geometry.cliffs.buffer_rules,
    );
    assert!(!cliffs.is_empty());
    let expected = cliffs.len();
    assert!(detect(&dem, &cloud).is_empty());
    let mut map = InternalMap::new(geo::coord! { x: 0., y: 0. }, Scale::S10_000, None);
    for object in cliffs.into_iter().chain(detect(&dem, &cloud)) {
        map.add_object(object);
    }
    map.filter_boulder_points(params.boulder.minimum_separation_m);
    assert!(
        map.objects
            .get(&Symbol::Point(PointSymbol::LargeBoulder))
            .is_none_or(Vec::is_empty)
    );
    assert_eq!(map.objects.values().map(Vec::len).sum::<usize>(), expected);
}

#[test]
fn narrow_crowns_are_rejected() {
    let (dem, mut cloud) = rock(3., false, 0.);
    cloud
        .points
        .retain(|point| (point.x() - 20.).abs() < 0.6 && (point.y() - 20.).abs() < 0.1);
    for point in &mut cloud.points {
        point.0.z = 3.;
    }
    assert!(detect(&dem, &cloud).is_empty());
}

#[test]
fn a_sparse_ring_without_an_observed_opaque_interior_is_rejected() {
    let (dem, mut cloud) = block(1, false);
    assert_eq!(detect(&dem, &cloud).len(), 1);
    cloud
        .points
        .retain(|point| (point.x() - 20.).abs() > 0.1 || point.y() != 20.);
    assert!(detect(&dem, &cloud).is_empty());
}

#[test]
fn broad_opaque_blocks_above_eight_square_metres_are_accepted() {
    for half_width in [4, 7] {
        let (dem, cloud) = block(half_width, false);
        assert_eq!(
            point_result(&detect(&dem, &cloud)).1,
            PointSymbol::LargeBoulder
        );
    }
}

#[test]
fn sides_must_reach_observed_normal_ground_around_the_block() {
    let (dem, mut cloud) = block(2, false);
    assert_eq!(detect(&dem, &cloud).len(), 1);
    let original = cloud.clone();
    cloud.points.retain(|point| point.0.z > 1.);
    assert!(
        detect(&dem, &cloud).is_empty(),
        "DEM interpolation is insufficient"
    );
    cloud = original.clone();
    for point in &mut cloud.points {
        if point.0.z == 0. {
            point.0.z = 1.;
        }
    }
    assert!(
        detect(&dem, &cloud).is_empty(),
        "an elevated shoulder is not ground"
    );
    cloud = original;
    cloud
        .points
        .retain(|point| point.0.z > 1. || point.x() > 20.);
    assert!(
        detect(&dem, &cloud).is_empty(),
        "one grounded side is insufficient"
    );
}

#[test]
fn a_gentle_non_ground_mound_is_rejected_by_its_sides() {
    let (dem, mut cloud) = block(2, false);
    cloud.points.clear();
    for y in 26..=54 {
        for x in 26..=54 {
            let p = dem.index2coord(y, x);
            let radius = (p.x - 20.).hypot(p.y - 20.);
            let height = (3. - 0.6 * (radius - 2.).max(0.)).max(0.);
            let mut point = PointLaz::new(p.x, p.y, height);
            point.0.classification = Classification::Unclassified;
            cloud.points.push(point);
        }
    }
    let params = permissive_parameters();
    let evidence = compute_boulder_evidence(&cloud, &dem);
    let response = evidence.filtered_surface(1.5, &params);
    let cells = (0..dem.field.len())
        .filter(|&cell| response[cell] >= 1.5)
        .collect::<Vec<_>>();
    assert!(matches!(
        classify(&cells, &response, &evidence, &cloud, &dem, &params),
        Err(Rejection::Sides)
    ));
    assert!(detect(&dem, &cloud).is_empty());
}

#[test]
fn intermediate_canopy_returns_cannot_change_height_or_placement() {
    let (dem, mut cloud) = rock(3., false, 0.);
    let expected = point_result(&detect(&dem, &cloud));
    for point in cloud.points.clone() {
        let mut canopy = point;
        canopy.0.z = 15.;
        canopy.0.classification = Classification::HighVegetation;
        canopy.0.return_number = 1;
        canopy.0.number_of_returns = 2;
        cloud.points.push(canopy);
    }
    assert_eq!(point_result(&detect(&dem, &cloud)), expected);
}

#[test]
fn tall_canopy_does_not_bridge_a_rock_into_a_tree_component() {
    let (dem, mut cloud) = rock(3., false, 0.);
    let expected = point_result(&detect(&dem, &cloud));
    for y in 35..=45 {
        for x in 35..=45 {
            if (38..=42).contains(&x) && (38..=42).contains(&y) {
                continue;
            }
            let p = dem.index2coord(y, x);
            let mut canopy = PointLaz::new(p.x, p.y, 11.);
            canopy.0.classification = Classification::HighVegetation;
            cloud.points.push(canopy);
        }
    }
    assert_eq!(point_result(&detect(&dem, &cloud)), expected);
}

#[test]
fn unclassified_penetrable_crowns_are_rejected() {
    let (dem, mut cloud) = rock(3., false, 0.);
    for point in &mut cloud.points {
        point.0.return_number = 3;
        point.0.number_of_returns = 3;
    }
    for last in cloud.points.clone() {
        for return_number in [1, 2] {
            let mut foliage = last.clone();
            foliage.0.return_number = return_number;
            foliage.0.z += 0.1 * f64::from(return_number);
            cloud.points.push(foliage);
        }
    }
    assert!(detect(&dem, &cloud).is_empty());
}

#[test]
fn buildings_water_noise_and_withheld_returns_are_rejected() {
    let (dem, cloud) = rock(3., false, 0.);
    for class in [
        Classification::Building,
        Classification::Water,
        Classification::HighNoise,
        Classification::LowPoint,
    ] {
        let mut cloud = cloud.clone();
        for point in &mut cloud.points {
            point.0.classification = class;
        }
        assert!(detect(&dem, &cloud).is_empty(), "class {class:?}");
    }
    let mut withheld = cloud;
    for point in &mut withheld.points {
        point.0.is_withheld = true;
    }
    assert!(detect(&dem, &withheld).is_empty());
}

#[test]
fn isolated_spikes_and_duplicate_observations_are_not_boulders() {
    let (dem, mut cloud) = rock(3., false, 0.);
    cloud.points.truncate(1);
    cloud.points = vec![cloud.points[0].clone(); 30];
    assert!(detect(&dem, &cloud).is_empty());
    cloud.points.push(cloud.points[0].clone());
    cloud.points.last_mut().unwrap().0.x += 0.5;
    assert!(detect(&dem, &cloud).is_empty());
}

#[test]
fn robust_height_and_position_ignore_an_extreme_return_and_point_order() {
    let (dem, mut cloud) = rock(3., false, 0.);
    let expected = point_result(&detect(&dem, &cloud));
    let mut spike = cloud.points[24].clone();
    spike.0.z = 100.;
    cloud.points.push(spike);
    let actual = point_result(&detect(&dem, &cloud));
    assert_eq!(actual.0, expected.0);
    assert_eq!(actual.1, PointSymbol::LargeBoulder);
    assert!((actual.2 - expected.2).abs() < 0.25);
    cloud.points.reverse();
    assert_eq!(point_result(&detect(&dem, &cloud)), actual);
}

#[test]
fn an_elongated_face_is_rejected() {
    let (dem, mut cloud) = rock(3., false, 0.);
    cloud.points.retain(|point| (point.y() - 20.).abs() < 0.1);
    for point in &mut cloud.points {
        point.0.x = 20. + (point.x() - 20.) * 3.;
    }
    assert!(detect(&dem, &cloud).is_empty());
}

#[test]
fn a_ground_supported_broad_knoll_is_rejected() {
    let (mut dem, mut cloud) = rock(1.5, true, 0.);
    cloud.points.clear();
    dem.field.fill(0.);
    for y in 32..=48 {
        for x in 32..=48 {
            let distance = ((x as f64 - 40.).powi(2) + (y as f64 - 40.).powi(2)).sqrt() * 0.5;
            let height = (1.5 * (1. - (distance / 4.).powi(2))).max(0.);
            dem[(y, x)] = height as f32;
            let p = dem.index2coord(y, x);
            cloud.points.push(PointLaz::new(p.x, p.y, height));
        }
    }
    assert!(detect(&dem, &cloud).is_empty());
}

#[test]
fn nearby_branches_do_not_veto_an_opaque_surface() {
    let (dem, original) = block(2, false);
    let expected = point_result(&detect(&dem, &original));
    for class in [Classification::HighVegetation, Classification::Unclassified] {
        for (return_number, number_of_returns) in [(1, 1), (1, 2), (2, 3)] {
            for height in [3.2, 4., 5.99, 6., 12.] {
                let mut cloud = original.clone();
                let mut canopy = PointLaz::new(20.02, 20., height);
                canopy.0.classification = class;
                canopy.0.return_number = return_number;
                canopy.0.number_of_returns = number_of_returns;
                cloud.points.push(canopy);
                let actual = point_result(&detect(&dem, &cloud));
                if number_of_returns == 1 && height == 3.2 {
                    // A terminal echo only 20 cm above the top cannot be
                    // distinguished from a rock measurement by its label.
                    // Keep the candidate within that measurement error.
                    assert_eq!(actual.1, expected.1);
                    assert!(
                        (actual.0.x() - expected.0.x()).hypot(actual.0.y() - expected.0.y()) < 0.5
                    );
                    assert!(actual.2 >= expected.2 && actual.2 <= expected.2 + 0.21);
                } else {
                    assert_eq!(
                        actual, expected,
                        "{class:?}, {return_number}/{number_of_returns}, height {height}"
                    );
                }
            }
        }
    }
}

#[test]
fn low_overhead_canopy_does_not_hide_a_forest_boulder() {
    let (dem, mut cloud) = rock(3., false, 0.);
    let expected = point_result(&detect(&dem, &cloud));
    let mut canopy = Vec::new();
    for point in &mut cloud.points[..50] {
        point.0.return_number = 2;
        point.0.number_of_returns = 2;
        let mut first = point.clone();
        first.0.z += 1.;
        first.0.return_number = 1;
        first.0.classification = Classification::HighVegetation;
        canopy.push(first);
    }
    cloud.points.extend(canopy);
    assert_eq!(point_result(&detect(&dem, &cloud)), expected);
}

#[test]
fn surface_return_opacity_is_configurable() {
    let (dem, mut cloud) = block(2, false);
    let passing = cloud.points[..50]
        .iter()
        .cloned()
        .map(|mut point| {
            point.0.return_number = 1;
            point.0.number_of_returns = 2;
            point.0.z += 0.15;
            point
        })
        .collect::<Vec<_>>();
    cloud.points.extend(passing);
    assert!(detect(&dem, &cloud).is_empty());
    let params = BoulderParameters {
        maximum_nonterminal_fraction: 0.6,
        ..permissive_parameters()
    };
    assert_eq!(detect_with_params(&dem, &cloud, &params).len(), 1);
}

#[test]
fn distant_unclassified_last_canopy_cannot_replace_or_bridge_the_lower_surface() {
    let (dem, mut cloud) = block(2, false);
    let expected = point_result(&detect(&dem, &cloud));
    for y in 34..=46 {
        for x in 34..=46 {
            let p = dem.index2coord(y, x);
            // More canopy than rock returns in every overlapped cell.
            for offset in [-0.12, -0.06, 0., 0.06, 0.12] {
                let mut canopy = PointLaz::new(p.x + offset, p.y, 12.);
                canopy.0.classification = Classification::Unclassified;
                cloud.points.push(canopy);
            }
        }
    }
    let result = detect(&dem, &cloud);
    assert_eq!(point_result(&result), expected);
    let MapObject::Point { tags, .. } = &result[0] else {
        unreachable!()
    };
    assert_eq!(tags["boulder_last_return_support"], "50");
    assert_eq!(tags["boulder_single_return_support"], "50");
    cloud.points.reverse();
    assert_eq!(point_result(&detect(&dem, &cloud)), expected);
}

#[test]
fn minimum_height_changes_acceptance_without_changing_symbol_height_rules() {
    let (dem, mut cloud) = block(2, false);
    let params = BoulderParameters {
        minimum_height_m: 1.,
        ..permissive_parameters()
    };
    for height in [1., 1.25, 2., 2.01] {
        for point in &mut cloud.points[..50] {
            point.0.z = height;
        }
        let result = detect_with_params(&dem, &cloud, &params);
        let (_, symbol, actual_height) = point_result(&result);
        assert!((actual_height - height).abs() < 1e-6);
        assert_eq!(
            symbol,
            if height >= 2. {
                PointSymbol::LargeBoulder
            } else {
                PointSymbol::SmallBoulder
            }
        );
        assert_eq!(!detect(&dem, &cloud).is_empty(), height >= 2.);
    }
    let params = BoulderParameters {
        minimum_height_m: 4.,
        ..permissive_parameters()
    };
    assert!(detect_with_params(&dem, &cloud, &params).is_empty());
}

#[test]
fn footprint_width_uses_observed_cells_and_has_no_hidden_eight_cell_floor() {
    let (dem, mut cloud) = block(1, false);
    cloud.points.retain(|point| point.0.z == 0.);
    for (x, y) in [(40, 40), (39, 40), (41, 40), (40, 39), (40, 41)] {
        let p = dem.index2coord(y, x);
        for offset in [-0.08, 0., 0.08] {
            let mut top = PointLaz::new(p.x + offset, p.y, 3.);
            top.0.classification = Classification::Unclassified;
            cloud.points.push(top);
        }
    }
    let mut params = BoulderParameters {
        maximum_height_to_diameter: 2.3,
        ..permissive_parameters()
    };
    // Five cells cover 1.25 m²; their equivalent width is 1.26 m, rather
    // than the 1.69 m width implied by filling their 3 × 3 bounding box.
    assert!(detect_with_params(&dem, &cloud, &params).is_empty());
    params.maximum_height_to_diameter = 2.4;
    assert_eq!(detect_with_params(&dem, &cloud, &params).len(), 1);
}

#[test]
fn single_and_multi_return_last_hits_both_count_as_support() {
    let (dem, mut cloud) = block(2, false);
    for point in &mut cloud.points[..25] {
        point.0.return_number = 3;
        point.0.number_of_returns = 3;
    }
    let objects = detect(&dem, &cloud);
    let MapObject::Point { tags, object, .. } = &objects[0] else {
        unreachable!()
    };
    assert_eq!(tags["boulder_last_return_support"], "50");
    assert_eq!(tags["boulder_single_return_support"], "25");
    assert!(
        cloud
            .points
            .iter()
            .any(|point| point.x() == object.x() && point.y() == object.y() && is_last(point))
    );
}

#[test]
fn neighboring_cut_tiles_emit_a_boundary_boulder_once() {
    let (dem, cloud) = rock(3., false, 0.);
    let params = permissive_parameters();
    let evidence = compute_boulder_evidence(&cloud, &dem);
    let hull = geo::Rect::new((0., 0.), (40., 40.)).to_polygon();
    let position = point_result(&detect(&dem, &cloud)).0;
    let cuts = [
        geo::Rect::new((0., 0.), (position.x(), 40.)),
        geo::Rect::new((position.x(), 0.), (40., 40.)),
    ];
    let objects = cuts
        .into_iter()
        .flat_map(|cut| boulder_objects(&evidence, &cloud, &dem, &hull, &cut.to_polygon(), &params))
        .collect::<Vec<_>>();
    assert_eq!(point_result(&objects).0, position);
}

#[test]
fn sparse_opaque_rock_with_six_returns_and_sampling_holes_is_detected() {
    let (dem, mut cloud) = block(1, false);
    cloud.points.retain(|point| point.0.z == 0.);
    for (x, y) in [(39, 39), (41, 39), (40, 40), (41, 40), (39, 41), (40, 41)] {
        let p = dem.index2coord(y, x);
        let mut point = PointLaz::new(p.x, p.y, 2.);
        point.0.classification = Classification::Unclassified;
        cloud.points.push(point);
    }
    let objects = detect(&dem, &cloud);
    assert_eq!(point_result(&objects).1, PointSymbol::LargeBoulder);
    let MapObject::Point { tags, .. } = &objects[0] else {
        unreachable!()
    };
    assert_eq!(tags["boulder_last_return_support"], "6");
    assert_eq!(tags["boulder_surface_area_m2"], "1.5");
}

#[test]
fn returns_to_ground_through_a_surface_hole_reject_a_crown() {
    let (dem, mut cloud) = block(2, false);
    cloud
        .points
        .retain(|point| point.x() < 19.9 || point.x() > 20.1 || point.y() != 20.);
    assert_eq!(
        detect(&dem, &cloud).len(),
        1,
        "an unobserved cell is a sampling hole"
    );
    cloud.points.push(PointLaz::new(20., 20., 0.));
    assert!(
        detect(&dem, &cloud).is_empty(),
        "observed ground reveals penetration"
    );
}

#[test]
fn boulders_below_one_metre_are_never_accepted() {
    let (dem, mut cloud) = block(2, false);
    let params = BoulderParameters {
        minimum_height_m: 0.1,
        ..permissive_parameters()
    };
    for point in &mut cloud.points[..50] {
        point.0.z = 0.99;
    }
    assert!(detect_with_params(&dem, &cloud, &params).is_empty());
    for point in &mut cloud.points[..50] {
        point.0.z = 1.;
    }
    assert_eq!(
        point_result(&detect_with_params(&dem, &cloud, &params)).1,
        PointSymbol::SmallBoulder
    );
}

#[test]
fn duplicated_solid_returns_cannot_dilute_passing_surface_echoes() {
    let (dem, mut cloud) = block(2, false);
    let passing = cloud.points[..50]
        .iter()
        .cloned()
        .map(|mut point| {
            point.0.return_number = 1;
            point.0.number_of_returns = 2;
            point.0.z += 0.1;
            point
        })
        .collect::<Vec<_>>();
    cloud.points.extend(passing);
    assert!(detect(&dem, &cloud).is_empty());
    let solid = cloud.points[..50].to_vec();
    for _ in 0..20 {
        cloud.points.extend(solid.clone());
    }
    assert!(detect(&dem, &cloud).is_empty());
}

#[test]
fn dense_top_sampling_cannot_hide_porous_interior_cells() {
    let (dem, mut cloud) = block(2, false);
    for (x, y) in [(39, 39), (41, 39), (39, 41), (41, 41)] {
        let p = dem.index2coord(y, x);
        cloud.points.push(PointLaz::new(p.x, p.y, 0.));
    }
    for index in 0..1000 {
        let mut top = PointLaz::new(20. + index as f64 * 0.0001, 20., 3.);
        top.0.classification = Classification::Unclassified;
        cloud.points.push(top);
    }
    assert!(detect(&dem, &cloud).is_empty());
}

#[test]
fn missing_ground_cells_cannot_propose_a_surface() {
    let (mut dem, cloud) = block(2, false);
    for value in [f32::MIN, f32::NAN, f32::INFINITY] {
        dem.field.fill(value);
        let evidence = compute_boulder_evidence(&cloud, &dem);
        assert!(evidence.layer_heights.is_empty());
        assert!(detect(&dem, &cloud).is_empty());
    }
}

#[test]
fn a_nearby_penetrable_crown_is_removed_before_grouping_the_rock() {
    let (dem, mut cloud) = block(2, false);
    let expected = point_result(&detect(&dem, &cloud));
    // Similar heights would join the crown to the rock if vegetation were
    // rejected only after connected components had already been grown.
    for y in 38..=42 {
        for x in 43..=47 {
            let p = dem.index2coord(y, x);
            for offset in [-0.08, 0.08] {
                let mut crown = PointLaz::new(p.x + offset, p.y, 4.);
                crown.0.classification = Classification::Unclassified;
                cloud.points.push(crown);
            }
        }
    }
    assert_eq!(point_result(&detect(&dem, &cloud)), expected);
}

#[test]
fn untrusted_labels_cannot_override_an_observed_opaque_rock() {
    let (dem, original) = rock(3., false, 0.);
    let expected = point_result(&detect(&dem, &original));
    for classification in [
        Classification::LowVegetation,
        Classification::MediumVegetation,
        Classification::HighVegetation,
    ] {
        let mut cloud = original.clone();
        for point in &mut cloud.points[..50] {
            point.0.classification = classification;
        }
        assert_eq!(point_result(&detect(&dem, &cloud)), expected);
        // The same label on an actually penetrable crown is rejected.
        for surface in original.points[..50].iter() {
            let mut ground = surface.clone();
            ground.0.z = 0.;
            ground.0.classification = Classification::Ground;
            cloud.points.push(ground);
        }
        assert!(detect(&dem, &cloud).is_empty());
    }
}

#[test]
fn dense_tree_skirts_connected_to_a_tall_crown_are_rejected() {
    for class in [Classification::Unclassified, Classification::HighVegetation] {
        let (dem, mut cloud) = block(2, false);
        for point in &mut cloud.points[..50] {
            point.0.classification = class;
        }
        // The low skirt alone looks exactly like a broad solid block.
        // Higher branches connect it to a tall crown throughout the full
        // vertical return profile.
        for y in 39..=41 {
            for x in 39..=41 {
                let p = dem.index2coord(y, x);
                for level in 0..30 {
                    let mut branch = PointLaz::new(p.x, p.y, 3.6 + level as f64 * 0.5);
                    branch.0.classification = class;
                    cloud.points.push(branch);
                }
            }
        }
        assert!(
            detect(&dem, &cloud).is_empty(),
            "tree skirt classified {class:?}"
        );
    }
}

#[test]
fn a_sparse_rounded_rock_keeps_its_single_sampled_peak() {
    let (dem, mut cloud) = block(1, false);
    cloud.points.retain(|point| point.0.z == 0.);
    for y in 39..=41 {
        for x in 39..=41 {
            let p = dem.index2coord(y, x);
            let height = if x == 40 && y == 40 { 2.3 } else { 1.8 };
            let mut point = PointLaz::new(p.x, p.y, height);
            point.0.classification = Classification::Unclassified;
            cloud.points.push(point);
        }
    }
    let (_, symbol, height) = point_result(&detect(&dem, &cloud));
    assert_eq!(symbol, PointSymbol::LargeBoulder);
    assert!((height - 2.3).abs() < 0.01);
}

#[test]
fn irregular_ground_sampling_around_a_rock_does_not_require_exact_rays() {
    let (dem, mut cloud) = block(2, false);
    cloud.points.retain(|point| point.0.z > 1.);
    for (x, y) in [(18.2, 18.3), (21.8, 18.2), (18.3, 21.8), (21.7, 21.6)] {
        cloud.points.push(PointLaz::new(x, y, 0.));
    }
    assert_eq!(
        point_result(&detect(&dem, &cloud)).1,
        PointSymbol::LargeBoulder
    );
}

#[test]
fn a_large_rounded_boulder_is_not_lost_to_the_building_detector() {
    use crate::{
        map::{InternalMap, Symbol},
        parameters::{MapParameters, Scale},
    };
    let (dem, mut cloud) = block(2, false);
    cloud.points.clear();
    for y in 26..=54 {
        for x in 26..=54 {
            let p = dem.index2coord(y, x);
            let radius = (p.x - 20.).hypot(p.y - 20.);
            if radius <= 2.2 {
                for offset in [-0.08, 0.08] {
                    let mut point = PointLaz::new(
                        p.x + offset,
                        p.y,
                        3. * (1. - radius.powi(2) / 2.5_f64.powi(2)).sqrt(),
                    );
                    point.0.classification = Classification::Unclassified;
                    cloud.points.push(point);
                }
            } else {
                cloud.points.push(PointLaz::new(p.x, p.y, 0.));
            }
        }
    }
    let params = MapParameters::default();
    let fit =
        crate::generation::features::compute_building_surface_fit(&cloud, &dem, &params.building)
            .unwrap();
    let detection = crate::generation::features::detect_buildings(&fit, &params.building);
    let hull = geo::Rect::new((0., 0.), (40., 40.)).to_polygon();
    let buildings = crate::generation::features::building_objects(
        &detection,
        &hull,
        &hull,
        &params.building,
        &params.geometry.buildings,
    );
    assert!(
        !buildings.is_empty(),
        "fixture must exercise a false building footprint"
    );
    let boulders = detect(&dem, &cloud);
    assert_eq!(boulders.len(), 1);
    let mut map = InternalMap::new(geo::coord! { x: 0., y: 0. }, Scale::S10_000, None);
    for object in buildings.into_iter().chain(boulders) {
        map.add_object(object);
    }
    map.filter_boulder_points(params.boulder.minimum_separation_m);
    assert_eq!(
        map.objects
            .get(&Symbol::Point(PointSymbol::LargeBoulder))
            .map_or(0, Vec::len),
        1
    );
    assert!(
        map.objects
            .get(&crate::map::AreaSymbol::Building.into())
            .is_none_or(Vec::is_empty)
    );
}

#[test]
fn flat_and_ridged_roofs_keep_building_priority() {
    use crate::{
        map::{AreaSymbol, InternalMap},
        parameters::{MapParameters, Scale},
    };
    for ridge in [false, true] {
        let (dem, mut cloud) = block(4, false);
        if ridge {
            for point in &mut cloud.points {
                if point.0.z > 1. {
                    point.0.z -= 0.25 * (point.x() - 20.).abs();
                }
            }
        }
        let params = MapParameters::default();
        let fit = crate::generation::features::compute_building_surface_fit(
            &cloud,
            &dem,
            &params.building,
        )
        .unwrap();
        let buildings = crate::generation::features::detect_buildings(&fit, &params.building);
        let hull = geo::Rect::new((0., 0.), (40., 40.)).to_polygon();
        let buildings = crate::generation::features::building_objects(
            &buildings,
            &hull,
            &hull,
            &params.building,
            &params.geometry.buildings,
        );
        assert!(!buildings.is_empty());
        let mut map = InternalMap::new(geo::coord! { x: 0., y: 0. }, Scale::S10_000, None);
        for object in buildings.into_iter().chain(detect(&dem, &cloud)) {
            map.add_object(object);
        }
        map.filter_boulder_points(params.boulder.minimum_separation_m);
        assert!(!map.objects[&AreaSymbol::Building.into()].is_empty());
        assert!(
            map.objects
                .get(&PointSymbol::LargeBoulder.into())
                .is_none_or(Vec::is_empty)
        );
    }
}

#[test]
fn rock_height_uses_the_dem_on_undulating_ground() {
    let (mut dem, mut cloud) = block(2, false);
    for y in 0..dem.height() {
        for x in 0..dem.width() {
            let p = dem.index2coord(y, x);
            dem[(y, x)] = (5. + 1.5 * ((p.x - 20.) / 2.).cos() * ((p.y - 20.) / 2.).cos()) as f32;
        }
    }
    for point in &mut cloud.points {
        let ground = evidence::ground_at(&dem, geo::coord! { x: point.x(), y: point.y() }).unwrap();
        point.0.z = f64::from(ground) + if point.0.z > 1. { 2.1 } else { 0. };
    }
    let (_, symbol, height) = point_result(&detect(&dem, &cloud));
    assert_eq!(symbol, PointSymbol::LargeBoulder);
    assert!((height - 2.1).abs() < 0.01);
}

#[test]
fn irregular_surface_sampling_does_not_require_collinear_triples() {
    let (dem, mut cloud) = block(2, false);
    cloud.points.retain(|point| point.0.z == 0.);
    for (x, y) in [
        (38, 38),
        (39, 38),
        (40, 39),
        (41, 39),
        (38, 40),
        (39, 40),
        (40, 41),
        (41, 41),
    ] {
        let p = dem.index2coord(y, x);
        let mut point = PointLaz::new(p.x, p.y, 2.4);
        point.0.classification = Classification::Unclassified;
        cloud.points.push(point);
    }
    let objects = detect(&dem, &cloud);
    assert_eq!(point_result(&objects).1, PointSymbol::LargeBoulder);
    let MapObject::Point { tags, .. } = &objects[0] else {
        unreachable!()
    };
    assert_eq!(tags["boulder_surface_roughness_m"], "unmeasured");
    assert_eq!(tags["boulder_dome_fraction"], "0");
}

#[test]
fn small_high_surface_fragments_require_boulder_proportions() {
    let (dem, mut cloud) = block(1, false);
    for point in &mut cloud.points {
        if point.0.z > 1. {
            point.0.z = 6.;
        }
    }
    assert!(detect(&dem, &cloud).is_empty());
    let params = BoulderParameters {
        maximum_height_to_diameter: 4.,
        ..permissive_parameters()
    };
    assert_eq!(detect_with_params(&dem, &cloud, &params).len(), 1);
}

fn boulder_objects(
    evidence: &BoulderEvidence,
    cloud: &PointCloud,
    dem: &Dfm<Elevation>,
    hull: &geo::Polygon,
    cut_overlay: &geo::Polygon,
    params: &BoulderParameters,
) -> Vec<MapObject> {
    detect_boulders(evidence, cloud, dem, hull, cut_overlay, params).objects
}
