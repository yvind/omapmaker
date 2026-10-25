use super::*;

// Local data is deliberately opt-in. No source lidar is changed.
#[test]
#[ignore = "requires local reference COPCs; writes diagnostic CSVs under /tmp/omap-ground-cover"]
fn inspect_reference_sites() -> crate::Result<()> {
    use crate::{
        geometry::PointLaz,
        raster::{DfmGrid, GroundRelief2m},
    };
    use spade::{DelaunayTriangulation, Triangulation};
    use std::io::Write;
    std::fs::create_dir_all("/tmp/omap-ground-cover")?;
    let sites = [
        (
            "sundsvall-funnel",
            "sundsvall/sundsvall.copc.laz",
            Some((17.2954, 62.6244)),
            "3006",
        ),
        (
            "sundsvall-stream",
            "sundsvall/sundsvall.copc.laz",
            Some((17.3011, 62.6314)),
            "3006",
        ),
        (
            "sundsvall-field",
            "sundsvall/sundsvall.copc.laz",
            Some((17.2710, 62.6420)),
            "3006",
        ),
        (
            "sundsvall-rocks",
            "sundsvall/sundsvall.copc.laz",
            Some((17.2643, 62.6285)),
            "3006",
        ),
        ("tunhovd", "Tunhovd/32-1-497-146-60.copc.laz", None, "25832"),
        (
            "tunhovd-north",
            "Tunhovd/32-1-497-146-61.copc.laz",
            None,
            "25832",
        ),
        (
            "tunhovd-west",
            "Tunhovd/32-1-497-146-50.copc.laz",
            None,
            "25832",
        ),
        (
            "tunhovd-northwest",
            "Tunhovd/32-1-497-146-51.copc.laz",
            None,
            "25832",
        ),
        (
            "tunhovd-north2",
            "Tunhovd/32-1-497-146-62.copc.laz",
            None,
            "25832",
        ),
        (
            "tunhovd-west2",
            "Tunhovd/32-1-497-146-40.copc.laz",
            None,
            "25832",
        ),
        (
            "sandhammaren",
            "sandhammaren/sandhammaren.copc.laz",
            None,
            "3006",
        ),
        (
            "flatinmarka-west",
            "flatinmarka/32-1-501-142-54.copc.laz",
            None,
            "25832",
        ),
        (
            "flatinmarka-east",
            "flatinmarka/32-1-501-142-76.copc.laz",
            None,
            "25832",
        ),
        (
            "sandhammaren-forest",
            "sandhammaren/sandhammaren.copc.laz",
            None,
            "3006",
        ),
        (
            "sandhammaren-north",
            "sandhammaren/m25a002-614_45.copc.laz",
            None,
            "3006",
        ),
        (
            "flatinmarka",
            "flatinmarka/32-1-501-142-65.copc.laz",
            None,
            "25832",
        ),
    ];
    let mut summary =
        std::io::BufWriter::new(std::fs::File::create("/tmp/omap-ground-cover/summary.csv")?);
    writeln!(
        summary,
        "site,easting,northing,undergrowth_m2,boulder_field_m2,green_from_overlap_m2"
    )?;
    for (name, path, ll, crs) in sites {
        let mut reader = las::CopcReader::from_path(format!("../lidar/{path}"))?;
        let header = reader.header().bounds();
        let (mut cx, mut cy) = if let Some((lon, lat)) = ll {
            let t = proj_core::Transform::from_horizontal_components(
                &proj_wkt::parse_crs("4326")?,
                &proj_wkt::parse_crs(crs)?,
            )?;
            let p: geo::Point = t.convert_geometry(geo::Point::new(lon, lat))?;
            (p.x(), p.y())
        } else {
            (
                (header.min.x + header.max.x) / 2.,
                if name == "tunhovd" {
                    header.max.y - 140.
                } else {
                    (header.min.y + header.max.y) / 2.
                },
            )
        };
        if name == "sandhammaren" {
            cy += 1000.;
        }
        if name == "sandhammaren-forest" || name == "sandhammaren-north" {
            // Pick a reproducible mixed ground/non-ground land sample instead
            // of the source rectangle's midpoint, which can be in the sea.
            let mut buckets = std::collections::BTreeMap::<(i64, i64), (usize, usize)>::new();
            for point in reader
                .query(
                    las::LodSelection::LevelMinMax(0, 5),
                    las::BoundsSelection::All,
                )?
                .points()
            {
                let p = point?;
                if p.is_withheld
                    || matches!(
                        p.classification,
                        Classification::Water
                            | Classification::LowPoint
                            | Classification::HighNoise
                            | Classification::Building
                    )
                {
                    continue;
                }
                let counts = buckets
                    .entry(((p.x / 250.).floor() as i64, (p.y / 250.).floor() as i64))
                    .or_default();
                if p.classification == Classification::Ground {
                    counts.0 += 1;
                } else {
                    counts.1 += 1;
                }
            }
            let (&(bx, by), _) = buckets
                .iter()
                .filter(|(_, counts)| counts.0 >= 5)
                .max_by_key(|(key, (ground, other))| (ground.min(other), ground + other, **key))
                .expect("source has ground-supported land");
            cx = bx as f64 * 250. + 125.;
            cy = by as f64 * 250. + 125.;
        }
        eprintln!("{name}: centre {cx:.1},{cy:.1}, source {header:?}");
        let full_tile = name.starts_with("tunhovd");
        let bounds = if full_tile {
            header
        } else {
            las::Bounds {
                min: las::Vector {
                    x: cx - 138.,
                    y: cy - 138.,
                    z: header.min.z,
                },
                max: las::Vector {
                    x: cx + 138.,
                    y: cy + 138.,
                    z: header.max.z,
                },
            }
        };
        let points = reader
            .query(las::LodSelection::All, las::BoundsSelection::Within(bounds))?
            .points()
            .map(|p| p.map(PointLaz))
            .collect::<Result<Vec<_>, _>>()?;
        let ground = points
            .iter()
            .filter(|p| p.0.classification == Classification::Ground && !p.0.is_withheld)
            .cloned()
            .collect::<Vec<_>>();
        eprintln!("  {} points, {} ground", points.len(), ground.len());
        anyhow::ensure!(
            ground.len() >= 4,
            "{name}: reference crop needs observed ground"
        );
        let dt = DelaunayTriangulation::<PointLaz>::bulk_load_stable(ground)?;
        let nn = dt.natural_neighbor();
        let mut hull_line = geo::LineString::new(
            dt.convex_hull()
                .map(|edge| {
                    let p = edge.from().position();
                    geo::coord! {x:p.x,y:p.y}
                })
                .collect(),
        );
        hull_line.close();
        let ground_hull = geo::Polygon::new(hull_line, vec![]);
        let (left, top, width, height) = if full_tile {
            (
                header.min.x + 6.,
                header.max.y - 6.,
                ((header.max.x - header.min.x - 12.) / 0.5) as usize,
                ((header.max.y - header.min.y - 12.) / 0.5) as usize,
            )
        } else {
            (cx - 120., cy + 120., 481, 481)
        };
        let grid = DfmGrid::new(width, height, 0.5, geo::coord! {x:left,y:top})?;
        let mut dem = Dfm::<Elevation>::new(grid);
        for y in 0..dem.height() {
            for x in 0..dem.width() {
                dem[(y, x)] = nn
                    .interpolate(|p| p.data().0.z, dem.index2spade(y, x))
                    .unwrap_or_else(|| {
                        dt.nearest_neighbor(dem.index2spade(y, x))
                            .unwrap()
                            .data()
                            .0
                            .z
                    }) as f32;
            }
        }
        let cloud = PointCloud::new(points, bounds);
        // The production surface feature computation supplies this same relief.
        let surfaces = super::super::surface_features::compute_surface_features(&cloud, &dem);
        let relief: &Dfm<GroundRelief2m> = &surfaces.ground_relief_2m;
        let values = evidence(&cloud, &dem, relief);
        let rasters = compute_ground_cover(&cloud, &dem, relief);
        let params = crate::parameters::MapParameters::default();
        use geo::BooleanOps;
        let footprint = if full_tile {
            geo::Rect::new(
                (left + 8., top - (height as f64) * 0.5 + 8.),
                (left + (width as f64) * 0.5 - 8., top - 8.),
            )
            .to_polygon()
        } else {
            geo::Rect::new((cx - 110., cy - 110.), (cx + 110., cy + 110.)).to_polygon()
        };
        let footprint = footprint
            .intersection(&ground_hull)
            .0
            .into_iter()
            .max_by(|a, b| {
                use geo::Area;
                a.unsigned_area().total_cmp(&b.unsigned_area())
            })
            .expect("ground-supported crop");
        let mut map = crate::map::InternalMap::new(
            geo::coord! {x:cx,y:cy},
            params.scale,
            Some(proj_wkt::parse_crs(crs)?),
        );
        use crate::map::{AreaSymbol, MapObject, Symbol};
        use geo::{Area, MapCoordsInPlace};
        let mut objects = super::super::compute_vegetation(
            &rasters.undergrowth,
            crate::raster::Threshold::Lower(params.vegetation.undergrowth_threshold),
            &footprint,
            &footprint,
            AreaSymbol::Undergrowth,
            &params,
            &params.geometry.vegetation.buffer_rules,
        );
        objects.extend(super::super::compute_vegetation(
            &rasters.boulder_field,
            crate::raster::Threshold::Lower(params.boulder_fields.threshold),
            &footprint,
            &footprint,
            AreaSymbol::BoulderField,
            &params,
            &params.geometry.boulder_fields.buffer_rules,
        ));
        for mut object in objects {
            if let MapObject::Area { object, .. } = &mut object {
                object.map_coords_in_place(|c| geo::coord! {x:c.x-cx,y:c.y-cy});
            }
            map.add_object(object);
        }
        map.merge_and_filter_min_size([AreaSymbol::Undergrowth, AreaSymbol::BoulderField])?;
        map.resolve_ground_cover_conflicts();
        let polygons = |symbol| {
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
        };
        let under = polygons(AreaSymbol::Undergrowth);
        let rocks = polygons(AreaSymbol::BoulderField);
        let green = polygons(AreaSymbol::LightGreen).unsigned_area();
        let overlap = under.intersection(&rocks).unsigned_area();
        assert!(overlap < 1e-3, "{name}: remaining overlap {overlap} m²");
        if name.starts_with("sandhammaren") {
            assert_eq!(rocks.unsigned_area(), 0.);
        }
        if ["sundsvall-funnel", "sundsvall-field", "sundsvall-stream"].contains(&name) {
            assert!(under.unsigned_area() > 100.);
        }
        eprintln!(
            "  mapped 409={:.0} m², 211={:.0} m², overlap green={green:.0} m²",
            under.unsigned_area(),
            rocks.unsigned_area()
        );
        writeln!(
            summary,
            "{name},{cx},{cy},{},{},{green}",
            under.unsigned_area(),
            rocks.unsigned_area()
        )?;
        map.into_omap(f64::from(dem.field[dem.field.len() / 2]), &params.geometry)?
            .to_file(format!("/tmp/omap-ground-cover/{name}.omap"))?;
        let mut out = std::io::BufWriter::new(std::fs::File::create(format!(
            "/tmp/omap-ground-cover/{name}.csv"
        ))?);
        writeln!(
            out,
            "x,y,z,near,undergrowth_hits,nonterminal,terminal,higher_near_ground_hits,ground,roughness,coherent,ground_in_undergrowth_band,undergrowth,boulder_field"
        )?;
        for y in (12..dem.height() - 12).step_by(4) {
            for x in (12..dem.width() - 12).step_by(4) {
                let i = y * dem.width() + x;
                let v = values[i];
                let p = dem.index2coord(y, x);
                writeln!(
                    out,
                    "{},{},{},{},{},{},{},{},{},{},{},{},{},{}",
                    p.x,
                    p.y,
                    dem.field[i],
                    v.near,
                    v.undergrowth_hits,
                    v.undergrowth_nonterminal,
                    v.undergrowth_terminal,
                    v.higher_near_ground_hits,
                    v.ground,
                    v.roughness,
                    v.coherent,
                    v.ground_in_undergrowth_band,
                    rasters.undergrowth.field[i],
                    rasters.boulder_field.field[i]
                )?;
            }
        }
    }
    Ok(())
}

fn synthetic_cloud(
    low_heights: &[f64],
    nonterminal: bool,
) -> (PointCloud, Dfm<Elevation>, Dfm<GroundRelief2m>) {
    use crate::{geometry::PointLaz, raster::DfmGrid};
    let mut dem =
        Dfm::<Elevation>::new(DfmGrid::new(41, 41, 0.5, geo::coord! {x:0.,y:20.}).unwrap());
    dem.field.fill(0.);
    let mut relief = Dfm::<GroundRelief2m>::new_like(&dem);
    relief.field.fill(0.10);
    let mut points = Vec::new();
    for y in 0..40 {
        for x in 0..40 {
            let mut ground = PointLaz::new(x as f64 / 2., y as f64 / 2., 0.);
            ground.0.is_synthetic = false;
            points.push(ground);
            for &height in low_heights {
                let mut p = PointLaz::new(x as f64 / 2. + 0.01, y as f64 / 2. + 0.01, height);
                p.0.classification = Classification::Unclassified;
                p.0.is_synthetic = false;
                if nonterminal {
                    p.0.number_of_returns = 2;
                    p.0.return_number = 1;
                }
                points.push(p);
            }
        }
    }
    let bounds = las::Bounds {
        min: las::Vector {
            x: 0.,
            y: 0.,
            z: 0.,
        },
        max: las::Vector {
            x: 20.,
            y: 20.,
            z: 20.,
        },
    };
    (PointCloud::new(points, bounds), dem, relief)
}

#[test]
fn low_penetrable_vegetation_is_undergrowth_and_taller_vegetation_is_not() {
    let (cloud, dem, relief) = synthetic_cloud(&[0.25, 0.5, 0.75], true);
    let result = compute_ground_cover(&cloud, &dem, &relief);
    let i = 20 * dem.width() + 20;
    assert!(result.undergrowth.field[i] > 0.5);
    assert_eq!(result.boulder_field.field[i], 0.);
    let (cloud, dem, relief) = synthetic_cloud(&[1., 1.4, 1.9], true);
    assert_eq!(
        compute_ground_cover(&cloud, &dem, &relief)
            .undergrowth
            .field[i],
        0.
    );
}

#[test]
fn opaque_low_surfaces_survive_canopy_and_last_return_metadata() {
    let (mut cloud, dem, relief) = synthetic_cloud(&[0.45, 0.5, 0.55], false);
    for point in &mut cloud.points {
        if point.0.z > 0.2 {
            point.0.return_number = 2;
            point.0.number_of_returns = 2;
        }
    }
    let before = compute_ground_cover(&cloud, &dem, &relief);
    let mut canopy = cloud.points.clone();
    for point in &mut canopy {
        point.0.z += 10.;
        point.0.return_number = 1;
        point.0.number_of_returns = 2;
    }
    cloud.points.extend(canopy);
    let after = compute_ground_cover(&cloud, &dem, &relief);
    let i = 20 * dem.width() + 20;
    assert!(before.boulder_field.field[i] > 0.5);
    assert_eq!(before.boulder_field.field, after.boulder_field.field);
    assert_eq!(before.undergrowth.field, after.undergrowth.field);
    assert!(before.undergrowth.field[i] < 0.5);
}

#[test]
fn rough_terrain_without_near_ground_objects_and_unsupported_returns_are_rejected() {
    let (cloud, dem, mut relief) = synthetic_cloud(&[], false);
    relief.field.fill(0.5);
    let result = compute_ground_cover(&cloud, &dem, &relief);
    assert!(result.boulder_field.field.iter().all(|&v| v == 0.));
    let sparse = PointCloud::new(
        vec![crate::geometry::PointLaz::new(10., 10., 0.5)],
        cloud.bounds,
    );
    let result = compute_ground_cover(&sparse, &dem, &relief);
    assert!(result.undergrowth.field.iter().all(|&v| v == 0.));
    assert!(result.boulder_field.field.iter().all(|&v| v == 0.));
}

#[test]
fn excluded_returns_cannot_provide_low_vegetation_or_rock_support() {
    let (base, dem, relief) = synthetic_cloud(&[0.3, 0.5, 0.7], false);
    for kind in 0..6 {
        let mut cloud = base.clone();
        for point in &mut cloud.points {
            match kind {
                0 => point.0.is_withheld = true,
                1 => point.0.is_synthetic = true,
                2 => point.0.classification = Classification::HighNoise,
                3 => point.0.classification = Classification::Water,
                4 => point.0.classification = Classification::Building,
                _ => point.0.return_number = 0,
            }
        }
        let result = compute_ground_cover(&cloud, &dem, &relief);
        assert!(result.undergrowth.field.iter().all(|&v| v == 0.));
        assert!(result.boulder_field.field.iter().all(|&v| v == 0.));
    }
}

#[test]
fn undergrowth_and_rock_scores_ignore_all_untrusted_labels() {
    for penetrable in [false, true] {
        let (base, dem, relief) = synthetic_cloud(&[0.3, 0.5, 0.7], penetrable);
        let expected = compute_ground_cover(&base, &dem, &relief);
        for label in [
            Classification::Unclassified,
            Classification::LowVegetation,
            Classification::MediumVegetation,
            Classification::HighVegetation,
        ] {
            let mut cloud = base.clone();
            for point in &mut cloud.points {
                if point.0.classification != Classification::Ground {
                    point.0.classification = label;
                }
            }
            let actual = compute_ground_cover(&cloud, &dem, &relief);
            assert_eq!(
                actual.undergrowth.field, expected.undergrowth.field,
                "{label:?}"
            );
            assert_eq!(
                actual.boulder_field.field, expected.boulder_field.field,
                "{label:?}"
            );
        }
    }
}

#[test]
fn low_vegetation_boundary_is_excluded_from_undergrowth() {
    for height in [0.8, 1., 1.79] {
        let (cloud, dem, relief) = synthetic_cloud(&[height], true);
        let result = compute_ground_cover(&cloud, &dem, &relief);
        assert!(
            result.undergrowth.field.iter().all(|&v| v == 0.),
            "height {height}"
        );
    }
}
