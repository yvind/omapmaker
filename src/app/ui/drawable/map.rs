use std::collections::{HashMap, hash_map::Keys};

use crate::rendering::{
    geo::{EARTH_CIRCUMFERENCE_M, GeoTransform, Mercator, from_mercator, to_mercator},
    paint::{Camera, RenderTuning, View, paint_primitives},
    render::{Primitive, draw_objects, triangulate_rings},
};
use eframe::egui::{self, Color32, Pos2, Stroke};
use omap::parts::MapPart;
use walkers::Projection;

use crate::{
    Result,
    map::{AreaSymbol, InternalMap, Symbol},
    parameters::GeometryParameters,
};

pub struct DrawableOmap {
    hull: Vec<Mercator>,
    map_objects: HashMap<Symbol, Vec<Primitive>>,
    // Each symbol can use several colours. Interleave its primitives with
    // other symbols by the OMAP colour table, rather than by feature type.
    draw_order: Vec<(Symbol, usize)>,
    geo: GeoTransform,
    ref_point: geo::Coord,
}

impl DrawableOmap {
    pub fn keys(&self) -> Keys<'_, Symbol, Vec<Primitive>> {
        self.map_objects.keys()
    }

    pub fn from_temp_map(
        tmap: InternalMap,
        hull: geo::LineString,
        geometry: &GeometryParameters,
        meters_above_sea: f64,
    ) -> Result<Self> {
        let ref_point = tmap.ref_point;
        let keys = tmap.objects.keys().copied().collect::<Vec<_>>();
        // Use the export path for symbol definitions, scaling, rotation and
        // Bézier fitting, so the preview describes the file we will save.
        let mut omap = tmap.into_omap(meters_above_sea, geometry)?;
        let geo = GeoTransform::from_geo_ref(&omap.geo_referencing)?;
        let transform = omap.geo_referencing.create_transform();
        let hull = hull
            .0
            .into_iter()
            .map(|c| {
                let paper = transform.to_map(c + ref_point);
                geo.to_mercator(Pos2::new(paper.x as f32, paper.y as f32))
            })
            .collect();

        let mut objects = omap.parts.remove(0).expect("default OMAP has a part");
        omap.parts.push(MapPart::new("Preview"));
        let mut map_objects = HashMap::with_capacity(keys.len());
        for symbol in keys {
            let id = symbol
                .get_omap_symbol_ref(&omap.symbols)
                .ok_or_else(|| anyhow::anyhow!("Missing preview symbol {symbol}"))?
                .id();
            let mut part = MapPart::new("Preview");
            for object in objects.remove(id) {
                part.add_object(object);
            }
            *omap.parts.get_mut(0).expect("preview part exists") = part;
            // Keep empty entries: an incremental update uses them to remove
            // a feature layer whose regenerated objects have disappeared.
            map_objects.insert(symbol, draw_objects(&omap, &geo));
        }

        let mut drawable = Self {
            hull,
            map_objects,
            draw_order: Vec::new(),
            geo,
            ref_point,
        };
        drawable.sort_primitives();
        Ok(drawable)
    }

    fn sort_primitives(&mut self) {
        self.draw_order = self
            .map_objects
            .iter()
            .flat_map(|(&symbol, primitives)| (0..primitives.len()).map(move |i| (symbol, i)))
            .collect();
        self.draw_order.sort_by_key(|&(symbol, index)| {
            (
                std::cmp::Reverse(self.map_objects[&symbol][index].z),
                symbol,
                index,
            )
        });
    }

    pub fn update(&mut self, other: Self) {
        for (symbol, primitives) in other.map_objects {
            if primitives.is_empty() {
                self.map_objects.remove(&symbol);
            } else {
                self.map_objects.insert(symbol, primitives);
            }
        }
        self.hull = other.hull;
        self.geo = other.geo;
        self.ref_point = other.ref_point;
        self.sort_primitives();
    }

    pub fn draw<P: Projection>(
        &self,
        ui: &mut egui::Ui,
        projector: &walkers::Projector<'_, P>,
        visibilities: &HashMap<Symbol, bool>,
        opacity: f32,
    ) {
        if opacity <= 0. {
            return;
        }

        let mut painter = ui.painter().clone();
        painter.multiply_opacity(opacity);
        let tuning = RenderTuning::default();
        let rect = *projector.clip_rect();
        let center = projector.unproject(rect.center());
        let mm_to_px = self.geo.meters_per_mm() as f32 * projector.scale_pixel_per_meter(center);

        if self.geo.is_local() {
            // The renderer anchors unreferenced maps near the equator. Walkers
            // here uses the original metre coordinates on a planar canvas.
            let to_screen = |p| projector.project(self.local_position(p));
            let bounds = geo::Rect::new(
                self.local_mercator(projector.unproject(rect.min)),
                self.local_mercator(projector.unproject(rect.max)),
            );
            let view = View {
                min: bounds.min(),
                max: bounds.max(),
                to_screen,
                mm_to_px,
                tuning: &tuning,
            };
            self.paint(&painter, &view, visibilities);
        } else {
            // Mercator primitives already contain the expensive half of the
            // projection. Match Walkers' camera with one probe per frame.
            let camera = Camera::at(center, 256. * 2_f64.powf(projector.zoom()));
            let anchor_px = projector.project(center);
            let (min, max) = camera.view_bounds(rect, anchor_px);
            let view = View {
                min,
                max,
                to_screen: camera.to_screen(anchor_px),
                mm_to_px,
                tuning: &tuning,
            };
            self.paint(&painter, &view, visibilities);
        }
    }

    fn local_position(&self, p: Mercator) -> walkers::Position {
        let synthetic = from_mercator(p);
        let meters_per_degree = EARTH_CIRCUMFERENCE_M / 360.;
        walkers::lon_lat(
            self.ref_point.x + synthetic.x() * meters_per_degree,
            self.ref_point.y + synthetic.y() * meters_per_degree,
        )
    }

    fn local_mercator(&self, p: walkers::Position) -> Mercator {
        let degrees_per_meter = 360. / EARTH_CIRCUMFERENCE_M;
        to_mercator(walkers::lon_lat(
            (p.x() - self.ref_point.x) * degrees_per_meter,
            (p.y() - self.ref_point.y) * degrees_per_meter,
        ))
    }

    fn paint<F: Fn(Mercator) -> Pos2>(
        &self,
        painter: &egui::Painter,
        view: &View<'_, F>,
        visibilities: &HashMap<Symbol, bool>,
    ) {
        let hull = self
            .hull
            .iter()
            .map(|&p| (view.to_screen)(p))
            .collect::<Vec<_>>();
        if visibilities
            .get(&Symbol::Area(AreaSymbol::WhiteForest))
            .copied()
            .unwrap_or(true)
        {
            let mut mesh = egui::Mesh {
                indices: triangulate_rings(&hull, &[]),
                ..Default::default()
            };
            for &p in &hull {
                mesh.colored_vertex(p, Color32::WHITE);
            }
            painter.add(mesh);
        }
        painter.add(egui::Shape::closed_line(
            hull,
            Stroke::new(2., Color32::RED),
        ));

        for &(symbol, index) in &self.draw_order {
            if visibilities.get(&symbol).copied().unwrap_or(true) {
                paint_primitives(painter, &self.map_objects[&symbol][index..=index], view);
            }
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::rendering::render::{AreaPattern, Kind, PointMark};
    use crate::{
        app::DrawOrder,
        map::{LineSymbol, MapObject, PointSymbol},
        parameters::Scale,
    };
    use geo::line_string;

    fn hull() -> geo::LineString {
        geo::Rect::new((-100., -100.), (100., 100.))
            .to_polygon()
            .into_inner()
            .0
    }

    fn local_map(scale: Scale) -> InternalMap {
        InternalMap::new(geo::coord! { x: 500_000., y: 6_500_000. }, scale, None)
    }

    fn object(symbol: Symbol) -> MapObject {
        match symbol {
            Symbol::Area(symbol) => MapObject::Area {
                object: geo::Rect::new((-80., -80.), (80., 80.)).to_polygon(),
                symbol,
                tags: HashMap::new(),
            },
            Symbol::Line(symbol) => MapObject::Line {
                object: geo::line_string![(x: -80., y: -20.), (x: 0., y: 20.), (x: 80., y: -20.)],
                symbol,
                tags: HashMap::new(),
            },
            Symbol::Point(symbol) => MapObject::Point {
                object: geo::Point::new(10., 20.),
                symbol,
                rotation: 0.7,
                tags: HashMap::new(),
            },
        }
    }

    fn preview(map: InternalMap) -> DrawableOmap {
        DrawableOmap::from_temp_map(map, hull(), &GeometryParameters::default(), 100.).unwrap()
    }

    #[test]
    fn every_preview_symbol_resolves_and_colours_interleave_in_omap_order() {
        let mut map = local_map(Scale::S15_000);
        for symbol in Symbol::draw_order() {
            map.add_object(object(symbol));
        }
        let map = preview(map);
        for symbol in Symbol::draw_order() {
            let primitives = &map.map_objects[&symbol];
            assert!(!primitives.is_empty(), "no primitives for {symbol}");
            assert!(
                primitives.iter().all(|p| !matches!(
                    p.kind,
                    Kind::FallbackOutline { .. } | Kind::UnresolvedPoint { .. }
                )),
                "unresolved symbol {symbol}"
            );
        }
        let z = map
            .draw_order
            .iter()
            .map(|&(s, i)| map.map_objects[&s][i].z)
            .collect::<Vec<_>>();
        assert!(z.windows(2).all(|p| p[0] >= p[1]));
    }

    #[test]
    fn form_lines_keep_both_their_dash_pattern_and_bezier_curves() {
        let mut map = local_map(Scale::S15_000);
        map.add_object(MapObject::Line {
            object: geo::LineString::from(
                (0..=40)
                    .map(|i| {
                        let x = -80. + 4. * f64::from(i);
                        (x, 20. * (x / 60.).cos())
                    })
                    .collect::<Vec<_>>(),
            ),
            symbol: LineSymbol::FormLine,
            tags: HashMap::new(),
        });
        let map = preview(map);
        let primitives = &map.map_objects[&LineSymbol::FormLine.into()];
        let line = primitives
            .iter()
            .find_map(|p| match &p.kind {
                Kind::Line {
                    path,
                    width_mm,
                    dash,
                    ..
                } => Some((path, width_mm, dash)),
                _ => None,
            })
            .unwrap();
        assert!(
            !line.0.curves.is_empty(),
            "contour Bézier fitting should reach the renderer"
        );
        assert!((line.1 - 0.10).abs() < 1e-6);
        let dash = line.2.as_ref().expect("form lines should be dashed");
        assert!(!dash.dash_lengths_mm.is_empty());
        assert!(dash.gap_lengths_mm.iter().any(|&gap| gap > 0.));
    }

    #[test]
    fn marshes_and_sand_keep_their_patterns_and_water_its_bank_line() {
        let mut map = local_map(Scale::S15_000);
        for symbol in [
            AreaSymbol::Marsh,
            AreaSymbol::SandyGround,
            AreaSymbol::UncrossableWaterWithBankLine,
        ] {
            map.add_object(object(symbol.into()));
        }
        let map = preview(map);
        assert!(map.map_objects[&AreaSymbol::Marsh.into()].iter().any(|p| matches!(
            &p.kind, Kind::Pattern(AreaPattern::Hatch { segments, .. }) if !segments.is_empty()
        )));
        assert!(
            map.map_objects[&AreaSymbol::SandyGround.into()]
                .iter()
                .any(|p| matches!(
                    &p.kind, Kind::Pattern(AreaPattern::Dots { marks, .. }) if !marks.is_empty()
                ))
        );
        assert!(
            map.map_objects[&AreaSymbol::UncrossableWaterWithBankLine.into()]
                .iter()
                .any(|p| matches!(p.kind, Kind::Line { .. }))
        );
    }

    #[test]
    fn depressions_use_the_symbol_shape_and_boulders_match_export_at_both_scales() {
        for scale in [Scale::S10_000, Scale::S15_000] {
            let mut map = local_map(scale);
            map.add_object(object(PointSymbol::UDepression.into()));
            map.add_object(object(PointSymbol::SmallBoulder.into()));
            let map = preview(map);
            assert!(map.map_objects[&PointSymbol::UDepression.into()].iter().any(|p| matches!(
                &p.kind, Kind::PointMark(PointMark::Stroke { path, .. }) if !path.curves.is_empty()
            )));
            let radius = map.map_objects[&PointSymbol::SmallBoulder.into()]
                .iter()
                .find_map(|p| match p.kind {
                    Kind::PointMark(PointMark::Disc { radius_mm, .. }) => Some(radius_mm),
                    _ => None,
                })
                .unwrap();
            // The 1:10,000 symbol set enlarges paper symbols by 1.5, so
            // both default scales have the same footprint on the ground.
            let expected_paper_radius = match scale {
                Scale::S10_000 => 0.3,
                Scale::S15_000 => 0.2,
            };
            assert!((radius - expected_paper_radius).abs() < 1e-6);
            assert!((f64::from(radius) * map.geo.meters_per_mm() - 3.).abs() < 1e-5);
        }
    }

    #[test]
    fn area_meshes_leave_holes_unfilled() {
        let mut map = local_map(Scale::S15_000);
        let outer = geo::Rect::new((-80., -80.), (80., 80.)).to_polygon();
        let hole = geo::Rect::new((-20., -20.), (20., 20.)).to_polygon();
        map.add_object(MapObject::Area {
            object: geo::Polygon::new(outer.into_inner().0, vec![hole.into_inner().0]),
            symbol: AreaSymbol::LightGreen,
            tags: HashMap::new(),
        });
        let map = preview(map);
        let area = map.map_objects[&AreaSymbol::LightGreen.into()]
            .iter()
            .find_map(|p| match &p.kind {
                Kind::Area {
                    exterior,
                    interiors,
                    fill_indices,
                    ..
                } => Some((exterior, interiors, fill_indices)),
                _ => None,
            })
            .unwrap();
        assert_eq!(area.1.len(), 1);
        let vertices = area
            .0
            .iter()
            .chain(area.1.iter().flatten())
            .map(|&p| map.local_position(p).0)
            .collect::<Vec<_>>();
        let filled_area = area
            .2
            .as_chunks::<3>()
            .0
            .iter()
            .map(|&[a, b, c]| {
                let (a, b, c) = (
                    vertices[a as usize],
                    vertices[b as usize],
                    vertices[c as usize],
                );
                ((b.x - a.x) * (c.y - a.y) - (b.y - a.y) * (c.x - a.x)).abs() / 2.
            })
            .sum::<f64>();
        assert!(
            (filled_area - (160. * 160. - 40. * 40.)).abs() < 0.1,
            "filled area is {filled_area}"
        );
    }

    #[test]
    fn local_coordinates_keep_their_original_reference_point() {
        let map = preview(local_map(Scale::S15_000));
        assert!(map.geo.is_local());
        for (&actual, expected) in map.hull.iter().zip(hull().0) {
            let point = map.local_position(actual);
            assert!((point.x() - map.ref_point.x - expected.x).abs() < 1e-4);
            assert!((point.y() - map.ref_point.y - expected.y).abs() < 1e-4);
            let back = map.local_mercator(point);
            assert!((back.x - actual.x).abs() < 1e-12);
            assert!((back.y - actual.y).abs() < 1e-12);
        }
    }

    #[test]
    fn georeferenced_preview_matches_walkers_after_panning_and_zooming() {
        let mut map = local_map(Scale::S15_000);
        map.crs = Some(proj_wkt::parse_crs("EPSG:3006").unwrap());
        let ref_point = map.ref_point;
        let map = preview(map);
        assert!(!map.geo.is_local());
        let transform = proj_core::Transform::from_horizontal_components(
            &proj_wkt::parse_crs("EPSG:3006").unwrap(),
            &crate::projection::get_global_crs(),
        )
        .unwrap();
        let home = geo::Point::from(transform.convert(ref_point).unwrap());
        let rect = egui::Rect::from_min_size(Pos2::new(50., 80.), egui::vec2(600., 400.));
        let mut memory = walkers::MapMemory::default();
        for zoom in [12., 18., 21.] {
            memory.set_zoom(zoom).unwrap();
            memory.center_at(walkers::lon_lat(home.x() + 0.001, home.y() - 0.001));
            let projector = walkers::Projector::new(rect, &memory, home);
            let center = projector.unproject(rect.center());
            let camera = Camera::at(center, 256. * 2_f64.powf(projector.zoom()));
            let to_screen = camera.to_screen(projector.project(center));
            for (&actual, expected) in map.hull.iter().zip(hull().0) {
                let position = geo::Point::from(transform.convert(ref_point + expected).unwrap());
                assert!(to_screen(actual).distance(projector.project(position)) < 0.01);
            }
        }
    }

    #[test]
    fn incremental_updates_replace_changed_layers_and_remove_empty_ones() {
        let mut map = local_map(Scale::S15_000);
        map.add_object(object(LineSymbol::Contour.into()));
        map.add_object(object(AreaSymbol::LightGreen.into()));
        let mut map = preview(map);
        let contour_count = map.map_objects[&LineSymbol::Contour.into()].len();

        let mut update = local_map(Scale::S15_000);
        update.reserve_capacity(AreaSymbol::LightGreen, 0);
        update.add_object(object(PointSymbol::SmallBoulder.into()));
        map.update(preview(update));
        assert_eq!(
            map.map_objects[&LineSymbol::Contour.into()].len(),
            contour_count
        );
        assert!(!map.map_objects.contains_key(&AreaSymbol::LightGreen.into()));
        assert!(!map.map_objects[&PointSymbol::SmallBoulder.into()].is_empty());

        let mut update = local_map(Scale::S15_000);
        update.add_object(object(PointSymbol::SmallBoulder.into()));
        update.add_object(object(PointSymbol::SmallBoulder.into()));
        map.update(preview(update));
        assert_eq!(map.map_objects[&PointSymbol::SmallBoulder.into()].len(), 2);
    }

    #[test]
    fn local_painting_preserves_position_size_visibility_and_opacity() {
        let mut map = local_map(Scale::S15_000);
        map.add_object(object(PointSymbol::SmallBoulder.into()));
        let map = preview(map);
        let home = geo::Point::from(map.ref_point);
        let ctx = egui::Context::default();
        let mut memory = walkers::MapMemory::new(walkers::PlanarProjection::new(home, 0.01));
        let mut visibility = HashMap::new();
        let render = |memory: &walkers::MapMemory<walkers::PlanarProjection>,
                      visibility: &HashMap<Symbol, bool>,
                      opacity| {
            let mut output = ctx.run_ui(
                egui::RawInput {
                    screen_rect: Some(egui::Rect::from_min_size(
                        Pos2::ZERO,
                        egui::vec2(600., 400.),
                    )),
                    ..Default::default()
                },
                |ui| {
                    let projector = walkers::Projector::new(ui.max_rect(), memory, home);
                    map.draw(ui, &projector, visibility, opacity);
                },
            );
            output.textures_delta.clear();
            output.shapes
        };
        for zoom in [7., 9.] {
            memory.set_zoom(zoom).unwrap();
            memory.center_at(walkers::lon_lat(home.x() + 5., home.y() + 10.));
            let shapes = render(&memory, &visibility, 0.5);
            let circle = shapes
                .iter()
                .find_map(|s| match &s.shape {
                    egui::Shape::Circle(circle) => Some((s.clip_rect, circle)),
                    _ => None,
                })
                .expect("the boulder should be painted");
            let projector = walkers::Projector::new(circle.0, &memory, home);
            let position = walkers::lon_lat(home.x() + 10., home.y() + 20.);
            assert!(circle.1.center.distance(projector.project(position)) < 0.01);
            assert!(
                (circle.1.radius - 3. * projector.scale_pixel_per_meter(position)).abs() < 0.01
            );
            assert!((i16::from(circle.1.fill.a()) - 128).abs() <= 1);
        }
        visibility.insert(PointSymbol::SmallBoulder.into(), false);
        assert!(
            render(&memory, &visibility, 1.)
                .iter()
                .all(|s| !matches!(s.shape, egui::Shape::Circle(_)))
        );
        visibility.insert(AreaSymbol::WhiteForest.into(), false);
        assert!(
            render(&memory, &visibility, 1.)
                .iter()
                .all(|s| !matches!(s.shape, egui::Shape::Mesh(_)))
        );
        assert!(render(&memory, &HashMap::new(), 0.).is_empty());
    }
}
