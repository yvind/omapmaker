use crate::parameters::BufferRule;

use super::MapLineString;
#[cfg(test)]
use geo::{Area, Intersects};
use geo::{BooleanOps, Buffer, Contains, Simplify};

/// Match the CLI extractor's default while allowing finer sampling when the
/// collapse threshold itself is smaller.
#[cfg(test)]
const MAX_CENTERLINE_SPACING: f64 = 12.0;

pub trait MapMultiPolygon {
    fn from_contours(
        contours: geo::MultiLineString,
        convex_hull: &geo::Polygon,
        invert: bool,
    ) -> geo::MultiPolygon;

    fn apply_buffer_rule(self, buffer_rule: &BufferRule) -> geo::MultiPolygon;

    /// Replaces line-like polygons narrower than `2 * amount` with their
    /// medial-axis centerline branches.
    ///
    /// Returns `(collapsed_centerlines, retained_polygons)`. Portions whose
    /// centerlines intersect the inward buffer remain polygonal; narrow
    /// portions removed by that buffer become centerlines. Polygons that do not
    /// resemble thick lines remain unchanged. `minimum_branch_length` controls
    /// that line-like-shape criterion; polygons smaller than
    /// `linearity_exemption_area` bypass it. Collapsed terminals reach either
    /// the parent boundary or the exact retained-width transition so repeated
    /// collapse classes join without gaps. Invalid values leave all polygons
    /// unchanged.
    #[cfg(test)]
    fn collapse(
        self,
        amount: f64,
        minimum_branch_length: f64,
        linearity_exemption_area: f64,
    ) -> (geo::MultiLineString, geo::MultiPolygon);
}

impl MapMultiPolygon for geo::MultiPolygon {
    fn from_contours(
        mut contours: geo::MultiLineString,
        convex_hull: &geo::Polygon,
        invert: bool,
    ) -> geo::MultiPolygon {
        let mut polygons = Vec::with_capacity(contours.0.len());

        if contours.0.is_empty() {
            if invert {
                polygons.push(convex_hull.clone())
            }
            return geo::MultiPolygon::new(polygons);
        }

        let mut i = 0;
        while i < contours.0.len() {
            let Some(area) = contours.0[i].line_string_signed_area() else {
                contours.0.swap_remove(i);
                continue;
            };

            if area > 0. {
                polygons.push(geo::Polygon::new(contours.0.swap_remove(i), vec![]));
            } else {
                i += 1;
            }
        }

        // add the holes to the polygons
        for contour in contours {
            for polygon in &mut polygons {
                if polygon.contains(&contour.0[1]) {
                    polygon.interiors_push(contour);
                    break;
                }
            }
        }

        let mut polygons = geo::MultiPolygon::new(polygons);

        // invert the polygons with respect to the convex hull if we want area below the contours
        if invert {
            polygons = convex_hull.difference(&polygons);
        }

        polygons
    }

    fn apply_buffer_rule(self, buffer_rule: &BufferRule) -> geo::MultiPolygon {
        let sign = match buffer_rule.direction {
            crate::parameters::BufferDirection::Grow => 1.,
            crate::parameters::BufferDirection::Shrink => -1.,
        };
        let distance = sign * buffer_rule.amount;
        self.buffer(distance).simplify(crate::SIMPLIFICATION_DIST)
    }

    #[cfg(test)]
    fn collapse(
        self,
        amount: f64,
        minimum_branch_length: f64,
        linearity_exemption_area: f64,
    ) -> (geo::MultiLineString, geo::MultiPolygon) {
        if !amount.is_finite()
            || amount <= 0.0
            || !minimum_branch_length.is_finite()
            || minimum_branch_length < 0.0
            || !linearity_exemption_area.is_finite()
            || linearity_exemption_area < 0.0
        {
            return (geo::MultiLineString::new(vec![]), self);
        }

        let centerline_spacing = amount.clamp(crate::SIMPLIFICATION_DIST, MAX_CENTERLINE_SPACING);
        let mut lines = Vec::with_capacity(self.0.len());
        let mut polygons = Vec::with_capacity(self.0.len());

        for polygon in self {
            let polygon_minimum_branch_length =
                if polygon.unsigned_area() < linearity_exemption_area {
                    0.0
                } else {
                    minimum_branch_length
                };
            let Some(mut centerlines) = super::centerline::extract(
                &polygon,
                centerline_spacing,
                polygon_minimum_branch_length,
            ) else {
                // A degenerate polygon or failed triangulation must not make
                // source geometry disappear. This also retains compact shapes
                // whose medial axes contain no significant branch.
                polygons.push(polygon);
                continue;
            };

            let intersecting_buffer = geo::MultiPolygon::new(
                polygon
                    .buffer(-amount)
                    .into_iter()
                    .filter(|buffered_polygon| buffered_polygon.intersects(&centerlines))
                    .collect(),
            );

            if intersecting_buffer.0.is_empty() {
                super::centerline::extend_terminal_branches(
                    &mut centerlines,
                    &polygon,
                    &geo::MultiPolygon::empty(),
                );
                lines.extend(centerlines);
                continue;
            }

            // Re-expand only the inward-buffer components reached by the
            // centerline, then clip them to the input. This morphological
            // opening restores the original-width portions while leaving
            // narrow arms collapsed.
            let retained = polygon.intersection(&intersecting_buffer.buffer(amount));
            let mut collapsed_centerlines = retained.clip(&centerlines, true);
            super::centerline::extend_terminal_branches(
                &mut collapsed_centerlines,
                &polygon,
                &retained,
            );

            if collapsed_centerlines.0.is_empty() {
                // Avoid changing corners or boundaries when the whole
                // centerline is covered: this polygon did not collapse at all.
                polygons.push(polygon);
            } else {
                lines.extend(collapsed_centerlines);
                polygons.extend(retained);
            }
        }

        (
            geo::MultiLineString::new(lines),
            geo::MultiPolygon::new(polygons),
        )
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::raster::{Dfm, DfmGrid, Elevation};
    use geo::{Area, BoundingRect, Contains, Point, polygon};

    fn rectangle(x: f64, y: f64, width: f64, height: f64) -> geo::Polygon {
        polygon![
            (x: x, y: y),
            (x: x + width, y: y),
            (x: x + width, y: y + height),
            (x: x, y: y + height),
        ]
    }

    #[test]
    fn contour_winding_builds_a_polygon_with_a_low_island_hole() {
        let grid = DfmGrid::new(7, 7, 1., geo::coord! { x: 0., y: 6. }).unwrap();
        let mut raster = Dfm::<Elevation>::new(grid);
        raster.field.fill(2.);
        raster[(3, 3)] = 0.;
        let domain = geo::Rect::new(geo::coord! { x: -1., y: -1. }, geo::coord! { x: 7., y: 7. })
            .to_polygon();

        let polygons =
            geo::MultiPolygon::from_contours(raster.marching_squares(1.), &domain, false);

        assert_eq!(polygons.0.len(), 1);
        assert_eq!(polygons.0[0].interiors().len(), 1);
        assert!(polygons.0[0].contains(&geo::Point::new(1., 1.)));
        assert!(!polygons.0[0].contains(&geo::Point::new(3., 3.)));
    }

    #[test]
    fn collapse_turns_a_narrow_polygon_into_its_centerline() {
        let source = rectangle(0.0, 0.0, 100.0, 4.0);

        let (lines, polygons) =
            geo::MultiPolygon::new(vec![source.clone()]).collapse(3.0, 6.0, 0.0);

        assert!(polygons.0.is_empty());
        assert_eq!(lines.0.len(), 1);
        assert!(
            lines.0[0]
                .0
                .iter()
                .all(|coordinate| source.intersects(&Point::from(*coordinate)))
        );
        let bounds = lines.0[0].bounding_rect().expect("line has bounds");
        assert!(bounds.width() > 80.0);
    }

    #[test]
    fn small_collapse_amount_does_not_retain_end_cap_spokes() {
        let source = rectangle(0.0, 0.0, 100.0, 0.3);

        let (lines, polygons) = geo::MultiPolygon::new(vec![source]).collapse(0.2, 0.4, 67.0);

        assert!(polygons.0.is_empty());
        assert_eq!(lines.0.len(), 1, "lines={lines:?}");
        let bounds = lines.0[0].bounding_rect().expect("line has bounds");
        assert!(bounds.width() > 99.0, "line only filled {bounds:?}");
    }

    #[test]
    fn collapse_keeps_the_unbuffered_wide_polygon() {
        let source = rectangle(0.0, 0.0, 100.0, 10.0);
        let source_area = source.unsigned_area();

        let (lines, polygons) = geo::MultiPolygon::new(vec![source]).collapse(3.0, 6.0, 0.0);

        assert!(lines.0.is_empty());
        assert_eq!(polygons.0.len(), 1);
        assert_eq!(polygons.unsigned_area(), source_area);
    }

    #[test]
    fn collapse_returns_all_significant_centerline_branches() {
        let source = polygon![
            (x: -2.0, y: -31.0),
            (x: 2.0, y: -31.0),
            (x: 2.0, y: -2.0),
            (x: 29.0, y: -2.0),
            (x: 29.0, y: 2.0),
            (x: -33.0, y: 2.0),
            (x: -33.0, y: -2.0),
            (x: -2.0, y: -2.0),
        ];

        let (lines, polygons) = geo::MultiPolygon::new(vec![source]).collapse(3.0, 6.0, 0.0);

        assert!(polygons.0.is_empty());
        assert_eq!(lines.0.len(), 2);
        let main_bounds = lines.0[0].bounding_rect().expect("main line has bounds");
        assert!(main_bounds.width() > 50.0);
        assert!(main_bounds.height() < 5.0);
    }

    #[test]
    fn collapse_keeps_a_compact_polygon() {
        let source = rectangle(0.0, 0.0, 10.0, 10.0);
        let source_area = source.unsigned_area();

        let (lines, polygons) = geo::MultiPolygon::new(vec![source]).collapse(6.0, 12.0, 0.0);

        assert!(lines.0.is_empty());
        assert_eq!(polygons.0.len(), 1);
        assert_eq!(polygons.unsigned_area(), source_area);
    }

    #[test]
    fn linearity_threshold_is_adjustable() {
        let source = rectangle(0.0, 0.0, 5.0, 5.0);

        let (default_lines, default_polygons) =
            geo::MultiPolygon::new(vec![source.clone()]).collapse(3.0, 6.0, 0.0);
        let (permissive_lines, permissive_polygons) =
            geo::MultiPolygon::new(vec![source]).collapse(3.0, 0.0, 0.0);

        assert!(default_lines.0.is_empty());
        assert_eq!(default_polygons.0.len(), 1);
        assert!(!permissive_lines.0.is_empty());
        assert!(permissive_polygons.0.is_empty());
    }

    #[test]
    fn smallest_polygons_bypass_the_linearity_threshold() {
        let source = rectangle(0.0, 0.0, 5.0, 5.0);

        let (lines, polygons) = geo::MultiPolygon::new(vec![source]).collapse(3.0, 6.0, 26.0);

        assert!(!lines.0.is_empty());
        assert!(polygons.0.is_empty());
    }

    #[test]
    fn collapse_splits_a_wide_body_from_its_narrow_arm() {
        let source = polygon![
            (x: 0.0, y: 0.0),
            (x: 20.0, y: 0.0),
            (x: 20.0, y: 8.0),
            (x: 100.0, y: 8.0),
            (x: 100.0, y: 12.0),
            (x: 20.0, y: 12.0),
            (x: 20.0, y: 20.0),
            (x: 0.0, y: 20.0),
        ];
        let source_area = source.unsigned_area();

        let (lines, polygons) = geo::MultiPolygon::new(vec![source]).collapse(3.0, 6.0, 0.0);

        assert_eq!(lines.0.len(), 1);
        assert!(!polygons.0.is_empty());
        assert!(polygons.unsigned_area() < source_area);

        let polygon_max_x = polygons
            .0
            .iter()
            .filter_map(BoundingRect::bounding_rect)
            .map(|bounds| bounds.max().x)
            .fold(f64::NEG_INFINITY, f64::max);
        let line_max_x = lines
            .0
            .iter()
            .filter_map(BoundingRect::bounding_rect)
            .map(|bounds| bounds.max().x)
            .fold(f64::NEG_INFINITY, f64::max);
        assert!(polygon_max_x < 30.0, "polygon reached x={polygon_max_x}");
        assert!(line_max_x > 90.0, "centerline ended at x={line_max_x}");
    }

    #[test]
    fn consecutive_collapse_classes_fill_a_variable_width_polygon_without_gaps() {
        let source = polygon![
            (x: 0.0, y: -0.75),
            (x: 40.0, y: -0.75),
            (x: 40.0, y: -1.5),
            (x: 120.0, y: -1.5),
            (x: 120.0, y: 1.5),
            (x: 40.0, y: 1.5),
            (x: 40.0, y: 0.75),
            (x: 0.0, y: 0.75),
        ];

        let (small, retained) = geo::MultiPolygon::new(vec![source]).collapse(1.0, 2.0, 0.0);
        let (large, retained) = retained.collapse(2.0, 4.0, 0.0);

        assert!(retained.0.is_empty());
        assert_eq!(small.0.len(), 1, "small={small:?}");
        assert_eq!(large.0.len(), 1, "large={large:?}");
        let small_bounds = small.bounding_rect().unwrap();
        let large_bounds = large.bounding_rect().unwrap();
        let class_join_error = (large_bounds.min().x - small_bounds.max().x).abs();
        assert!(
            small_bounds.min().x.abs() <= 1e-6,
            "small={small_bounds:?}, large={large_bounds:?}"
        );
        assert!(
            class_join_error <= 1e-6,
            "distance between cliff classes was {class_join_error}"
        );
        assert!(
            (large_bounds.max().x - 120.0).abs() <= 1e-6,
            "large cliff ends at x={}",
            large_bounds.max().x
        );
    }

    #[test]
    fn collapse_classifies_each_polygon_independently() {
        let narrow = rectangle(0.0, 0.0, 100.0, 4.0);
        let wide = rectangle(200.0, 0.0, 100.0, 10.0);

        let (lines, polygons) = geo::MultiPolygon::new(vec![narrow, wide]).collapse(3.0, 6.0, 0.0);

        assert_eq!(lines.0.len(), 1);
        assert_eq!(polygons.0.len(), 1);
        assert!(polygons.0[0].bounding_rect().unwrap().min().x >= 200.0);
    }

    #[test]
    fn non_positive_amount_preserves_the_input() {
        let source = geo::MultiPolygon::new(vec![rectangle(0.0, 0.0, 10.0, 4.0)]);

        let (lines, polygons) = source.clone().collapse(0.0, 0.0, 0.0);

        assert!(lines.0.is_empty());
        assert_eq!(polygons, source);
    }
}
