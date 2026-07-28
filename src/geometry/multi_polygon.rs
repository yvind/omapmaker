use crate::parameters::BufferRule;

use super::MapLineString;
use geo::{BooleanOps, Buffer, Contains, Intersects, Simplify};

/// Match the CLI extractor's default while allowing finer sampling when the
/// collapse threshold itself is smaller.
const MAX_CENTERLINE_SPACING: f64 = 12.0;
/// A branch must extend at least one full collapse diameter to distinguish a
/// line-like arm from the medial-axis spokes of a compact polygon.
const MINIMUM_BRANCH_LENGTH_FACTOR: f64 = 2.0;

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
    /// resemble thick lines remain unchanged. Non-positive/non-finite amounts
    /// leave all polygons unchanged.
    fn collapse(self, amount: f64) -> (geo::MultiLineString, geo::MultiPolygon);
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

    fn collapse(self, amount: f64) -> (geo::MultiLineString, geo::MultiPolygon) {
        if !amount.is_finite() || amount <= 0.0 {
            return (geo::MultiLineString::new(vec![]), self);
        }

        let centerline_spacing = amount.clamp(crate::SIMPLIFICATION_DIST, MAX_CENTERLINE_SPACING);
        let minimum_branch_length = MINIMUM_BRANCH_LENGTH_FACTOR * amount;
        let mut lines = Vec::with_capacity(self.0.len());
        let mut polygons = Vec::with_capacity(self.0.len());

        for polygon in self {
            let Some(centerlines) =
                super::centerline::extract(&polygon, centerline_spacing, minimum_branch_length)
            else {
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
                lines.extend(centerlines);
                continue;
            }

            // Re-expand only the inward-buffer components reached by the
            // centerline, then clip them to the input. This morphological
            // opening restores the original-width portions while leaving
            // narrow arms collapsed.
            let retained = polygon.intersection(&intersecting_buffer.buffer(amount));
            let collapsed_centerlines = retained.clip(&centerlines, true);

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
    fn collapse_turns_a_narrow_polygon_into_its_centerline() {
        let source = rectangle(0.0, 0.0, 100.0, 4.0);

        let (lines, polygons) = geo::MultiPolygon::new(vec![source.clone()]).collapse(3.0);

        assert!(polygons.0.is_empty());
        assert_eq!(lines.0.len(), 1);
        assert!(
            lines.0[0]
                .0
                .iter()
                .all(|coordinate| source.contains(&Point::from(*coordinate)))
        );
        let bounds = lines.0[0].bounding_rect().expect("line has bounds");
        assert!(bounds.width() > 80.0);
    }

    #[test]
    fn collapse_keeps_the_unbuffered_wide_polygon() {
        let source = rectangle(0.0, 0.0, 100.0, 10.0);
        let source_area = source.unsigned_area();

        let (lines, polygons) = geo::MultiPolygon::new(vec![source]).collapse(3.0);

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

        let (lines, polygons) = geo::MultiPolygon::new(vec![source]).collapse(3.0);

        assert!(polygons.0.is_empty());
        assert_eq!(lines.0.len(), 3);
    }

    #[test]
    fn collapse_keeps_a_compact_polygon() {
        let source = rectangle(0.0, 0.0, 10.0, 10.0);
        let source_area = source.unsigned_area();

        let (lines, polygons) = geo::MultiPolygon::new(vec![source]).collapse(6.0);

        assert!(lines.0.is_empty());
        assert_eq!(polygons.0.len(), 1);
        assert_eq!(polygons.unsigned_area(), source_area);
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

        let (lines, polygons) = geo::MultiPolygon::new(vec![source]).collapse(3.0);

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
    fn collapse_classifies_each_polygon_independently() {
        let narrow = rectangle(0.0, 0.0, 100.0, 4.0);
        let wide = rectangle(200.0, 0.0, 100.0, 10.0);

        let (lines, polygons) = geo::MultiPolygon::new(vec![narrow, wide]).collapse(3.0);

        assert_eq!(lines.0.len(), 1);
        assert_eq!(polygons.0.len(), 1);
        assert!(polygons.0[0].bounding_rect().unwrap().min().x >= 200.0);
    }

    #[test]
    fn non_positive_amount_preserves_the_input() {
        let source = geo::MultiPolygon::new(vec![rectangle(0.0, 0.0, 10.0, 4.0)]);

        let (lines, polygons) = source.clone().collapse(0.0);

        assert!(lines.0.is_empty());
        assert_eq!(polygons, source);
    }
}
