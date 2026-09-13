use std::ops::RangeInclusive;

use walkers::{MercatorProjection, PlanarProjection, Position, Projection};

// Walkers currently represents projection-space pixels with the same point type as Position.
type Pixels = Position;

#[derive(Debug, Clone)]
pub(crate) enum MapProjection {
    Mercator(MercatorProjection),
    Planar(PlanarProjection),
}

impl MapProjection {
    pub(crate) fn for_map(
        local_coordinates: bool,
        home: Position,
        boundaries: &[[Position; 4]],
    ) -> Self {
        if !local_coordinates {
            return Self::default();
        }

        let (min_x, max_x, min_y, max_y) = boundaries.iter().flatten().fold(
            (f64::MAX, f64::MIN, f64::MAX, f64::MIN),
            |(min_x, max_x, min_y, max_y), position| {
                (
                    min_x.min(position.x()),
                    max_x.max(position.x()),
                    min_y.min(position.y()),
                    max_y.max(position.y()),
                )
            },
        );
        let extent = (max_x - min_x).max(max_y - min_y);
        let extent = if extent.is_finite() && extent > 0.0 {
            extent
        } else {
            1.0
        };

        Self::Planar(PlanarProjection::new(home, 1.0 / extent))
    }

    pub(crate) fn is_mercator(&self) -> bool {
        matches!(self, Self::Mercator(_))
    }

    pub(crate) fn zoom_range(&self) -> RangeInclusive<f64> {
        match self {
            Self::Mercator(_) => 3.0..=21.0,
            Self::Planar(_) => 6.0..=16.0,
        }
    }

    pub(crate) fn clamp_position(&self, position: Position) -> Position {
        let (x, y) = match self {
            Self::Mercator(_) => (
                position.x().clamp(-180.0, 180.0),
                position.y().clamp(-85.0, 85.0),
            ),
            Self::Planar(projection) => {
                let extent = 1.0 / projection.pixels_per_meter_at_zoom_zero;
                (
                    position.x().clamp(
                        projection.origin.x() - extent,
                        projection.origin.x() + extent,
                    ),
                    position.y().clamp(
                        projection.origin.y() - extent,
                        projection.origin.y() + extent,
                    ),
                )
            }
        };

        walkers::lon_lat(x, y)
    }

    fn projection(&self) -> &dyn Projection {
        match self {
            Self::Mercator(projection) => projection,
            Self::Planar(projection) => projection,
        }
    }
}

impl Default for MapProjection {
    fn default() -> Self {
        Self::Mercator(MercatorProjection)
    }
}

impl Projection for MapProjection {
    fn position_to_pixels(&self, position: Position, zoom: f64) -> Pixels {
        self.projection().position_to_pixels(position, zoom)
    }

    fn pixels_to_position(&self, pixels: Pixels, zoom: f64) -> Position {
        self.projection().pixels_to_position(pixels, zoom)
    }

    fn scale_pixel_per_meter(&self, position: Position, zoom: f64) -> f32 {
        self.projection().scale_pixel_per_meter(position, zoom)
    }

    fn coordinate_kind(&self) -> walkers::CoordinateKind {
        self.projection().coordinate_kind()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn local_projection_uses_largest_boundary_extent() {
        let home = walkers::lon_lat(15.0, 10.0);
        let boundaries = [[
            walkers::lon_lat(0.0, 20.0),
            walkers::lon_lat(0.0, 0.0),
            walkers::lon_lat(30.0, 0.0),
            walkers::lon_lat(30.0, 20.0),
        ]];

        let MapProjection::Planar(projection) = MapProjection::for_map(true, home, &boundaries)
        else {
            panic!("local coordinates should select a planar projection");
        };

        assert_eq!(projection.origin.x(), home.x());
        assert_eq!(projection.origin.y(), home.y());
        assert_eq!(projection.pixels_per_meter_at_zoom_zero, 1.0 / 30.0);
    }

    #[test]
    fn georeferenced_map_uses_mercator() {
        assert!(MapProjection::for_map(false, walkers::lon_lat(0.0, 0.0), &[]).is_mercator());
    }
}
