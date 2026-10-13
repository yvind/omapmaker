//! The zoom-independent half of getting a vertex onto the screen.

use std::f64::consts::PI;

use walkers::Position;

/// A point on the Web Mercator unit square: `x` running east from the
/// antimeridian, `y` south from the north edge, both in `0..1`.
///
/// What the render primitives store instead of longitude and latitude. Turning
/// lon/lat into this costs a `tan` and an `asinh` per vertex, and none of it
/// depends on the view — `walkers::Projector::project` is exactly *affine* in
/// this space, scaling by the zoom's pixel count and offsetting by the view
/// centre. Resolving it once at load time is what leaves the paint loop a
/// multiply and an add per coordinate instead of two transcendentals.
///
/// `f64` rather than `f32` because the whole world lives in `0..1`: a
/// competition map spans some `1e-5` of that, which `f32` would quantise to a
/// few dozen distinguishable positions across the sheet.
pub type Mercator = geo::Coord<f64>;

/// Normalize a geographic position exactly as walkers does before scaling by zoom.
pub fn to_mercator(position: Position) -> Mercator {
    let x = position.x().to_radians();
    let y = position.y().to_radians().tan().asinh();
    Mercator {
        x: (1. + (x / PI)) / 2.,
        y: (1. - (y / PI)) / 2.,
    }
}

/// Inverse of [`to_mercator`], for turning a screen point back into the ground.
pub fn from_mercator(point: Mercator) -> Position {
    let lon = (2.0 * point.x - 1.0) * 180.0;
    let lat = (PI * (1.0 - 2.0 * point.y)).sinh().atan().to_degrees();
    walkers::lon_lat(lon, lat)
}

#[cfg(test)]
mod tests {
    use super::*;
    use walkers::Projection;

    /// The whole saving rests on this. `walkers::Projector::project` is this
    /// normalization scaled by the zoom's pixel count and offset by the view
    /// centre, so the paint transform can skip straight to the scale and offset
    /// — but only while the two agree. If a `walkers` bump ever changes the
    /// projection, the map would slide off its backdrop tiles, and this is what
    /// says so.
    #[test]
    fn normalization_agrees_with_walkers() {
        for zoom in [4.0f64, 12.0, 18.0] {
            let total = 2f64.powf(zoom) * 256.0;
            for (lon, lat) in [
                (0.0, 0.0),
                (8.31, 61.64),
                (-122.4, 37.8),
                (179.9, -60.0),
                (-179.9, 84.0),
            ] {
                let p = walkers::lon_lat(lon, lat);
                let theirs = walkers::MercatorProjection.position_to_pixels(p, zoom);
                let ours = to_mercator(p);
                let (dx, dy) = (
                    (ours.x * total - theirs.x()).abs(),
                    (ours.y * total - theirs.y()).abs(),
                );
                // A ten-thousandth of a pixel, at zoom levels where one pixel
                // is a fraction of a metre on the ground.
                assert!(
                    dx < 1e-4 && dy < 1e-4,
                    "({lon}, {lat}) at zoom {zoom}: off by ({dx}, {dy}) pixels"
                );
            }
        }
    }

    #[test]
    fn a_position_survives_the_trip_to_the_square_and_back() {
        for (lon, lat) in [(0.0, 0.0), (8.31, 61.64), (-122.4, 37.8), (179.9, -60.0)] {
            let back = from_mercator(to_mercator(walkers::lon_lat(lon, lat)));
            assert!(
                (back.x() - lon).abs() < 1e-9 && (back.y() - lat).abs() < 1e-9,
                "({lon}, {lat}) came back as ({}, {})",
                back.x(),
                back.y()
            );
        }
    }
}
