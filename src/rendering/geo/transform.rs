//! Adapts [`omap::geo_referencing::MapTransform`] to the types the GUI speaks:
//! `egui`'s [`Pos2`] for paper millimetres, `walkers`' [`Position`] for WGS84.
//!
//! Paper coordinates narrow to `f32` at this boundary. A map's paper extent is
//! a few hundred millimetres, so `f32` resolves it far finer than the
//! georeferencing is accurate to.
//!
//! A map with no real-world location (`CrsType::Local` — a training map, a
//! club sketch map, anything never run through Mapper's georeferencing step)
//! still builds: see [`GeoTransform::from_geo_ref`].

use super::{Mercator, to_mercator};
use eframe::egui::Pos2;
use geo::{BoundingRect, LineString};
use omap::geo_referencing::{GeoRef, MapTransform};
use omap::geo_types::Coord;
use walkers::Position;

use super::EARTH_CIRCUMFERENCE_M;

enum Kind {
    Georeferenced(Box<MapTransform>),
    /// Paper mm treated as a locally-flat patch of ground, anchored at
    /// (lon 0°, lat 0°) — the equator, deliberately: Web Mercator's
    /// distortion factor is `1/cos(lat)`, exactly 1 there, so this patch
    /// needs no latitude correction to agree with every other place in the
    /// crate that already assumes Mercator units correspond to undistorted
    /// ground metres at the latitude in question (see [`Mercator`]).
    Local {
        meters_per_mm: f64,
    },
}

/// A transform from paper (mm, y-up = magnetic north) to WGS84 [`Position`],
/// built from a map's real georeferencing — or, for a map with none, a
/// synthetic stand-in (see [`Self::from_geo_ref`]) that keeps every other
/// formula in the crate working unchanged.
pub struct GeoTransform(Kind);

impl GeoTransform {
    /// Build a transform from a parsed [`GeoRef`].
    ///
    /// Fails only for a map whose CRS is real but cannot be related to
    /// WGS84. A map with *no* CRS at all (`CrsType::Local`) still succeeds,
    /// via a synthetic transform anchored at an arbitrary point — nothing
    /// about drawing the printed sheet on screen needs a real location, and
    /// plenty of real `.omap` files are never georeferenced.
    pub fn from_geo_ref(gr: &GeoRef) -> Result<Self, omap::Error> {
        match gr.try_get_transform() {
            Ok(map) => Ok(GeoTransform(Kind::Georeferenced(Box::new(map)))),
            Err(omap::Error::LocalCrsHasNoDefinition) => Ok(GeoTransform(Kind::Local {
                meters_per_mm: gr.map_to_ground_scale(),
            })),
            Err(e) => Err(e),
        }
    }

    /// Whether this map has no real-world location — see
    /// [`Self::from_geo_ref`]. A consumer that only makes sense with a real
    /// location (an OSM/topo basemap toggle, say) should gate on this.
    pub fn is_local(&self) -> bool {
        matches!(self.0, Kind::Local { .. })
    }

    /// Ground metres per paper millimetre. Well-defined as a single scalar
    /// because the paper ↔ projected-CRS transform is a similarity (uniform
    /// scale + rotation, no shear) — or, for [`Kind::Local`], because that's
    /// exactly what it was built from in the first place.
    pub fn meters_per_mm(&self) -> f64 {
        match &self.0 {
            Kind::Georeferenced(map) => map.scale_factor(),
            Kind::Local { meters_per_mm } => *meters_per_mm,
        }
    }

    /// Convert a paper coordinate (mm, y-up) into a geographic [`Position`].
    ///
    /// Falls back to `NaN` (culled by downstream bounds checks rather than
    /// panicking) on the practically-unreachable case of a projection domain
    /// error for a point inside the map's own paper extent.
    pub fn to_position(&self, paper: Pos2) -> Position {
        match &self.0 {
            Kind::Georeferenced(map) => {
                match map.to_wgs84(Coord {
                    x: paper.x as f64,
                    y: paper.y as f64,
                }) {
                    Ok(geo) => walkers::lon_lat(geo.x, geo.y),
                    Err(_) => walkers::lon_lat(f64::NAN, f64::NAN),
                }
            }
            Kind::Local { meters_per_mm } => {
                let degrees_per_mm = degrees_per_mm(*meters_per_mm);
                walkers::lon_lat(
                    paper.x as f64 * degrees_per_mm,
                    paper.y as f64 * degrees_per_mm,
                )
            }
        }
    }

    /// Inverse of [`Self::to_position`]: a geographic position back to paper
    /// millimetres.
    ///
    /// Needed to turn a click on the map widget into a map coordinate. Returns
    /// `None` on a projection domain error, which is reachable here in a way it
    /// is not for `to_position` — a user can click well outside the map's own
    /// extent.
    #[cfg(test)]
    pub fn to_paper(&self, pos: Position) -> Option<Pos2> {
        match &self.0 {
            Kind::Georeferenced(map) => {
                let paper = map
                    .from_wgs84(Coord {
                        x: pos.x(),
                        y: pos.y(),
                    })
                    .ok()?;
                (paper.x.is_finite() && paper.y.is_finite())
                    .then(|| Pos2::new(paper.x as f32, paper.y as f32))
            }
            Kind::Local { meters_per_mm } => {
                let degrees_per_mm = degrees_per_mm(*meters_per_mm);
                Some(Pos2::new(
                    (pos.x() / degrees_per_mm) as f32,
                    (pos.y() / degrees_per_mm) as f32,
                ))
            }
        }
    }

    /// A paper coordinate straight to the Mercator square — what the render
    /// primitives store. See [`Mercator`] for why they store that and not
    /// lon/lat.
    pub fn to_mercator(&self, paper: Pos2) -> Mercator {
        to_mercator(self.to_position(paper))
    }

    /// Bounds of all four paper-box corners in normalized Mercator space.
    pub fn mercator_bounds(&self, min: Pos2, max: Pos2) -> (Mercator, Mercator) {
        let corners = [min, Pos2::new(min.x, max.y), max, Pos2::new(max.x, min.y)];
        let bounds = LineString(corners.map(|c| self.to_mercator(c)).to_vec())
            .bounding_rect()
            .expect("a box has four corners");
        (bounds.min(), bounds.max())
    }
}

/// Degrees of synthetic longitude/latitude per paper millimetre, for
/// [`Kind::Local`] — the inverse of "metres per degree at the equator".
fn degrees_per_mm(meters_per_mm: f64) -> f64 {
    meters_per_mm / (EARTH_CIRCUMFERENCE_M / 360.0)
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::num::NonZeroU32;

    fn local_1_to_4000() -> GeoRef {
        GeoRef::new(NonZeroU32::new(4000).unwrap())
    }

    #[test]
    fn a_local_crs_builds_instead_of_failing() {
        let gr = local_1_to_4000();
        let geo = GeoTransform::from_geo_ref(&gr).expect("Local CRS should still build");
        assert!(geo.is_local());
    }

    #[test]
    fn a_local_crs_reports_ground_scale_from_its_map_scale() {
        let gr = local_1_to_4000();
        let geo = GeoTransform::from_geo_ref(&gr).unwrap();
        // 1:4000 means 1mm on paper is 4000mm = 4m on the ground.
        assert!((geo.meters_per_mm() - 4.0).abs() < 1e-9);
    }

    #[test]
    fn a_local_crs_paper_round_trips_through_its_synthetic_position() {
        let gr = local_1_to_4000();
        let geo = GeoTransform::from_geo_ref(&gr).unwrap();
        let paper = Pos2::new(123.0, -45.0);
        let back = geo
            .to_paper(geo.to_position(paper))
            .expect("a finite paper point should round-trip");
        assert!((back.x - paper.x).abs() < 1e-3 && (back.y - paper.y).abs() < 1e-3);
    }

    #[test]
    fn a_local_crs_is_anchored_at_the_origin() {
        let gr = local_1_to_4000();
        let geo = GeoTransform::from_geo_ref(&gr).unwrap();
        let origin = geo.to_position(Pos2::ZERO);
        assert_eq!((origin.x(), origin.y()), (0.0, 0.0));
    }
}
