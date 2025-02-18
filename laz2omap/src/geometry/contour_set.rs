use crate::{geometry::MapLineString, raster::Dfm, SIDE_LENGTH};

use geo::{Coord, MultiLineString, Vector2DOps};
use spade::{DelaunayTriangulation, HasPosition, Point2};

pub struct ContourSet(pub Vec<ContourLevel>);
impl ContourSet {
    pub fn with_capacity(num_levels: usize) -> ContourSet {
        ContourSet(Vec::with_capacity(num_levels))
    }

    pub fn interpolate(
        &self,
        interpolated_dem: &mut Dfm,
        adjusted_dem: &Dfm,
        control_points_per_side: usize,
    ) -> crate::Result<()> {
        let tri = self.triangulate(adjusted_dem, control_points_per_side);
        let nn = tri.natural_neighbor();

        // interpolate triangulation
        for y_index in 0..SIDE_LENGTH {
            for x_index in 0..SIDE_LENGTH {
                let coords = interpolated_dem.index2spade(y_index, x_index);

                if let Some(elev) = nn.interpolate(|p| p.data().z, coords) {
                    if elev.is_nan() {
                        println!("Nan in c1 interpolating!");
                    }
                    interpolated_dem[(y_index, x_index)] = elev;
                } else {
                    println!("DEM coord outside of contour hull");
                }
            }
        }
        Ok(())
    }

    fn triangulate(
        &self,
        dem: &Dfm,
        points_per_side: usize,
    ) -> DelaunayTriangulation<ContourPoint> {
        // coarse estimate of number of nodes in triangulation
        // 3 * number of levels * number of lines in first level * number of points in first line of first level
        let mut points = Vec::with_capacity(
            3 * self.0.len() * self.0[0].lines.0.len() * self.0[0].lines.0[0].0.len(),
        );

        // add control (ghost) points along the DEM sides
        // to make the entire DEM be in the interior of the contour set
        // and avoid issues with tiles with few contours
        points.extend(dem.create_ghost_points(points_per_side));

        for level in self.0.iter() {
            for line in level.lines.iter() {
                let cp = ContourPoint {
                    pos: Point2::new(line.0[0].x, line.0[0].y),
                    z: level.z,
                    grad: (line.0[1] - line.0[0])
                        .left()
                        .try_normalize()
                        .unwrap_or(Coord { x: 0., y: 0. })
                        .into(),
                };
                points.push(cp);

                for i in 1..line.0.len() - 1 {
                    let cp = ContourPoint {
                        pos: Point2::new(line.0[i].x, line.0[i].y),
                        z: level.z,
                        grad: (line.0[i + 1] - line.0[i - 1])
                            .left()
                            .try_normalize()
                            .unwrap_or(Coord { x: 0., y: 0. })
                            .into(),
                    };
                    points.push(cp);
                }

                let cp = ContourPoint {
                    pos: Point2::new(line.0[line.0.len() - 1].x, line.0[line.0.len() - 1].y),
                    z: level.z,
                    grad: (line.0[line.0.len() - 1] - line.0[line.0.len() - 2])
                        .left()
                        .try_normalize()
                        .unwrap_or(Coord { x: 0., y: 0. })
                        .into(),
                };
                points.push(cp);
            }
        }

        DelaunayTriangulation::bulk_load_stable(points).unwrap()
    }

    pub fn energy(&self, length_exp: i32) -> f64 {
        let mut tot_energy = 0.;
        for level in self.0.iter() {
            for c in level.lines.iter() {
                tot_energy += c.adjusted_bending_force(length_exp);
            }
        }
        tot_energy
    }
}

pub struct ContourLevel {
    pub lines: MultiLineString,
    pub z: f64,
}

impl ContourLevel {
    pub fn new(lines: MultiLineString, z: f64) -> ContourLevel {
        ContourLevel { lines, z }
    }
}

#[derive(Debug)]
pub struct ContourPoint {
    pub pos: Point2<f64>,
    pub z: f64,
    pub grad: [f64; 2], // direction is always normal to the contour line, length must be derived
}

impl HasPosition for ContourPoint {
    type Scalar = f64;

    fn position(&self) -> Point2<Self::Scalar> {
        self.pos
    }
}
