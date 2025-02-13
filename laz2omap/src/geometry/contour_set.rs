use crate::{raster::Dfm, SIDE_LENGTH};

use geo::{Coord, MultiLineString, Vector2DOps};
use spade::{DelaunayTriangulation, HasPosition, Point2};

pub struct ContourSet(pub Vec<ContourLevel>);
impl ContourSet {
    pub fn with_capacity(num_levels: usize) -> ContourSet {
        ContourSet(Vec::with_capacity(num_levels))
    }

    pub fn triangulate(&self, dem: &Dfm) -> DelaunayTriangulation<ContourPoint> {
        // coarse estimate of number of nodes in triangulation
        // number of levels * number of lines in first level * number of points in first line of first level
        let mut points = Vec::with_capacity(
            self.0.len() * self.0[0].lines.0.len() * self.0[0].lines.0[0].0.len(),
        );

        for level in self.0.iter() {
            for line in level.lines.iter() {
                let cp = ContourPoint {
                    pos: Point2::new(line.0[0].x, line.0[0].y),
                    z: level.z,
                    grad_dir: (line.0[1] - line.0[0])
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
                        grad_dir: (line.0[i + 1] - line.0[i - 1])
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
                    grad_dir: (line.0[line.0.len() - 1] - line.0[line.0.len() - 2])
                        .left()
                        .try_normalize()
                        .unwrap_or(Coord { x: 0., y: 0. })
                        .into(),
                };
                points.push(cp);
            }
        }

        // add ghost points in the corners from the DEM
        // to make the entire DEM be in the interior of the contour set
        let ghost_points = vec![
            ContourPoint {
                pos: dem.index2spade(0, 0),
                z: dem[(0, 0)],
                grad_dir: [0., 0.],
            },
            ContourPoint {
                pos: dem.index2spade(SIDE_LENGTH - 1, 0),
                z: dem[(SIDE_LENGTH - 1, 0)],
                grad_dir: [0., 0.],
            },
            ContourPoint {
                pos: dem.index2spade(SIDE_LENGTH - 1, SIDE_LENGTH - 1),
                z: dem[(SIDE_LENGTH - 1, SIDE_LENGTH - 1)],
                grad_dir: [0., 0.],
            },
            ContourPoint {
                pos: dem.index2spade(0, SIDE_LENGTH - 1),
                z: dem[(0, SIDE_LENGTH - 1)],
                grad_dir: [0., 0.],
            },
        ];
        points.extend(ghost_points);

        DelaunayTriangulation::bulk_load_stable(points).unwrap()
    }

    pub fn calculate_error(&self, true_dem: &Dfm, interpolated_dem: &Dfm, lambda: f64) -> f64 {
        1.
    }
}

pub struct ContourLevel {
    lines: MultiLineString,
    z: f64,
}

impl ContourLevel {
    pub fn new(lines: MultiLineString, z: f64) -> ContourLevel {
        ContourLevel { lines, z }
    }
}

pub struct ContourPoint {
    pub pos: Point2<f64>,
    pub z: f64,
    pub grad_dir: [f64; 2],
}

impl HasPosition for ContourPoint {
    type Scalar = f64;

    fn position(&self) -> Point2<Self::Scalar> {
        self.pos
    }
}
