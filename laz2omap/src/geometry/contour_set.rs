use crate::{geometry::MapLineString, raster::Dfm, SIDE_LENGTH};

use geo::{MultiLineString, Vector2DOps};
use spade::{DelaunayTriangulation, HasPosition, Point2, Triangulation};

#[derive(Debug, Clone)]
pub struct ContourSet(pub Vec<ContourLevel>);
impl ContourSet {
    pub fn with_capacity(num_levels: usize) -> ContourSet {
        ContourSet(Vec::with_capacity(num_levels))
    }

    pub fn interpolate(&self, interpolated_dem: &mut Dfm, adjusted_dem: &Dfm) -> crate::Result<()> {
        let tri = self.triangulate(adjusted_dem);
        let nn = tri.natural_neighbor();

        // interpolate triangulation
        for y_index in 0..SIDE_LENGTH {
            for x_index in 0..SIDE_LENGTH {
                let coords = interpolated_dem.index2spade(y_index, x_index);

                if let Some(elev) =
                    nn.interpolate_gradient(|p| p.data().z, |p| p.data().grad, 0.5, coords)
                {
                    if elev.is_nan() {
                        println!("Nan in c1 interpolating!");
                    } else {
                        interpolated_dem[(y_index, x_index)] = elev;
                    }
                } else {
                    println!("DEM coord outside of contour hull");
                }
            }
        }
        Ok(())
    }

    fn triangulate(&self, dem: &Dfm) -> DelaunayTriangulation<ContourPoint> {
        // coarse estimate of number of nodes in triangulation
        // 3 * number of levels * number of lines in first level * number of points in first line of first level
        let mut points = Vec::with_capacity(
            3 * self.0.len() * self.0[0].lines.0.len() * self.0[0].lines.0[0].0.len(),
        );

        // add control (ghost) points along the DEM sides
        // to make the entire DEM be in the interior of the contour set
        // and avoid issues with tiles with few contours
        points.extend(dem.create_ghost_points());

        for level in self.0.iter() {
            for line in level.lines.iter() {
                if line.is_closed() && line.0.len() < 4 {
                    continue;
                }

                for i in 1..line.0.len() - 1 {
                    let cp = ContourPoint {
                        pos: Point2::new(line.0[i].x, line.0[i].y),
                        z: level.z,
                        grad: (line.0[i + 1] - line.0[i - 1])
                            .left()
                            .try_normalize()
                            .unwrap() // should be okay bc of RDP simplification and the check above // is not okay panic happend
                            .into(),
                    };
                    points.push(cp);
                }

                if line.is_closed() {
                    let cp = ContourPoint {
                        pos: Point2::new(line.0[0].x, line.0[0].y),
                        z: level.z,
                        grad: (line.0[1] - line.0[line.0.len() - 2])
                            .left()
                            .try_normalize()
                            .unwrap()
                            .into(),
                    };
                    points.push(cp);
                } else {
                    let cp = ContourPoint {
                        pos: Point2::new(line.0[0].x, line.0[0].y),
                        z: level.z,
                        grad: (line.0[1] - line.0[0])
                            .left()
                            .try_normalize()
                            .unwrap()
                            .into(),
                    };
                    points.push(cp);

                    let cp = ContourPoint {
                        pos: Point2::new(line.0[line.0.len() - 1].x, line.0[line.0.len() - 1].y),
                        z: level.z,
                        grad: (line.0[line.0.len() - 1] - line.0[line.0.len() - 2])
                            .left()
                            .try_normalize()
                            .unwrap()
                            .into(),
                    };
                    points.push(cp);
                }
            }
        }

        let mut tri = DelaunayTriangulation::bulk_load_stable(points.clone()).unwrap();

        // We have the normalized direction of the gradients. Now get the length
        // skip 4 beacuse of the 4 ghosts
        let mut grads = vec![];
        let mut vertices = vec![];
        for v in tri.vertices().skip(4) {
            let mut grad_length = crate::MIN_GRAD_LENGTH;
            let v_pos = v.position();
            let v_height = v.data().z;
            let v_norm_grad = v.data().grad;

            for neighbour in v.out_edges().map(|e| e.to()) {
                // get vec from v to neighbour
                let n_pos = neighbour.position();

                let diff = [
                    n_pos.x - v_pos.x,
                    n_pos.y - v_pos.y,
                    neighbour.data().z - v_height,
                ];

                // if z-component is 0 move on
                if diff[2] == 0. {
                    continue;
                }

                let len2 = diff[0].powi(2) + diff[1].powi(2);

                let grad = [diff[2] * diff[0] / len2, diff[2] * diff[1] / len2];
                let grad_in_dir = grad[0] * v_norm_grad[0] + grad[1] * v_norm_grad[1];

                grad_length = grad_length.max(grad_in_dir);
            }

            vertices.push(v.fix());

            let grad = [v_norm_grad[0] * grad_length, v_norm_grad[1] * grad_length];
            grads.push(grad);
        }

        for (v, g) in vertices.into_iter().zip(grads.into_iter()) {
            tri.vertex_data_mut(v).grad = g;
        }

        tri
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

#[derive(Debug, Clone)]
pub struct ContourLevel {
    pub lines: MultiLineString,
    pub z: f64,
}

impl ContourLevel {
    pub fn new(lines: MultiLineString, z: f64) -> ContourLevel {
        ContourLevel { lines, z }
    }
}

#[derive(Debug, Clone)]
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
