use std::ops::{Add, Sub};

use las::Bounds;
use std::hash::{Hash, Hasher};

#[derive(Clone, Copy, Debug)]
pub struct Vertex {
    pub x: f64,
    pub y: f64,
    pub id: usize,
}

impl Vertex {
    pub fn new(x: f64, y: f64, xi: usize, yi: usize, ei: usize, width: usize) -> Vertex {
        Vertex {
            x,
            y,
            id: Vertex::edge_id(xi, yi, ei, width),
        }
    }

    fn edge_id(xi: usize, yi: usize, ei: usize, width: usize) -> usize {
        match ei {
            0 => return yi * (2 * width - 1) + xi,
            1 => return yi * (2 * width - 1) + xi + width,
            2 => return (yi + 1) * (2 * width - 1) + xi,
            3 => return yi * (2 * width - 1) + xi + width - 1,
            _ => panic!("edge index out of bounds: {ei} not in [0, 3]"),
        }
    }

    pub fn get_index(&self, width: usize) -> [[usize; 2]; 2] {
        if self.id % (2 * width - 1) < width - 1 {
            let xi = self.id % (2 * width - 1);
            let yi = self.id / (2 * width - 1);
            return [[xi, yi], [xi + 1, yi]];
        } else {
            let xi = self.id % (2 * width - 1) - width + 1;
            let yi = self.id / (2 * width - 1);
            return [[xi, yi], [xi, yi + 1]];
        }
    }

    // distance from self to other along the border of bounding_box if both self and other lies on the border
    pub fn get_boundary_dist(
        &self,
        other: &Vertex,
        bounding_box: &Bounds,
    ) -> Result<f64, &'static str> {
        let edge_index1 = self.get_box_edge_index(bounding_box)?;
        let edge_index2 = other.get_box_edge_index(bounding_box)?;

        let side_length_y = bounding_box.max.y - bounding_box.min.y;
        let side_length_x = bounding_box.max.x - bounding_box.min.x;

        if edge_index1 == 0 {
            // ymax
            if edge_index2 == 0 {
                // ymax
                if other.x <= self.x {
                    return Ok(self.x - other.x);
                } else {
                    return Ok(self.x - bounding_box.min.x
                        + side_length_y
                        + side_length_x
                        + side_length_y
                        + bounding_box.max.x
                        - other.x);
                }
            } else if edge_index2 == 1 {
                // xmax
                return Ok(
                    self.x - bounding_box.min.x + side_length_y + side_length_x + other.y
                        - bounding_box.min.y,
                );
            } else if edge_index2 == 2 {
                // ymin
                return Ok(
                    self.x - bounding_box.min.x + side_length_y + other.x - bounding_box.min.x
                );
            } else if edge_index2 == 3 {
                // xmin
                return Ok(self.x - bounding_box.min.x + bounding_box.max.y - other.y);
            } else {
                return Err("Boundary distance error?");
            }
        } else if edge_index1 == 1 {
            // xmax
            if edge_index2 == 0 {
                return Ok(bounding_box.max.y - self.y + bounding_box.max.x - other.x);
            } else if edge_index2 == 1 {
                if other.y >= self.y {
                    return Ok(other.y - self.y);
                } else {
                    return Ok(bounding_box.max.y - self.y
                        + side_length_x
                        + side_length_y
                        + side_length_x
                        + other.y
                        - bounding_box.min.y);
                }
            } else if edge_index2 == 2 {
                return Ok(
                    bounding_box.max.y - self.y + side_length_x + side_length_y + other.x
                        - bounding_box.min.x,
                );
            } else if edge_index2 == 3 {
                return Ok(
                    bounding_box.max.y - self.y + side_length_x + bounding_box.max.y - other.y,
                );
            } else {
                return Err("Boundary distance error?");
            }
        } else if edge_index1 == 2 {
            // ymin
            if edge_index2 == 0 {
                return Ok(
                    bounding_box.max.x - self.x + side_length_y + bounding_box.max.x - other.x,
                );
            } else if edge_index2 == 1 {
                return Ok(bounding_box.max.x - self.x + other.y - bounding_box.min.y);
            } else if edge_index2 == 2 {
                if other.x >= self.x {
                    return Ok(other.x - self.x);
                } else {
                    return Ok(bounding_box.max.x - self.x
                        + side_length_y
                        + side_length_x
                        + side_length_y
                        + other.x
                        - bounding_box.min.x);
                }
            } else if edge_index2 == 3 {
                return Ok(bounding_box.max.x - self.x
                    + side_length_y
                    + side_length_x
                    + bounding_box.max.y
                    - other.y);
            } else {
                return Err("Boundary distance error?");
            }
        } else if edge_index1 == 3 {
            // xmin
            if edge_index2 == 0 {
                return Ok(self.y - bounding_box.min.y
                    + side_length_x
                    + side_length_y
                    + bounding_box.max.x
                    - other.x);
            } else if edge_index2 == 1 {
                return Ok(
                    self.y - bounding_box.min.y + side_length_x + other.y - bounding_box.min.y
                );
            } else if edge_index2 == 2 {
                return Ok(self.y - bounding_box.min.y + other.x - bounding_box.min.x);
            } else if edge_index2 == 3 {
                if other.y <= self.y {
                    return Ok(self.y - other.y);
                } else {
                    return Ok(self.y - bounding_box.min.y
                        + side_length_x
                        + side_length_y
                        + side_length_x
                        + bounding_box.max.y
                        - other.y);
                }
            } else {
                return Err("Boundary distance error?");
            }
        } else {
            return Err("Boundary distance error?");
        }
    }

    // checks if self lies on the border and returns which border edge self lies on or error
    pub fn get_box_edge_index(&self, bounding_box: &Bounds) -> Result<usize, &'static str> {
        if bounding_box.max.y == self.y {
            return Ok(0);
        } else if bounding_box.max.x == self.x {
            return Ok(1);
        } else if self.y == bounding_box.min.y {
            return Ok(2);
        } else if self.x == bounding_box.min.x {
            return Ok(3);
        } else {
            return Err("Vertex not on the boundary!");
        }
    }

    // Returns the orientation of consecutive segments ab and bc.
    pub fn consecutive_orientation(&self, b: &Vertex, c: &Vertex) -> f64 {
        let p1 = b - self;
        let p2 = c - self;
        return p1.cross_prod(&p2);
    }

    pub fn cross_prod(&self, other: &Vertex) -> f64 {
        return self.x * other.y - self.y * other.x;
    }

    pub fn squared_euclidean_distance(&self, other: &Vertex) -> f64 {
        return (self.x - other.x).powi(2) + (self.y - other.y).powi(2);
    }
}

impl Eq for Vertex {}

impl PartialEq for Vertex {
    fn eq(&self, other: &Vertex) -> bool {
        return self.id == other.id;
    }
}

impl Hash for Vertex {
    fn hash<H: Hasher>(&self, state: &mut H) {
        self.id.hash(state);
    }
}

impl Sub for Vertex {
    type Output = Vertex;

    fn sub(self, other: Self) -> Vertex {
        let x = self.x - other.x;
        let y = self.y - other.y;
        return Vertex { x, y, id: self.id };
    }
}

impl Add for Vertex {
    type Output = Vertex;

    fn add(self, other: Self) -> Vertex {
        let x = self.x + other.x;
        let y = self.y + other.y;
        return Vertex { x, y, id: self.id };
    }
}
