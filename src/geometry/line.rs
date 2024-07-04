use super::{Point, Point2D};

use std::hash::{Hash, Hasher};

#[derive(Clone, Debug)]
pub struct Line {
    pub vertices: Vec<Point2D>,
}

impl Line {
    pub fn new(vert1: Point2D, vert2: Point2D) -> Line {
        Line {
            vertices: vec![vert1, vert2],
        }
    }

    pub fn is_closed(&self) -> bool {
        self.first_vertex() == self.last_vertex()
    }

    pub fn push(&mut self, vert: Point2D) {
        self.vertices.push(vert);
    }

    pub fn pop(&mut self) {
        self.vertices.pop();
    }

    pub fn close(&mut self) {
        if !self.is_closed() {
            self.vertices.push(*self.first_vertex());
        }
    }

    pub fn close_by_hull(&mut self, convex_hull: &Line) -> Result<(), &'static str> {
        let first_vertex = self.first_vertex();
        let last_vertex = self.last_vertex();

        let length = convex_hull.len();

        let last_index = last_vertex.on_edge_index(&convex_hull)?;
        let first_index = first_vertex.on_edge_index(&convex_hull)?;

        if last_index == first_index {
            let prev_vertex = &convex_hull.vertices[first_index];

            if last_vertex.squared_euclidean_distance(prev_vertex)
                <= first_vertex.squared_euclidean_distance(prev_vertex)
            {
                self.close();
                return Ok(());
            }
        }

        for i in Line::get_ccw_range(last_index, first_index, length) {
            self.vertices.push(convex_hull.vertices[i]);
        }
        self.close();

        Ok(())
    }

    fn get_ccw_range(last_index: usize, first_index: usize, length: usize) -> Vec<usize> {
        if last_index < first_index {
            (last_index + 1..first_index + 1).collect()
        } else {
            let mut out = (last_index + 1..length).collect::<Vec<usize>>();
            out.append(&mut (0..first_index + 1).collect::<Vec<usize>>());
            out
        }
    }

    pub fn contains(&self, point: &Point2D) -> Result<bool, &'static str> {
        if !self.is_closed() {
            return Err("Containment undefined for unclosed contour");
        }

        let mut intersection_count = 0;
        for i in 0..self.vertices.len() - 1 {
            let vertex1 = &self.vertices[i];
            let vertex2 = &self.vertices[i + 1];

            if (point.y <= vertex2.y && point.y > vertex1.y)
                || (point.y >= vertex2.y && point.y < vertex1.y)
            {
                if vertex1.y == vertex2.y {
                    continue;
                } else if point.x
                    < ((point.y - vertex1.y) / (vertex2.y - vertex1.y)) * (vertex2.x - vertex1.x)
                        + vertex1.x
                {
                    intersection_count += 1;
                }
            }
        }
        return Ok(intersection_count % 2 != 0);
    }

    pub fn first_vertex(&self) -> &Point2D {
        &self.vertices[0]
    }

    pub fn append(&mut self, other: Line) {
        self.vertices.extend(other.vertices);
    }

    pub fn append_by_hull(&mut self, other: Line, convex_hull: &Line) {
        a
    }

    pub fn last_vertex(&self) -> &Point2D {
        &self.vertices[self.len() - 1]
    }

    pub fn len(&self) -> usize {
        self.vertices.len()
    }

    pub fn prepend(&mut self, vert: Point2D) {
        let mut verts = vec![vert];
        verts.append(&mut self.vertices);
        self.vertices = verts;
    }

    pub fn signed_area(&self) -> Result<f64, &'static str> {
        if !self.is_closed() {
            return Err("Cannot compute area of unclosed contour");
        }
        let mut area: f64 = 0.;
        for i in 0..self.len() - 1 {
            area += 0.5
                * (self.vertices[i].x * self.vertices[i + 1].y
                    - self.vertices[i].y * self.vertices[i + 1].x);
        }
        return Ok(area);
    }
}

impl Eq for Line {}

impl PartialEq for Line {
    fn eq(&self, other: &Line) -> bool {
        self.first_vertex() == other.last_vertex() && self.last_vertex() == other.first_vertex()
    }
}

impl Hash for Line {
    fn hash<H: Hasher>(&self, state: &mut H) {
        self.first_vertex().hash(state);
        self.last_vertex().hash(state);
    }
}
