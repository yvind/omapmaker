use super::{Point, Point2D};

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
            self.vertices.push(self.first_vertex().clone());
        }
    }

    pub fn close_by_line(&mut self, line: &Line, epsilon: f64) -> Result<(), &'static str> {
        let first_vertex = self.first_vertex();
        let last_vertex = self.last_vertex();

        let last_index = last_vertex.on_edge_index(&line, epsilon)?;
        let first_index = first_vertex.on_edge_index(&line, epsilon)?;

        if last_index == first_index {
            let prev_vertex = &line.vertices[first_index];

            if last_vertex.squared_euclidean_distance(prev_vertex)
                <= first_vertex.squared_euclidean_distance(prev_vertex)
            {
                self.close();
                return Ok(());
            }
        }

        if !line.is_closed() && last_index >= first_index {
            return Err("The other point is before the first point on the open line");
        }

        for i in Line::get_range_on_line(last_index, first_index, line.len()) {
            self.vertices.push(line.vertices[i]);
        }
        self.close();

        Ok(())
    }

    pub fn get_range_on_line(last_index: usize, first_index: usize, length: usize) -> Vec<usize> {
        if last_index < first_index {
            (last_index + 1..first_index + 1).collect()
        } else {
            let mut out = (last_index + 1..length - 1).collect::<Vec<usize>>();
            out.extend((0..first_index + 1).collect::<Vec<usize>>());
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

    pub fn append_by_line(
        &mut self,
        other: Line,
        line: &Line,
        epsilon: f64,
    ) -> Result<(), &'static str> {
        let last_self = self.last_vertex();
        let first_other = other.first_vertex();

        let self_index = last_self.on_edge_index(line, epsilon)?;
        let other_index = first_other.on_edge_index(line, epsilon)?;

        if self_index == other_index {
            let prev_vertex = &line.vertices[self_index];

            if last_self.squared_euclidean_distance(prev_vertex)
                <= first_other.squared_euclidean_distance(prev_vertex)
            {
                self.append(other);
                return Ok(());
            }
        }

        if !line.is_closed() && self_index >= other_index {
            return Err("The other point is before the first point on the open line");
        }

        let range = Self::get_range_on_line(self_index, other_index, line.len());

        for i in range {
            self.push(line.vertices[i]);
        }
        self.append(other);
        Ok(())
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

    pub fn simplify(&mut self, epsilon: f64) {
        let mut i = 1;
        while i < self.len() - 1 {
            if self.vertices[i]
                .dist_to_line_segment_squared(&self.vertices[i - 1], &self.vertices[i + 1])
                < epsilon.powi(2)
            {
                self.vertices.remove(i);
            } else {
                i += 1;
            }
        }

        if self.is_closed() {
            if self.vertices[0].dist_to_line_segment_squared(
                &self.vertices[self.vertices.len() - 2],
                &self.vertices[1],
            ) < epsilon.powi(2)
            {
                self.vertices.remove(0);
                self.vertices.remove(self.vertices.len() - 1);
                self.close();
            }
        }
    }

    pub fn fix_ends_to_line(&mut self, line: &Line, epsilon: f64) {
        if self.is_closed() {
            return;
        } else if self
            .last_vertex()
            .squared_euclidean_distance(self.first_vertex())
            < epsilon * epsilon
        {
            self.close();
            return;
        }

        let first_index = self.first_vertex().on_edge_index(line, epsilon).unwrap();
        let last_index = self.last_vertex().on_edge_index(line, epsilon).unwrap();

        let first_add = self.first_vertex().closest_point_on_line_segment(
            &line.vertices[first_index],
            &line.vertices[(first_index + 1) % line.len()],
        );
        let last_add = self.last_vertex().closest_point_on_line_segment(
            &line.vertices[last_index],
            &line.vertices[(last_index + 1) % line.len()],
        );

        self.prepend(first_add);
        self.push(last_add);
    }

    pub fn dilate(&mut self, epsilon: f64) {
        if !self.is_closed() {
            return;
        }

        let len = self.len();

        let mut normals = Vec::with_capacity(len);

        let mut prev_vertex = self.vertices[len - 2];
        for i in 0..len - 1 {
            let this_vertex = self.vertices[i];
            let next_vertex = self.vertices[i + 1];

            let n1 = (this_vertex - prev_vertex).normal();
            let n2 = (next_vertex - this_vertex).normal();

            let normal = (n1 + n2).norm();

            normals.push(normal.scale(epsilon));
            prev_vertex = this_vertex;
        }
        normals.push(normals[0]);

        for (i, p) in self.vertices.iter_mut().enumerate() {
            p.x += normals[i].x;
            p.y += normals[i].y;
        }
    }

    pub fn erode(&mut self, epsilon: f64) {
        if !self.is_closed() {
            return;
        }

        let len = self.len();

        let mut normals = Vec::with_capacity(len);

        let mut prev_vertex = self.vertices[len - 2];
        for i in 0..len - 1 {
            let this_vertex = self.vertices[i];
            let next_vertex = self.vertices[i + 1];

            let n1 = (this_vertex - prev_vertex).normal();
            let n2 = (next_vertex - this_vertex).normal();

            let normal = (n1 + n2).norm();

            normals.push(normal.scale(epsilon));
            prev_vertex = this_vertex;
        }
        normals.push(normals[0]);

        for (i, p) in self.vertices.iter_mut().enumerate() {
            p.x -= normals[i].x;
            p.y -= normals[i].y;
        }
    }
}

impl Eq for Line {}

impl PartialEq for Line {
    fn eq(&self, other: &Line) -> bool {
        self.first_vertex() == other.first_vertex() && self.last_vertex() == other.last_vertex()
    }
}

#[cfg(test)]
mod test {
    use super::*;

    #[test]
    fn test_fix_to_line() {
        let mut line = Line::new(Point2D::new(-1., -1.), Point2D::new(1., -1.));

        line.push(Point2D::new(1., 1.));
        line.push(Point2D::new(-1., 1.));
        line.push(Point2D::new(-1., -1.));

        let mut contour = Line::new(Point2D::new(-0.95, 0.), Point2D::new(0., 0.));
        contour.push(Point2D::new(0., 0.95));

        contour.fix_ends_to_line(&line, 0.1);

        assert!(
            contour
                .first_vertex()
                .squared_euclidean_distance(&Point2D::new(-1., 0.))
                < 0.05 * 0.05
        );
        assert!(
            contour
                .last_vertex()
                .squared_euclidean_distance(&Point2D::new(0., 1.))
                < 0.05 * 0.05
        );
    }
}
