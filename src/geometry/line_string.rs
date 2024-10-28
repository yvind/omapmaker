#![allow(dead_code)]

use super::{Line, Point, Point2D, Rectangle};

#[derive(Clone, Debug)]
pub struct LineString {
    pub vertices: Vec<Point2D>,
}

impl LineString {
    pub fn new(vert1: Point2D, vert2: Point2D) -> LineString {
        LineString {
            vertices: vec![vert1, vert2],
        }
    }

    fn default() -> LineString {
        LineString {
            vertices: Vec::new(),
        }
    }

    pub fn bounding_box(&self) -> Rectangle {
        let mut min_x = f64::MAX;
        let mut min_y = f64::MAX;
        let mut max_x = f64::MIN;
        let mut max_y = f64::MIN;

        self.vertices.iter().for_each(|p| {
            if p.x > max_x {
                max_x = p.x
            } else if p.x < min_x {
                min_x = p.x
            }
            if p.y > max_y {
                max_y = p.y
            } else if p.y < min_y {
                min_y = p.y
            }
        });
        Rectangle {
            min: Point2D::new(min_x, min_y),
            max: Point2D::new(max_x, max_y),
        }
    }

    pub fn is_closed(&self) -> bool {
        self.first_vertex() == self.last_vertex()
    }

    pub fn is_empty(&self) -> bool {
        self.vertices.is_empty()
    }

    pub fn push(&mut self, vert: Point2D) {
        self.vertices.push(vert);
    }

    pub fn pop(&mut self) {
        self.vertices.pop();
    }

    pub fn inner_line(&self, other: &LineString) -> LineString {
        let mut self_in_other = self.clone().clip(other);
        if self_in_other.len() == 1 && self_in_other[0].is_closed() {
            return self_in_other.remove(0);
        }

        let mut other_in_self = other.clone().clip(self);
        if other_in_self.len() == 1 && other_in_self[0].is_closed() {
            return other_in_self.remove(0);
        }

        assert!(self_in_other.len() > 0 && other_in_self.len() > 0);
        assert!((self_in_other.len() as isize - other_in_self.len() as isize).abs() <= 1);

        let mut result = LineString::default();

        let min_length = self_in_other.len().min(other_in_self.len());

        if self_in_other[0].last_vertex() == other_in_self[0].first_vertex() {
            for (l1, l2) in self_in_other
                .drain(0..min_length)
                .zip(other_in_self.drain(0..min_length))
            {
                result.append(l1);
                result.append(l2);
            }
        } else if other_in_self[0].last_vertex() == self_in_other[0].first_vertex() {
            for (l2, l1) in self_in_other
                .drain(0..min_length)
                .zip(other_in_self.drain(0..min_length))
            {
                result.append(l1);
                result.append(l2);
            }
        } else {
            panic!("Error in inner line finding logic");
        }
        if self_in_other.len() == 1 {
            result.append(self_in_other.remove(0));
        } else if other_in_self.len() == 1 {
            result.append(other_in_self.remove(0));
        }

        assert!(result.is_closed());
        result
    }

    pub fn intersections(&self, line: &Line) -> Vec<Point2D> {
        let mut result = Vec::new();

        for window in self.vertices.windows(2) {
            if let Some(intersection) = line.intersection(&window.into()) {
                result.push(intersection);
            }
        }
        result
    }

    pub fn clip(self, overlay: &LineString) -> Vec<LineString> {
        assert!(
            overlay.is_closed(),
            "Overlay in clip function is not closed"
        );
        assert!(
            self.vertices.len() > 1,
            "Degenerate line encountered in clip function"
        );

        let mut result = Vec::new();
        let mut current_line = LineString::default();

        let mut prev_point = self.first_vertex();
        let mut prev_inside = overlay.contains(prev_point).unwrap();

        if prev_inside {
            current_line.push(*prev_point);
        }

        for next_point in self.vertices.iter().skip(1) {
            let next_inside = overlay.contains(next_point).unwrap();

            let intersections = overlay.intersections(&[prev_point, next_point].into());

            for (i, intersection) in intersections.into_iter().enumerate() {
                if i % 2 == prev_inside as usize {
                    // begining of line segment
                    current_line.push(intersection);
                } else {
                    // end of line segment
                    current_line.push(intersection);
                    result.push(current_line);
                    current_line = LineString::default();
                }
            }

            if next_inside {
                current_line.push(*next_point);
            }

            prev_inside = next_inside;
            prev_point = next_point;
        }

        if !current_line.is_empty() {
            result.push(current_line);
        }

        result
    }

    pub fn close(&mut self) {
        if !self.is_closed() {
            self.vertices.push(*self.first_vertex());
        }
    }

    pub fn close_by_line(&mut self, line: &LineString, epsilon: f64) -> Result<(), &'static str> {
        let first_vertex = self.first_vertex();
        let last_vertex = self.last_vertex();

        let last_index = last_vertex.on_edge_index(line, epsilon)?;
        let first_index = first_vertex.on_edge_index(line, epsilon)?;

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

        for i in LineString::get_range_on_line(last_index, first_index, line.len()) {
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
            return Err("Containment undefined for unclosed line");
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
        Ok(intersection_count % 2 != 0)
    }

    pub fn first_vertex(&self) -> &Point2D {
        &self.vertices[0]
    }

    pub fn append(&mut self, other: LineString) {
        self.vertices.extend(other.vertices);
    }

    pub fn append_by_line(
        &mut self,
        other: LineString,
        line: &LineString,
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
        Ok(area)
    }

    pub fn simplify(&mut self, epsilon: f64) {
        let mut i = 1;
        while i < self.len() - 1 {
            if self.vertices[i].dist_to_line_segment_squared(
                &[&self.vertices[i - 1], &self.vertices[i + 1]].into(),
            ) < epsilon.powi(2)
            {
                self.vertices.remove(i);
            } else {
                i += 1;
            }
        }

        if self.is_closed()
            && self.vertices[0]
                .dist_to_line_segment_squared(&[&self.vertices[i - 2], &self.vertices[1]].into())
                < epsilon.powi(2)
        {
            self.vertices.remove(0);
            self.vertices.remove(self.vertices.len() - 1);
            self.close();
        }
    }

    pub fn fix_ends_to_line(&mut self, line: &LineString, epsilon: f64) {
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
            &[
                &line.vertices[first_index],
                &line.vertices[(first_index + 1) % line.len()],
            ]
            .into(),
        );
        let last_add = self.last_vertex().closest_point_on_line_segment(
            &[
                &line.vertices[last_index],
                &line.vertices[(last_index + 1) % line.len()],
            ]
            .into(),
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

            let n1 = (&this_vertex - &prev_vertex).normal();
            let n2 = (&next_vertex - &this_vertex).normal();

            let mut normal = &n1 + &n2;
            normal.norm();
            normal.scale(epsilon);

            normals.push(normal);
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

            let n1 = (&this_vertex - &prev_vertex).normal();
            let n2 = (&next_vertex - &this_vertex).normal();

            let mut normal = &n1 + &n2;
            normal.norm();
            normal.scale(epsilon);

            normals.push(normal);
            prev_vertex = this_vertex;
        }
        normals.push(normals[0]);

        for (i, p) in self.vertices.iter_mut().enumerate() {
            p.x -= normals[i].x;
            p.y -= normals[i].y;
        }
    }
}

impl Eq for LineString {}

impl PartialEq for LineString {
    fn eq(&self, other: &LineString) -> bool {
        self.first_vertex() == other.first_vertex() && self.last_vertex() == other.last_vertex()
    }
}

impl From<Rectangle> for LineString {
    fn from(r: Rectangle) -> Self {
        LineString {
            vertices: vec![
                Point2D::new(r.min.x, r.max.y),
                Point2D::new(r.min.x, r.min.y),
                Point2D::new(r.max.x, r.min.y),
                Point2D::new(r.max.x, r.max.y),
                Point2D::new(r.min.x, r.max.y),
            ],
        }
    }
}

#[cfg(test)]
mod test {
    use super::*;

    #[test]
    fn test_fix_to_line() {
        let mut line = LineString::new(Point2D::new(-1., -1.), Point2D::new(1., -1.));

        line.push(Point2D::new(1., 1.));
        line.push(Point2D::new(-1., 1.));
        line.push(Point2D::new(-1., -1.));

        let mut contour = LineString::new(Point2D::new(-0.95, 0.), Point2D::new(0., 0.));
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
