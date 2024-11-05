use super::{Coord, Line, Polygon};
pub use geo::LineString;
use geo::{BooleanOps, ClosestPoint, Distance, Euclidean};

pub trait MapLineString {
    fn first_vertex(&self) -> &Coord;
    fn last_vertex(&self) -> &Coord;
    fn line_string_signed_area(&self) -> Option<f64>;
    fn inner_line(&self, other: &LineString) -> Option<Polygon>;
    fn get_range_on_line(
        start: usize,
        end: usize,
        length: usize,
        is_closed: bool,
    ) -> Result<Vec<usize>, &'static str>;
    fn on_edge_index(&self, coord: &Coord, epsilon: f64) -> Option<usize>;
    fn close_by_line(&mut self, line: &LineString, epsilon: f64) -> Result<(), &'static str>;
    fn fix_ends_to_line(&mut self, line: &LineString, epsilon: f64) -> Result<(), &'static str>;
    fn get_distance_along_line(
        &self,
        start: &Coord,
        end: &Coord,
        epsilon: f64,
    ) -> Result<f64, &'static str>;
    fn append_by_line(
        &mut self,
        other: LineString,
        line: &LineString,
        epsilon: f64,
    ) -> Result<(), &'static str>;
    fn prepend(&mut self, other: LineString);
}

impl MapLineString for LineString {
    fn prepend(&mut self, mut other: LineString) {
        let a = self.0.drain(..);
        other.0.extend(a);
        self.0 = other.0;
    }

    fn line_string_signed_area(&self) -> Option<f64> {
        if self.0.len() < 3 || !self.is_closed() {
            return None;
        }
        let mut area: f64 = 0.;
        for i in 0..self.0.len() - 1 {
            area += 0.5 * (self.0[i].x * self.0[i + 1].y - self.0[i].y * self.0[i + 1].x);
        }
        Some(area)
    }

    fn get_distance_along_line(
        &self,
        start: &Coord,
        end: &Coord,
        epsilon: f64,
    ) -> Result<f64, &'static str> {
        let last_index = match self.on_edge_index(end, epsilon) {
            Some(i) => i,
            None => {
                return Err("Could not get distance along line as end coord is not on the line")
            }
        };
        let first_index = match self.on_edge_index(start, epsilon) {
            Some(i) => i,
            None => {
                return Err("Could not get distance along line as start coord is not on the line")
            }
        };

        if !self.is_closed() {
            if last_index < first_index {
                return Err("The end point is before the start point on the line");
            }

            if last_index == first_index {
                let prev_vertex = &self.0[first_index];

                if Euclidean::distance(*start, *prev_vertex)
                    > Euclidean::distance(*end, *prev_vertex)
                {
                    return Err("The end point is before the start point on the line");
                } else {
                    return Ok(Euclidean::distance(*start, *end));
                }
            }
        }

        if last_index == first_index {
            let prev_vertex = &self.0[first_index];

            if Euclidean::distance(*start, *prev_vertex) <= Euclidean::distance(*end, *prev_vertex)
            {
                return Ok(Euclidean::distance(*start, *end));
            }
        }

        let range =
            LineString::get_range_on_line(first_index, last_index, self.0.len(), self.is_closed())
                .unwrap();

        let mut dist = 0.;

        let mut prev_vertex = *start;
        for i in range {
            let next_vertex = self.0[i];

            dist += Euclidean::distance(prev_vertex, next_vertex);
            prev_vertex = next_vertex;
        }
        dist += Euclidean::distance(prev_vertex, *end);

        Ok(dist)
    }

    fn inner_line(&self, other: &LineString) -> Option<Polygon> {
        let p1 = Polygon::new(self.clone(), vec![]);
        let p2 = Polygon::new(other.clone(), vec![]);

        let multipolygon = p1.intersection(&p2);
        if multipolygon.0.is_empty() {
            None
        } else if multipolygon.0.len() == 1 {
            let mut i = multipolygon.into_iter().next().unwrap();

            if i.exterior().line_string_signed_area().unwrap() < 0. {
                i.exterior_mut(|e| e.0.reverse());
            }

            Some(i)
        } else {
            panic!("Multiple disjoint overlaps between the convex hull and the clipping region");
        }
    }

    fn close_by_line(&mut self, line: &LineString, epsilon: f64) -> Result<(), &'static str> {
        let first_vertex = self.first_vertex();
        let last_vertex = self.last_vertex();

        let last_index = match line.on_edge_index(last_vertex, epsilon) {
            Some(i) => i,
            None => {
                return Err("Could not close by line as last vertex of self is not on the line")
            }
        };
        let first_index = match line.on_edge_index(first_vertex, epsilon) {
            Some(i) => i,
            None => {
                return Err("Could not close by line as first vertex of self is not on the line")
            }
        };

        if last_index == first_index {
            let prev_vertex = &line.0[first_index];

            if Euclidean::distance(*last_vertex, *prev_vertex)
                <= Euclidean::distance(*first_vertex, *prev_vertex)
            {
                self.close();
                return Ok(());
            }
        }

        if !line.is_closed() && last_index >= first_index {
            return Err("The other point is before the first point on the open line");
        }

        for i in
            LineString::get_range_on_line(last_index, first_index, line.0.len(), line.is_closed())
                .unwrap()
        {
            self.0.push(line.0[i]);
        }
        self.close();

        Ok(())
    }

    fn get_range_on_line(
        start: usize,
        end: usize,
        length: usize,
        is_closed: bool,
    ) -> Result<Vec<usize>, &'static str> {
        if start < end {
            Ok((start + 1..=end).collect())
        } else if is_closed {
            let mut out = (start + 1..length - 1).collect::<Vec<usize>>();
            out.extend((0..=end).collect::<Vec<usize>>());
            Ok(out)
        } else {
            Err("Could not get range on line as the line is not closed and start is after end")
        }
    }

    fn first_vertex(&self) -> &Coord {
        &self.0[0]
    }

    fn append_by_line(
        &mut self,
        other: LineString,
        line: &LineString,
        epsilon: f64,
    ) -> Result<(), &'static str> {
        let last_self = self.last_vertex();
        let first_other = other.first_vertex();

        let self_index = match line.on_edge_index(last_self, epsilon) {
            Some(i) => i,
            None => {
                return Err("Could not append by line as last vertex of self is not on the line")
            }
        };
        let other_index = match line.on_edge_index(first_other, epsilon) {
            Some(i) => i,
            None => {
                return Err("Could not append by line as first vertex of other is not on the line")
            }
        };

        if self_index == other_index {
            let prev_vertex = &line.0[self_index];

            if Euclidean::distance(*last_self, *prev_vertex)
                <= Euclidean::distance(*first_other, *prev_vertex)
            {
                self.0.extend(other.0);
                return Ok(());
            }
        }

        let range =
            LineString::get_range_on_line(self_index, other_index, line.0.len(), line.is_closed())?;

        for i in range {
            self.0.push(line.0[i]);
        }
        self.0.extend(other.0);
        Ok(())
    }

    fn last_vertex(&self) -> &Coord {
        &self.0[self.0.len() - 1]
    }

    fn on_edge_index(&self, point: &Coord, epsilon: f64) -> Option<usize> {
        for (i, line) in self.lines().enumerate() {
            if Euclidean::distance(&line, *point) <= epsilon {
                return Some(i);
            }
        }
        None
    }

    fn fix_ends_to_line(&mut self, line: &LineString, epsilon: f64) -> Result<(), &'static str> {
        if self.is_closed() {
            return Ok(());
        } else if Euclidean::distance(*self.last_vertex(), *self.first_vertex()) < epsilon {
            self.close();
            return Ok(());
        }

        let first_index = line
            .on_edge_index(self.first_vertex(), epsilon)
            .expect("coordinate is not on the line");
        let last_index = line
            .on_edge_index(self.last_vertex(), epsilon)
            .expect("coordinate is not on the line");

        let first_line = Line::new(
            line.0[first_index],
            line.0[(first_index + 1) % line.0.len()],
        );
        let last_line = Line::new(line.0[last_index], line.0[(last_index + 1) % line.0.len()]);

        match first_line.closest_point(&(*self.first_vertex()).into()) {
            geo::Closest::Indeterminate => {
                return Err("no unique closest point on the line when fixing ends to line")
            }
            geo::Closest::Intersection(_) => (),
            geo::Closest::SinglePoint(p) => self.0.insert(0, p.0),
        };
        match last_line.closest_point(&(*self.last_vertex()).into()) {
            geo::Closest::Indeterminate => {
                return Err("no unique closest point on the line when fixing ends to line")
            }
            geo::Closest::Intersection(_) => (),
            geo::Closest::SinglePoint(p) => self.0.push(p.0),
        };
        Ok(())
    }
}

#[cfg(test)]
mod tests {
    use geo::Area;

    use super::super::*;

    #[test]
    fn test_range_on_line_overlap() {
        let range = <LineString as MapLineString>::get_range_on_line(2, 1, 5, true).unwrap();
        let expected = vec![3, 0, 1];
        assert_eq!(range, expected);
    }

    #[test]
    fn test_range_on_line_simple() {
        let range = <LineString as MapLineString>::get_range_on_line(2, 3, 5, false).unwrap();
        let expected = vec![3];
        assert_eq!(range, expected);
    }

    #[test]
    fn test_distance_along_line_simple() {
        let line = LineString::new(vec![
            Coord { x: 0., y: 100. },
            Coord { x: 0., y: 0. },
            Coord { x: 100., y: 0. },
            Coord { x: 100., y: 100. },
            Coord { x: 0., y: 100. },
        ]);

        let v1 = Coord { x: 20., y: 0. };
        let v2 = Coord { x: 10., y: 100. };

        let d = line.get_distance_along_line(&v1, &v2, 1.);

        assert_eq!(d, Ok(270.));
    }

    #[test]
    fn test_distance_along_line_overlap() {
        let line = LineString::new(vec![
            Coord { x: 0., y: 100. },
            Coord { x: 0., y: 0. },
            Coord { x: 100., y: 0. },
            Coord { x: 100., y: 100. },
            Coord { x: 0., y: 100. },
        ]);

        let v1 = Coord { x: 20., y: 0. };
        let v2 = Coord { x: 10., y: 100. };

        let d = line.get_distance_along_line(&v2, &v1, 1.);

        assert_eq!(d, Ok(130.));
    }

    #[test]
    fn test_on_edge_index() {
        let line = LineString::new(vec![
            Coord { x: 0., y: 100. },
            Coord { x: 0., y: 0. },
            Coord { x: 100., y: 0. },
            Coord { x: 100., y: 100. },
            Coord { x: 0., y: 100. },
        ]);

        let v1 = Coord { x: 20., y: 100.9 };

        let i = line.on_edge_index(&v1, 1.);

        assert_eq!(i, Some(3));
    }

    #[test]
    fn test_signed_area_linestring() {
        let line = LineString::new(vec![
            Coord { x: 0., y: 100. },
            Coord { x: 0., y: 0. },
            Coord { x: 100., y: 0. },
            Coord { x: 100., y: 100. },
            Coord { x: 0., y: 100. },
        ]);

        assert_eq!(Some(10_000.), line.line_string_signed_area());
    }

    #[test]
    fn test_geo_agreed_signed_area_linestring() {
        let line = LineString::new(vec![
            Coord { x: 0., y: 100. },
            Coord { x: 0., y: 0. },
            Coord { x: 100., y: 0. },
            Coord { x: 100., y: 100. },
            Coord { x: 0., y: 100. },
        ]);

        let own_area = line.line_string_signed_area().unwrap();

        let polygon = Polygon::new(line, vec![]);

        assert_eq!(own_area, polygon.signed_area());
    }

    #[test]
    fn test_fix_ends_to_line() {
        let hull = LineString::new(vec![
            Coord { x: 0., y: 100. },
            Coord { x: 0., y: 0. },
            Coord { x: 100., y: 0. },
            Coord { x: 100., y: 100. },
            Coord { x: 0., y: 100. },
        ]);

        let mut cont = LineString::new(vec![
            Coord { x: 12.2, y: 100.5 },
            Coord { x: 50., y: 50. },
            Coord { x: 67.1, y: 0.7 },
        ]);

        cont.fix_ends_to_line(&hull, 1.0).unwrap();

        let expected = LineString::new(vec![
            Coord { x: 12.2, y: 100. },
            Coord { x: 12.2, y: 100.5 },
            Coord { x: 50., y: 50. },
            Coord { x: 67.1, y: 0.7 },
            Coord { x: 67.1, y: 0.0 },
        ]);

        assert!((cont.0[0] - expected.0[0]).y.abs() < 0.001);
        assert!((cont.0[4] - expected.0[4]).y.abs() < 0.001);
    }
}
