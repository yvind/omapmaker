use super::{Coord, Line, Polygon};
pub use geo::LineString;
use geo::{BooleanOps, ClosestPoint, EuclideanDistance}; //, Distance, Euclidean};

pub trait MapLineString {
    fn first_vertex(&self) -> &Coord;
    fn last_vertex(&self) -> &Coord;
    fn inner_line(&self, other: &LineString) -> Option<Polygon>;
    fn get_range_on_line(last_index: usize, first_index: usize, length: usize) -> Vec<usize>;
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
}

impl MapLineString for LineString {
    fn get_distance_along_line(
        &self,
        start: &Coord,
        end: &Coord,
        epsilon: f64,
    ) -> Result<f64, &'static str> {
        let length = self.0.len();

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
            if last_index > first_index {
                return Err("The end point is before the start point on the line");
            }

            if last_index == first_index {
                let prev_vertex = &self.0[first_index];

                if start.euclidean_distance(prev_vertex) > end.euclidean_distance(prev_vertex)
                //Euclidean::distance(*start, *prev_vertex) > Euclidean::distance(*end, *prev_vertex)
                {
                    return Err("The end point is before the start point on the line");
                }
            }
        }

        if last_index == first_index {
            let prev_vertex = &self.0[first_index];

            if start.euclidean_distance(prev_vertex) <= end.euclidean_distance(prev_vertex)
            //Euclidean::distance(*start, *prev_vertex) <= Euclidean::distance(*end, *prev_vertex)
            {
                return Ok(start.euclidean_distance(end)); //Euclidean::distance(*start, *end));
            }
        }

        let range = LineString::get_range_on_line(last_index, first_index, length);

        let mut dist = 0.;

        let mut prev_vertex = *start;
        for i in range {
            let next_vertex = self.0[i];

            dist += prev_vertex.euclidean_distance(&next_vertex); //Euclidean::distance(prev_vertex, next_vertex);
            prev_vertex = next_vertex;
        }
        dist += prev_vertex.euclidean_distance(end); //Euclidean::distance(prev_vertex, *end);

        Ok(dist)
    }

    fn inner_line(&self, other: &LineString) -> Option<Polygon> {
        let p1 = Polygon::new(self.clone(), vec![]);
        let p2 = Polygon::new(other.clone(), vec![]);

        let multipolygon = p1.intersection(&p2);
        if multipolygon.0.is_empty() {
            None
        } else if multipolygon.0.len() == 1 {
            Some(multipolygon.into_iter().next().unwrap())
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

            if last_vertex.euclidean_distance(prev_vertex)
                <= first_vertex.euclidean_distance(prev_vertex)
            //Euclidean::distance(*last_vertex, *prev_vertex) <= Euclidean::distance(*first_vertex, *prev_vertex)
            {
                self.close();
                return Ok(());
            }
        }

        if !line.is_closed() && last_index >= first_index {
            return Err("The other point is before the first point on the open line");
        }

        for i in LineString::get_range_on_line(last_index, first_index, line.0.len()) {
            self.0.push(line.0[i]);
        }
        self.close();

        Ok(())
    }

    fn get_range_on_line(last_index: usize, first_index: usize, length: usize) -> Vec<usize> {
        if last_index < first_index {
            (last_index + 1..first_index + 1).collect()
        } else {
            let mut out = (last_index + 1..length - 1).collect::<Vec<usize>>();
            out.extend((0..first_index + 1).collect::<Vec<usize>>());
            out
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

            if last_self.euclidean_distance(prev_vertex)
                <= first_other.euclidean_distance(prev_vertex)
            //Euclidean::distance(*last_self, *prev_vertex) <= Euclidean::distance(*first_other, *prev_vertex)
            {
                self.0.extend(other.0);
                return Ok(());
            }
        }

        if !line.is_closed() && self_index >= other_index {
            return Err("The other point is before the first point on the open line");
        }

        let range = Self::get_range_on_line(self_index, other_index, line.0.len());

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
            if line.euclidean_distance(point) < epsilon {
                //Euclidean::distance(&line, *point) < epsilon {
                return Some(i);
            }
        }
        None
    }

    fn fix_ends_to_line(&mut self, line: &LineString, epsilon: f64) -> Result<(), &'static str> {
        if self.is_closed() {
            return Ok(());
        } else if self.last_vertex().euclidean_distance(self.first_vertex()) < epsilon {
            //Euclidean::distance(*self.last_vertex(), *self.first_vertex()) < epsilon {
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

        let first_add = match first_line.closest_point(&(*self.first_vertex()).into()) {
            geo::Closest::Indeterminate => {
                return Err("no unique closest point on the line when fixing ends to line")
            }
            geo::Closest::Intersection(p) => p,
            geo::Closest::SinglePoint(p) => p,
        };
        let last_add = match last_line.closest_point(&(*self.last_vertex()).into()) {
            geo::Closest::Indeterminate => {
                return Err("no unique closest point on the line when fixing ends to line")
            }
            geo::Closest::Intersection(p) => p,
            geo::Closest::SinglePoint(p) => p,
        };

        self.0.insert(0, first_add.0);
        self.0.push(last_add.0);
        Ok(())
    }
}
