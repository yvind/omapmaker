use geo::{BooleanOps, Vector2DOps};
use geo::{LineString, Polygon};

pub trait MapLineString {
    fn line_string_signed_area(&self) -> Option<f64>;
    fn inner_line(&self, other: &LineString) -> Option<Polygon>;
    fn adjusted_bending_force(&self, length_exp: i32) -> f64;
}

impl MapLineString for LineString {
    fn line_string_signed_area(&self) -> Option<f64> {
        if self.0.len() < 3 || !self.is_closed() {
            return None;
        }
        let mut area: f64 = 0.;
        for i in 0..self.0.len() - 1 {
            area += self.0[i].x * self.0[i + 1].y - self.0[i].y * self.0[i + 1].x;
        }
        Some(0.5 * area)
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

    fn adjusted_bending_force(&self, lenght_exp: i32) -> f64 {
        if self.0.len() < 3 {
            return 1000.; // Not enough vertices to calculate bending energy, should be punished hard
        }

        let mut lines = self.lines().map(|l| l.delta());

        // store first line for use if the linestring is closed
        let first_line = lines.next().unwrap();
        let first_len = first_line.magnitude();
        let norm_first = first_line / first_len;

        let mut prev_len = first_len;
        let mut norm_prev = first_line / first_len;

        let mut total_energy = 0.0;
        let mut tot_len = first_len;

        for cur_line in lines {
            // Calculate lengths
            let cur_len = cur_line.magnitude();
            let norm_cur = cur_line / cur_len;

            // Calculate dot product of unit vectors, should equal cos(theta)
            let dot_product = norm_cur.dot_product(norm_prev).clamp(-1., 1.);

            // Calculate turning angle
            let turning_angle = dot_product.acos();

            // Calculate average segment length at this vertex
            let avg_length = (prev_len + cur_len) / 2.;

            // Add contribution to total energy
            total_energy += turning_angle.powi(2) / avg_length;
            tot_len += cur_len;

            prev_len = cur_len;
            norm_prev = norm_cur;
        }

        // add the final contribution if the linestring is closed
        if self.is_closed() {
            let dot_product = norm_first.dot_product(norm_prev).clamp(-1., 1.);
            let turning_angle = dot_product.acos();

            let avg_length = (first_len + prev_len) / 2.;
            total_energy += turning_angle.powi(2) / avg_length;
        }

        total_energy / tot_len.powi(lenght_exp)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use geo::{Area, Coord, LineString, Polygon};

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
}
