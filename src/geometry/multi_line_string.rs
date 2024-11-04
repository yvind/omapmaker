use super::{LineString, MapLineString};
pub use geo::{Distance, Euclidean, MultiLineString};

pub trait MapMultiLineString {
    fn fix_ends_to_line(&mut self, hull: &LineString, epsilon: f64);
    fn merge(self, bound: &LineString) -> MultiLineString;
}

impl MapMultiLineString for MultiLineString {
    fn fix_ends_to_line(&mut self, hull: &LineString, epsilon: f64) {
        for c in self.iter_mut() {
            c.fix_ends_to_line(hull, epsilon).unwrap();
        }
    }

    fn merge(mut self, bound: &LineString) -> MultiLineString {
        let mut merge_indecies = Vec::new();
        let mut needs_merging = Vec::with_capacity(8);

        let epsilon = 0.01;

        for (i, part) in self.0.iter().enumerate() {
            if !part.is_closed()
                && (bound.on_edge_index(part.first_vertex(), epsilon).is_none()
                    || bound.on_edge_index(part.last_vertex(), epsilon).is_none())
            {
                merge_indecies.push(i);
            }
        }

        for i in merge_indecies.into_iter().rev() {
            needs_merging.push(self.0.swap_remove(i));
        }

        if needs_merging.len() % 2 == 1 {
            println!("merging len: {}", needs_merging.len());
            println!("error causing: {:?}", needs_merging[0]);
            println!("bounds: {:?}", bound);
            panic!("merge error");
        }

        while !needs_merging.is_empty() {
            let mut mergee = needs_merging.swap_remove(0);
            let mut append = usize::MAX;
            let mut prepend = usize::MAX;

            for (j, other) in needs_merging.iter().enumerate() {
                if Euclidean::distance(*mergee.last_vertex(), *other.first_vertex()) < epsilon {
                    append = j;
                    break;
                }
                if Euclidean::distance(*mergee.first_vertex(), *other.last_vertex()) < epsilon {
                    prepend = j;
                    break;
                }
            }

            if append != usize::MAX {
                mergee.0.pop();
                mergee.0.extend(needs_merging.swap_remove(append));
            } else if prepend != usize::MAX {
                let mut pre = needs_merging.swap_remove(prepend);
                pre.0.pop();
                mergee.prepend(pre);
            } else {
                panic!("The line needs merging but there is no suitable linestring to merge to");
            }
            self.0.push(mergee);
        }

        self
    }
}

#[cfg(test)]
mod test {
    use super::super::*;

    #[test]
    fn test_merge() {
        let before = MultiLineString::new(vec![
            LineString::new(vec![Coord { x: 1., y: 40. }, Coord { x: 0., y: 40. }]),
            LineString::new(vec![
                Coord { x: 0., y: 30. },
                Coord { x: 1., y: 30. },
                Coord { x: 1., y: 40. },
            ]),
        ]);

        let bound = LineString::new(vec![
            Coord { x: 0., y: 0. },
            Coord { x: 100., y: 0. },
            Coord { x: 100., y: 100. },
            Coord { x: 0., y: 100. },
            Coord { x: 0., y: 0. },
        ]);

        let after = before.merge(&bound);

        let expected = MultiLineString::new(vec![LineString::new(vec![
            Coord { x: 0., y: 30. },
            Coord { x: 1., y: 30. },
            Coord { x: 1., y: 40. },
            Coord { x: 0., y: 40. },
        ])]);

        assert_eq!(expected, after);
    }
}
