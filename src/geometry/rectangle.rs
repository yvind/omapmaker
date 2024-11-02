use super::{Coord, Line, LineString, MultiLineString};
pub use geo::Rect as Rectangle;
use geo::{
    line_intersection::line_intersection, Contains, Distance, Euclidean, Intersects,
    LineIntersection,
};
use las::Bounds;

pub trait MapRectangle {
    fn shrink(&mut self, v: f64);
    fn from_bounds(value: Bounds) -> Rectangle;
    fn into_line_string(self) -> LineString;
    fn touch_margin(&self, other: &Rectangle, margin: f64) -> bool;
    // functions below become obsolete when i_overlay in geo is fixed
    fn clip_lines(&self, lines: MultiLineString) -> MultiLineString;
    fn clip_line(&self, line: LineString) -> Option<Vec<LineString>>;
    fn rect_line_intersection(&self, line: &Line) -> Option<Coord>;
    fn rect_line_intersections(&self, line: &Line) -> Option<(Coord, Coord)>;
    fn lines(&self) -> Vec<Line>;
}

impl MapRectangle for Rectangle {
    fn shrink(&mut self, v: f64) {
        self.min().x += v;
        self.min().y += v;
        self.max().x -= v;
        self.max().y -= v;
    }

    fn from_bounds(value: Bounds) -> Rectangle {
        Rectangle::new(
            Coord {
                x: value.min.x,
                y: value.min.y,
            },
            Coord {
                x: value.max.x,
                y: value.max.y,
            },
        )
    }

    fn into_line_string(self) -> LineString {
        LineString::new(vec![
            Coord {
                x: self.min().x,
                y: self.max().y,
            },
            self.min(),
            Coord {
                x: self.max().x,
                y: self.min().y,
            },
            self.max(),
            Coord {
                x: self.min().x,
                y: self.max().y,
            },
        ])
    }

    fn touch_margin(&self, other: &Rectangle, margin: f64) -> bool {
        !(self.max().x < other.min().x - margin
            || self.min().x > other.max().x + margin
            || self.max().y < other.min().y - margin
            || self.min().y > other.max().y + margin)
    }

    fn clip_lines(&self, lines: MultiLineString) -> MultiLineString {
        let mut output = MultiLineString::new(vec![]);

        for line in lines.into_iter() {
            if self.contains(&line) {
                output.0.push(line);
            } else if let Some(parts) = self.clip_line(line) {
                output.0.extend(parts)
            }
        }
        output
    }

    fn clip_line(&self, line: LineString) -> Option<Vec<LineString>> {
        if self.intersects(&line) {
            let mut parts = vec![];

            let mut current_line = vec![];

            for segment in line.lines() {
                if self.contains(&segment) {
                    // start and end inside
                    if current_line.is_empty() {
                        current_line.push(segment.start);
                    }
                    current_line.push(segment.end);
                } else if self.contains(&segment.start) {
                    // only start inside
                    if current_line.is_empty() {
                        current_line.push(segment.start);
                    }
                    current_line.push(
                        self.rect_line_intersection(&segment)
                            .expect("Line and rectangle do not intersect"),
                    );
                    parts.push(LineString::new(current_line));
                    current_line = vec![];
                } else if self.contains(&segment.end) {
                    // only end inside
                    current_line.push(
                        self.rect_line_intersection(&segment)
                            .expect("Line and rectangle do not intersect"),
                    );
                    current_line.push(segment.end);
                } else {
                    // neither start or end is inside, but might still intersect twice
                    if let Some((i1, i2)) = self.rect_line_intersections(&segment) {
                        assert!(current_line.is_empty());
                        current_line.push(i1);
                        current_line.push(i2);
                        parts.push(LineString::new(current_line));
                        current_line = vec![];
                    }
                }
            }
            if current_line.len() > 1 {
                parts.push(LineString::new(current_line));
            }
            Some(parts)
        } else {
            None
        }
    }

    fn rect_line_intersection(&self, line: &Line) -> Option<Coord> {
        for segment in self.lines() {
            if let Some(w_intersection) = line_intersection(segment, *line) {
                match w_intersection {
                    LineIntersection::SinglePoint {
                        intersection: c,
                        is_proper: _,
                    } => return Some(c),
                    LineIntersection::Collinear { intersection: _ } => {
                        panic!("Collinear cutting!!");
                    }
                }
            }
        }
        None
    }

    fn rect_line_intersections(&self, line: &Line) -> Option<(Coord, Coord)> {
        let mut is = [None, None];

        for i in 0..is.len() {
            for segment in self.lines() {
                if let Some(w_intersection) = line_intersection(segment, *line) {
                    match w_intersection {
                        LineIntersection::SinglePoint {
                            intersection: c,
                            is_proper: _,
                        } => {
                            is[i] = Some(c);
                            break;
                        }
                        LineIntersection::Collinear { intersection: _ } => {
                            panic!("Collinear cutting!!");
                        }
                    }
                }
            }
        }
        if is[0].is_none() || is[1].is_none() {
            None
        } else if Euclidean::distance(line.start, is[0].unwrap())
            <= Euclidean::distance(line.start, is[1].unwrap())
        {
            Some((is[0].unwrap(), is[1].unwrap()))
        } else {
            Some((is[1].unwrap(), is[0].unwrap()))
        }
    }

    fn lines(&self) -> Vec<Line> {
        vec![
            Line::new(
                Coord {
                    x: self.min().x,
                    y: self.max().y,
                },
                Coord {
                    x: self.min().x,
                    y: self.min().y,
                },
            ),
            Line::new(
                Coord {
                    x: self.min().x,
                    y: self.min().y,
                },
                Coord {
                    x: self.max().x,
                    y: self.min().y,
                },
            ),
            Line::new(
                Coord {
                    x: self.max().x,
                    y: self.min().y,
                },
                Coord {
                    x: self.max().x,
                    y: self.max().y,
                },
            ),
            Line::new(
                Coord {
                    x: self.max().x,
                    y: self.max().y,
                },
                Coord {
                    x: self.min().x,
                    y: self.max().y,
                },
            ),
        ]
    }
}
