#[derive(Debug, Clone, Default)]
pub struct Neighborhood {
    pub center: usize,
    pub top_left: Option<usize>,
    pub top: Option<usize>,
    pub top_right: Option<usize>,
    pub right: Option<usize>,
    pub bottom_right: Option<usize>,
    pub bottom: Option<usize>,
    pub bottom_left: Option<usize>,
    pub left: Option<usize>,
}

impl Neighborhood {
    pub fn new(center: usize) -> Neighborhood {
        Neighborhood {
            center,
            ..Default::default()
        }
    }

    pub fn neighbor_indices(
        &self,
    ) -> std::iter::Flatten<std::array::IntoIter<std::option::Option<usize>, 8>> {
        [
            self.top_left,
            self.top,
            self.top_right,
            self.right,
            self.bottom_right,
            self.bottom,
            self.bottom_left,
            self.left,
        ]
        .into_iter()
        .flatten()
    }

    pub fn all_indices(&self) -> Vec<usize> {
        let mut vec = [
            self.top_left,
            self.top,
            self.top_right,
            self.right,
            self.bottom_right,
            self.bottom,
            self.bottom_left,
            self.left,
        ]
        .into_iter()
        .flatten()
        .collect::<Vec<_>>();
        vec.push(self.center);
        vec
    }

    pub fn register_neighbor(&mut self, index: usize, side: NeighborSide) {
        match side {
            NeighborSide::TopLeft => self.top_left = Some(index),
            NeighborSide::Top => self.top = Some(index),
            NeighborSide::TopRight => self.top_right = Some(index),
            NeighborSide::Right => self.right = Some(index),
            NeighborSide::BottomRight => self.bottom_right = Some(index),
            NeighborSide::Bottom => self.bottom = Some(index),
            NeighborSide::BottomLeft => self.bottom_left = Some(index),
            NeighborSide::Left => self.left = Some(index),
            _ => (),
        }
    }

    pub fn has_neighbor_above(&self) -> bool {
        self.top_left.is_some() || self.top.is_some() || self.top_right.is_some()
    }

    pub fn has_neighbor_below(&self) -> bool {
        self.bottom_left.is_some() || self.bottom.is_some() || self.bottom_right.is_some()
    }

    pub fn has_neighbor_right(&self) -> bool {
        self.bottom_right.is_some() || self.right.is_some() || self.top_right.is_some()
    }

    pub fn has_neighbor_left(&self) -> bool {
        self.top_left.is_some() || self.left.is_some() || self.bottom_left.is_some()
    }
}

impl TryFrom<[Option<usize>; 9]> for Neighborhood {
    type Error = crate::Error;

    fn try_from(value: [Option<usize>; 9]) -> Result<Self, Self::Error> {
        if let Some(center) = value[0] {
            Ok(Neighborhood {
                center,
                top_left: value[1],
                top: value[2],
                top_right: value[3],
                right: value[4],
                bottom_right: value[5],
                bottom: value[6],
                bottom_left: value[7],
                left: value[8],
            })
        } else {
            Err(Self::Error::NeighborhoodError)
        }
    }
}

#[derive(Debug, Clone, Copy)]
pub enum NeighborSide {
    TopLeft,
    Top,
    TopRight,
    Right,
    BottomRight,
    Bottom,
    BottomLeft,
    Left,
    None,
}

impl NeighborSide {
    pub fn get_side(bounds: &geo::Rect, tile_center: [f64; 2]) -> NeighborSide {
        if tile_center[0] < bounds.min().x && tile_center[1] > bounds.max().y {
            return NeighborSide::TopLeft;
        }
        if tile_center[0] > bounds.max().x && tile_center[1] > bounds.max().y {
            return NeighborSide::TopRight;
        }
        if tile_center[0] > bounds.max().x && tile_center[1] < bounds.min().y {
            return NeighborSide::BottomRight;
        }
        if tile_center[0] < bounds.min().x && tile_center[1] < bounds.min().y {
            return NeighborSide::BottomLeft;
        }
        if tile_center[1] > bounds.max().y {
            return NeighborSide::Top;
        }
        if tile_center[0] > bounds.max().x {
            return NeighborSide::Right;
        }
        if tile_center[1] < bounds.min().y {
            return NeighborSide::Bottom;
        }
        if tile_center[0] < bounds.min().x {
            return NeighborSide::Left;
        }
        NeighborSide::None
    }

    pub fn is_edge_tile(index: usize, nx: usize, ny: usize) -> NeighborSide {
        let right = nx - 1;
        let bottom = ny - 1;
        match (index % nx, index / nx) {
            (0, 0) => NeighborSide::TopLeft,
            (x, 0) if x == right => NeighborSide::TopRight,
            (x, y) if x == right && y == bottom => NeighborSide::BottomRight,
            (0, y) if y == bottom => NeighborSide::BottomLeft,
            (0, _) => NeighborSide::Left,
            (x, _) if x == right => NeighborSide::Right,
            (_, 0) => NeighborSide::Top,
            (_, y) if y == bottom => NeighborSide::Bottom,
            _ => NeighborSide::None,
        }
    }
}

pub fn neighbors_on_grid(nx: usize, ny: usize) -> Vec<Neighborhood> {
    let mut neighbours = Vec::with_capacity(nx * ny);

    for yi in 0..ny {
        for xi in 0..nx {
            if xi == 0 && yi == 0 {
                //no neighbors to the left or top
                neighbours.push(
                    Neighborhood::try_from([
                        Some(yi * nx + xi),
                        None,
                        None,
                        None,
                        Some(yi * nx + xi + 1),
                        Some(yi * nx + xi + 1 + nx),
                        Some(yi * nx + xi + nx),
                        None,
                        None,
                    ])
                    .unwrap(),
                );
            } else if xi == nx - 1 && yi == 0 {
                // no neighbors to the right or top
                neighbours.push(
                    Neighborhood::try_from([
                        Some(yi * nx + xi),
                        None,
                        None,
                        None,
                        None,
                        None,
                        Some(yi * nx + xi + nx),
                        Some(yi * nx + xi + nx - 1),
                        Some(yi * nx + xi - 1),
                    ])
                    .unwrap(),
                );
            } else if xi == 0 && yi == ny - 1 {
                // no neighbors to the left or bottom
                neighbours.push(
                    Neighborhood::try_from([
                        Some(yi * nx + xi),
                        None,
                        Some(yi * nx + xi - nx),
                        Some(yi * nx + xi - nx + 1),
                        Some(yi * nx + xi + 1),
                        None,
                        None,
                        None,
                        None,
                    ])
                    .unwrap(),
                );
            } else if xi == nx - 1 && yi == ny - 1 {
                // no neighbors to the right or bottom
                neighbours.push(
                    Neighborhood::try_from([
                        Some(yi * nx + xi),
                        Some(yi * nx + xi - 1 - nx),
                        Some(yi * nx + xi - nx),
                        None,
                        None,
                        None,
                        None,
                        None,
                        Some(yi * nx + xi - 1),
                    ])
                    .unwrap(),
                );
            } else if xi == 0 {
                // no neighbors to the left
                neighbours.push(
                    Neighborhood::try_from([
                        Some(yi * nx + xi),
                        None,
                        Some(yi * nx + xi - nx),
                        Some(yi * nx + xi - nx + 1),
                        Some(yi * nx + xi + 1),
                        Some(yi * nx + xi + nx + 1),
                        Some(yi * nx + xi + nx),
                        None,
                        None,
                    ])
                    .unwrap(),
                );
            } else if xi == nx - 1 {
                neighbours.push(
                    Neighborhood::try_from([
                        Some(yi * nx + xi),
                        Some(yi * nx + xi - 1 - nx),
                        Some(yi * nx + xi - nx),
                        None,
                        None,
                        None,
                        Some(yi * nx + xi + nx),
                        Some(yi * nx + xi + nx - 1),
                        Some(yi * nx + xi - 1),
                    ])
                    .unwrap(),
                );
            } else if yi == 0 {
                neighbours.push(
                    Neighborhood::try_from([
                        Some(yi * nx + xi),
                        None,
                        None,
                        None,
                        Some(yi * nx + xi + 1),
                        Some(yi * nx + xi + nx + 1),
                        Some(yi * nx + xi + nx),
                        Some(yi * nx + xi + nx - 1),
                        Some(yi * nx + xi - 1),
                    ])
                    .unwrap(),
                );
            } else if yi == ny - 1 {
                neighbours.push(
                    Neighborhood::try_from([
                        Some(yi * nx + xi),
                        Some(yi * nx + xi - 1 - nx),
                        Some(yi * nx + xi - nx),
                        Some(yi * nx + xi - nx + 1),
                        Some(yi * nx + xi + 1),
                        None,
                        None,
                        None,
                        Some(yi * nx + xi - 1),
                    ])
                    .unwrap(),
                );
            } else {
                neighbours.push(
                    Neighborhood::try_from([
                        Some(yi * nx + xi),
                        Some(yi * nx + xi - 1 - nx),
                        Some(yi * nx + xi - nx),
                        Some(yi * nx + xi - nx + 1),
                        Some(yi * nx + xi + 1),
                        Some(yi * nx + xi + nx + 1),
                        Some(yi * nx + xi + nx),
                        Some(yi * nx + xi + nx - 1),
                        Some(yi * nx + xi - 1),
                    ])
                    .unwrap(),
                );
            }
        }
    }
    neighbours
}
