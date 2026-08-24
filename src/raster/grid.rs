use crate::{STANDARD_CELL_SIZE_METERS, TILE_SIZE_PIXELS};

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct DfmPixelBounds {
    pub top: usize,
    pub bottom: usize,
    pub left: usize,
    pub right: usize,
}

impl DfmPixelBounds {
    pub const fn full(width: usize, height: usize) -> Self {
        Self {
            top: 0,
            bottom: height,
            left: 0,
            right: width,
        }
    }

    pub const fn is_empty(self) -> bool {
        self.top >= self.bottom || self.left >= self.right
    }
}

/// Geometry of a cell-centred scalar raster.
///
/// `top_left` is the centre of cell `(0, 0)`. Raster extent therefore includes
/// half a cell beyond the first and last stored coordinates.
#[derive(Clone, Debug, PartialEq)]
pub struct DfmGrid {
    pub width: usize,
    pub height: usize,
    pub cell_size_m: f64,
    pub top_left: geo::Coord,
    pub inner: DfmPixelBounds,
}

impl DfmGrid {
    pub fn new(
        width: usize,
        height: usize,
        cell_size_m: f64,
        top_left: geo::Coord,
    ) -> crate::Result<Self> {
        anyhow::ensure!(
            width >= 2 && height >= 2,
            "a DFM grid needs at least 2 × 2 cells"
        );
        anyhow::ensure!(
            cell_size_m.is_finite() && cell_size_m > 0.,
            "DFM cell size must be positive and finite"
        );
        Ok(Self {
            width,
            height,
            cell_size_m,
            top_left,
            inner: DfmPixelBounds::full(width, height),
        })
    }

    pub fn standard(top_left: geo::Coord) -> Self {
        Self {
            width: TILE_SIZE_PIXELS,
            height: TILE_SIZE_PIXELS,
            cell_size_m: STANDARD_CELL_SIZE_METERS,
            top_left,
            inner: DfmPixelBounds::full(TILE_SIZE_PIXELS, TILE_SIZE_PIXELS),
        }
    }

    #[inline]
    pub fn coord(&self, row: usize, column: usize) -> geo::Coord {
        geo::Coord {
            x: self.top_left.x + column as f64 * self.cell_size_m,
            y: self.top_left.y - row as f64 * self.cell_size_m,
        }
    }

    #[allow(dead_code)]
    pub fn with_inner(mut self, inner: DfmPixelBounds) -> crate::Result<Self> {
        anyhow::ensure!(
            inner.top <= inner.bottom
                && inner.bottom <= self.height
                && inner.left <= inner.right
                && inner.right <= self.width,
            "inner DFM bounds lie outside the grid"
        );
        self.inner = inner;
        Ok(self)
    }

    pub fn pixel_bounds(&self, bounds: geo::Rect) -> DfmPixelBounds {
        let left = (0..self.width)
            .find(|&x| self.coord(0, x).x >= bounds.min().x)
            .unwrap_or(self.width);
        let right = (left..self.width)
            .find(|&x| self.coord(0, x).x > bounds.max().x)
            .unwrap_or(self.width);
        let top = (0..self.height)
            .find(|&y| self.coord(y, 0).y <= bounds.max().y)
            .unwrap_or(self.height);
        let bottom = (top..self.height)
            .find(|&y| self.coord(y, 0).y < bounds.min().y)
            .unwrap_or(self.height);
        DfmPixelBounds {
            top,
            bottom,
            left,
            right,
        }
    }

    pub fn same_layout(&self, other: &Self) -> bool {
        self.width == other.width
            && self.height == other.height
            && (self.cell_size_m - other.cell_size_m).abs() <= 1e-9
            && (self.top_left.x - other.top_left.x).abs() <= 1e-8
            && (self.top_left.y - other.top_left.y).abs() <= 1e-8
            && self.inner == other.inner
    }

    pub fn ensure_compatible(&self, other: &Self) -> crate::Result<()> {
        anyhow::ensure!(self.same_layout(other), "incompatible DFM grids");
        Ok(())
    }

    pub fn aligned_coarsened(&self, cell_size_m: f64) -> crate::Result<Self> {
        let ratio = cell_size_m / self.cell_size_m;
        let ratio_i = ratio.round() as usize;
        anyhow::ensure!(
            ratio_i > 0 && (ratio - ratio_i as f64).abs() <= 1e-9,
            "DFM resolution ratio must be a positive integer"
        );
        anyhow::ensure!(
            self.width.is_multiple_of(ratio_i) && self.height.is_multiple_of(ratio_i),
            "coarse grid must preserve the cell-centred extent"
        );
        let mut grid = Self::new(
            self.width / ratio_i,
            self.height / ratio_i,
            cell_size_m,
            geo::Coord {
                x: self.top_left.x + (cell_size_m - self.cell_size_m) / 2.,
                y: self.top_left.y - (cell_size_m - self.cell_size_m) / 2.,
            },
        )?;
        if !self.inner.is_empty() {
            let top_left = self.coord(self.inner.top, self.inner.left);
            let bottom_right = self.coord(self.inner.bottom - 1, self.inner.right - 1);
            grid.inner = grid.pixel_bounds(geo::Rect::new(
                geo::coord! {
                    x: top_left.x - self.cell_size_m / 2.,
                    y: bottom_right.y - self.cell_size_m / 2.
                },
                geo::coord! {
                    x: bottom_right.x + self.cell_size_m / 2.,
                    y: top_left.y + self.cell_size_m / 2.
                },
            ));
        } else {
            grid.inner = DfmPixelBounds {
                top: 0,
                bottom: 0,
                left: 0,
                right: 0,
            };
        }
        Ok(grid)
    }

    pub(crate) fn aligned_ratio_to(&self, other: &Self) -> crate::Result<usize> {
        anyhow::ensure!(
            (self.top_left.x - self.cell_size_m / 2. - (other.top_left.x - other.cell_size_m / 2.))
                .abs()
                <= 1e-8
                && (self.top_left.y + self.cell_size_m / 2.
                    - (other.top_left.y + other.cell_size_m / 2.))
                    .abs()
                    <= 1e-8,
            "DFM grids must have aligned origins"
        );
        let ratio = other.cell_size_m / self.cell_size_m;
        let ratio_i = ratio.round() as usize;
        anyhow::ensure!(
            ratio_i > 0 && (ratio - ratio_i as f64).abs() <= 1e-9,
            "DFM resolution ratio must be a positive integer"
        );
        anyhow::ensure!(
            self.width == other.width * ratio_i && self.height == other.height * ratio_i,
            "DFM grids must cover the same cell-centred extent"
        );
        Ok(ratio_i)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn coarsening_preserves_extent_and_coordinates() {
        let fine = DfmGrid::new(8, 6, 0.5, geo::coord! { x: 4., y: 9. }).unwrap();
        let coarse = fine.aligned_coarsened(1.).unwrap();
        assert_eq!((coarse.width, coarse.height), (4, 3));
        assert_eq!(coarse.top_left, geo::coord! { x: 4.25, y: 8.75 });
    }

    #[test]
    fn compatibility_checks_all_layout_metadata() {
        let grid = DfmGrid::new(5, 4, 1., geo::coord! { x: 2., y: 3. }).unwrap();
        let mut shifted = grid.clone();
        shifted.top_left.x += 0.1;
        assert!(grid.ensure_compatible(&shifted).is_err());
        let mut cropped = grid.clone();
        cropped.inner.left = 1;
        assert!(grid.ensure_compatible(&cropped).is_err());
    }

    #[test]
    fn standard_grid_coarsens_without_changing_cell_edge_extent() {
        let fine = DfmGrid::standard(geo::coord! { x: 100.25, y: 255.75 });
        let one_metre = fine.aligned_coarsened(1.).unwrap();
        let two_metre = fine.aligned_coarsened(2.).unwrap();
        assert_eq!(
            (one_metre.width, one_metre.height),
            (TILE_SIZE_PIXELS / 2, TILE_SIZE_PIXELS / 2)
        );
        assert_eq!(
            (two_metre.width, two_metre.height),
            (TILE_SIZE_PIXELS / 4, TILE_SIZE_PIXELS / 4)
        );
        for grid in [&one_metre, &two_metre] {
            assert!(
                (fine.top_left.x
                    - fine.cell_size_m / 2.
                    - (grid.top_left.x - grid.cell_size_m / 2.))
                    .abs()
                    < 1e-9
            );
            assert!(
                (fine.top_left.y + fine.cell_size_m / 2.
                    - (grid.top_left.y + grid.cell_size_m / 2.))
                    .abs()
                    < 1e-9
            );
        }
    }
}
