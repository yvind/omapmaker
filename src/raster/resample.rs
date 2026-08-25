use crate::raster::{ContinuousRasterMarker, Dfm, DfmGrid, DfmPixelBounds, MaskRasterMarker};

#[allow(dead_code)]
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum MaskRestriction {
    Any,
    Majority,
    All,
}

impl<T: ContinuousRasterMarker + Copy> Dfm<T> {
    /// Area-weight a continuous cell-centred raster onto an aligned coarse grid.
    pub fn restrict_to(&self, coarse_grid: &DfmGrid) -> crate::Result<Self> {
        let ratio = self.grid.aligned_ratio_to(coarse_grid)?;
        let mut coarse = Self::new(transferred_grid(&self.grid, coarse_grid));
        for y in 0..coarse.height() {
            for x in 0..coarse.width() {
                let mut sum = 0.;
                for dy in 0..ratio {
                    for dx in 0..ratio {
                        sum += f64::from(self[(y * ratio + dy, x * ratio + dx)]);
                    }
                }
                coarse[(y, x)] = (sum / (ratio * ratio) as f64) as f32;
            }
        }
        Ok(coarse)
    }

    /// Bilinearly prolong a continuous raster, with linear edge extrapolation
    /// so constants and planes remain exact over the shared cell-edge extent.
    pub fn prolong_to(&self, fine_grid: &DfmGrid) -> crate::Result<Self> {
        fine_grid.aligned_ratio_to(&self.grid)?;
        let mut fine = Self::new(transferred_grid(&self.grid, fine_grid));
        for y in 0..fine.height() {
            let source_y =
                (self.grid.top_left.y - fine.index2coord(y, 0).y) / self.grid.cell_size_m;
            let top = (source_y.floor() as isize).clamp(0, self.height() as isize - 2) as usize;
            let bottom = top + 1;
            let ty = (source_y - top as f64) as f32;
            for x in 0..fine.width() {
                let source_x =
                    (fine.index2coord(0, x).x - self.grid.top_left.x) / self.grid.cell_size_m;
                let left = (source_x.floor() as isize).clamp(0, self.width() as isize - 2) as usize;
                let right = left + 1;
                let tx = (source_x - left as f64) as f32;
                let top_value = self[(top, left)] + tx * (self[(top, right)] - self[(top, left)]);
                let bottom_value =
                    self[(bottom, left)] + tx * (self[(bottom, right)] - self[(bottom, left)]);
                fine[(y, x)] = top_value + ty * (bottom_value - top_value);
            }
        }
        Ok(fine)
    }
}

impl<T: MaskRasterMarker> Dfm<T> {
    /// Restrict a zero/nonzero mask using an explicit categorical policy.
    #[allow(dead_code)]
    pub fn restrict_mask_to(
        &self,
        coarse_grid: &DfmGrid,
        policy: MaskRestriction,
    ) -> crate::Result<Self> {
        let ratio = self.grid.aligned_ratio_to(coarse_grid)?;
        let mut coarse = Self::new(transferred_grid(&self.grid, coarse_grid));
        let cells = ratio * ratio;
        for y in 0..coarse.height() {
            for x in 0..coarse.width() {
                let mut set = 0;
                for dy in 0..ratio {
                    for dx in 0..ratio {
                        set += usize::from(self[(y * ratio + dy, x * ratio + dx)] != 0.);
                    }
                }
                coarse[(y, x)] = if match policy {
                    MaskRestriction::Any => set > 0,
                    MaskRestriction::Majority => set * 2 >= cells,
                    MaskRestriction::All => set == cells,
                } {
                    1.
                } else {
                    0.
                };
            }
        }
        Ok(coarse)
    }
}

fn transferred_grid(source: &DfmGrid, target: &DfmGrid) -> DfmGrid {
    let mut grid = target.clone();
    grid.inner = if source.inner.is_empty() {
        DfmPixelBounds {
            top: 0,
            bottom: 0,
            left: 0,
            right: 0,
        }
    } else {
        let top_left = source.coord(source.inner.top, source.inner.left);
        let bottom_right = source.coord(source.inner.bottom - 1, source.inner.right - 1);
        grid.pixel_bounds(geo::Rect::new(
            geo::coord! {
                x: top_left.x - source.cell_size_m / 2.,
                y: bottom_right.y - source.cell_size_m / 2.
            },
            geo::coord! {
                x: bottom_right.x + source.cell_size_m / 2.,
                y: top_left.y + source.cell_size_m / 2.
            },
        ))
    };
    grid
}
