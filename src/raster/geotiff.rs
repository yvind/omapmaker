use std::{
    ffi::OsString,
    path::{Path, PathBuf},
};

use anyhow::Context;
use geotiff_writer::GeoTiffBuilder;
use ndarray::Array2;
use proj_core::CrsDef;

use crate::{
    CELL_SIZE_METERS,
    raster::{Dfm, RasterMarker},
};

const NODATA_VALUE: f64 = -9999.;
const RENDERED_NODATA_VALUE: u8 = 0;
const RENDERED_NODATA_TEXT: &str = "0";

pub fn write_merged_dfm_geotiff<T: RasterMarker>(
    save_location: &Path,
    suffix: &str,
    tiles: &[Dfm<T>],
    ref_point: geo::Coord,
    crs: Option<&CrsDef>,
) -> crate::Result<PathBuf> {
    let path = raster_output_path(save_location, suffix);
    let Some((merged, top_left)) = merge_dfms(tiles) else {
        return Ok(path);
    };

    let rendered = render_raster_for_image_viewers(&merged);

    let (height, width) = rendered.dim();
    let width = u32::try_from(width).context("Merged raster width does not fit in u32")?;
    let height = u32::try_from(height).context("Merged raster height does not fit in u32")?;

    let (origin_x, origin_y) = geotiff_origin(top_left, ref_point);
    let mut builder = GeoTiffBuilder::new(width, height)
        .pixel_scale(CELL_SIZE_METERS, CELL_SIZE_METERS)
        .origin(origin_x, origin_y)
        .nodata(RENDERED_NODATA_TEXT);

    if let Some(epsg) = crs
        .map(CrsDef::epsg)
        .filter(|epsg| *epsg != 0)
        .and_then(|epsg| u16::try_from(epsg).ok())
    {
        builder = builder.epsg(epsg);
    }

    builder.write_2d(&path, rendered.view()).with_context(|| {
        format!(
            "Failed to write merged {suffix} raster to {}",
            path.display()
        )
    })?;

    Ok(path)
}

/// Write a metric/diagnostic raster without display normalization.
///
/// Unlike [`write_merged_dfm_geotiff`], the resulting band is IEEE `f32` and
/// therefore preserves metres, signed residuals, probabilities, and NoData.
pub fn write_merged_dfm_geotiff_f32<T: RasterMarker>(
    save_location: &Path,
    suffix: &str,
    tiles: &[Dfm<T>],
    ref_point: geo::Coord,
    crs: Option<&CrsDef>,
) -> crate::Result<PathBuf> {
    let path = raster_output_path(save_location, suffix);
    let Some((merged, top_left)) = merge_dfms(tiles) else {
        return Ok(path);
    };
    let metric = merged.mapv(|value| value as f32);

    let (height, width) = metric.dim();
    let width = u32::try_from(width).context("Merged raster width does not fit in u32")?;
    let height = u32::try_from(height).context("Merged raster height does not fit in u32")?;
    let (origin_x, origin_y) = geotiff_origin(top_left, ref_point);
    let mut builder = GeoTiffBuilder::new(width, height)
        .pixel_scale(CELL_SIZE_METERS, CELL_SIZE_METERS)
        .origin(origin_x, origin_y)
        .nodata("-9999");

    if let Some(epsg) = crs
        .map(CrsDef::epsg)
        .filter(|epsg| *epsg != 0)
        .and_then(|epsg| u16::try_from(epsg).ok())
    {
        builder = builder.epsg(epsg);
    }

    builder.write_2d(&path, metric.view()).with_context(|| {
        format!(
            "Failed to write merged {suffix} metric raster to {}",
            path.display()
        )
    })?;

    Ok(path)
}

fn raster_output_path(save_location: &Path, suffix: &str) -> PathBuf {
    let stem = save_location
        .file_stem()
        .map(|stem| stem.to_os_string())
        .unwrap_or_else(|| OsString::from("omap"));
    let mut file_name = stem;
    file_name.push(format!("_{suffix}.tif"));

    save_location.with_file_name(file_name)
}

fn merge_dfms<T: RasterMarker>(tiles: &[Dfm<T>]) -> Option<(Array2<f64>, geo::Coord)> {
    let mut inner_tiles = tiles.iter().filter(|tile| !tile.grid.inner.is_empty());
    let first = inner_tiles.next()?;
    let first_top_left = first.index2coord(first.grid.inner.top, first.grid.inner.left);
    let first_bottom_right =
        first.index2coord(first.grid.inner.bottom - 1, first.grid.inner.right - 1);

    let mut min_x = first_top_left.x;
    let mut max_x = first_bottom_right.x;
    let mut max_y = first_top_left.y;
    let mut min_y = first_bottom_right.y;

    for tile in inner_tiles {
        let top_left = tile.index2coord(tile.grid.inner.top, tile.grid.inner.left);
        let bottom_right = tile.index2coord(tile.grid.inner.bottom - 1, tile.grid.inner.right - 1);
        min_x = min_x.min(top_left.x);
        max_x = max_x.max(bottom_right.x);
        max_y = max_y.max(top_left.y);
        min_y = min_y.min(bottom_right.y);
    }

    let width = ((max_x - min_x) / CELL_SIZE_METERS).round() as usize + 1;
    let height = ((max_y - min_y) / CELL_SIZE_METERS).round() as usize + 1;

    let mut sums = Array2::zeros((height, width));
    let mut counts = vec![0_u16; width * height];

    for tile in tiles {
        if tile.grid.inner.is_empty() {
            continue;
        }
        let inner_top_left = tile.index2coord(tile.grid.inner.top, tile.grid.inner.left);
        let x_offset = ((inner_top_left.x - min_x) / CELL_SIZE_METERS).round() as usize;
        let y_offset = ((max_y - inner_top_left.y) / CELL_SIZE_METERS).round() as usize;

        for y in tile.grid.inner.top..tile.grid.inner.bottom {
            let target_y = y_offset + y - tile.grid.inner.top;
            for x in tile.grid.inner.left..tile.grid.inner.right {
                let value = tile[(y, x)];
                if value == f32::MIN || !value.is_finite() {
                    continue;
                }

                let target_x = x_offset + x - tile.grid.inner.left;
                sums[[target_y, target_x]] += value as f64;
                counts[target_y * width + target_x] =
                    counts[target_y * width + target_x].saturating_add(1);
            }
        }
    }

    for y in 0..height {
        for x in 0..width {
            let count = counts[y * width + x];
            if count == 0 {
                sums[[y, x]] = NODATA_VALUE;
            } else {
                sums[[y, x]] /= f64::from(count);
            }
        }
    }

    Some((sums, geo::Coord { x: min_x, y: max_y }))
}

fn render_raster_for_image_viewers(raster: &Array2<f64>) -> Array2<u8> {
    let mut min = f64::INFINITY;
    let mut max = f64::NEG_INFINITY;

    for value in raster.iter().copied().filter(|value| is_renderable(*value)) {
        min = min.min(value);
        max = max.max(value);
    }

    if !min.is_finite() || !max.is_finite() {
        return Array2::from_elem(raster.dim(), RENDERED_NODATA_VALUE);
    }

    if min == max {
        return raster.mapv(|value| {
            if is_renderable(value) {
                u8::MAX
            } else {
                RENDERED_NODATA_VALUE
            }
        });
    }

    let scale = f64::from(u8::MAX - 1) / (max - min);
    raster.mapv(|value| {
        if !is_renderable(value) {
            return RENDERED_NODATA_VALUE;
        }

        ((value - min) * scale).round() as u8 + 1
    })
}

fn is_renderable(value: f64) -> bool {
    value.is_finite() && value != NODATA_VALUE
}

fn geotiff_origin(top_left: geo::Coord, ref_point: geo::Coord) -> (f64, f64) {
    (
        top_left.x + ref_point.x - CELL_SIZE_METERS / 2.,
        top_left.y + ref_point.y + CELL_SIZE_METERS / 2.,
    )
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::raster::{DfmGrid, GroundRelief2m};
    use std::time::{SystemTime, UNIX_EPOCH};

    fn sample_tags(bytes: &[u8]) -> (Option<u16>, Option<u16>) {
        assert_eq!(&bytes[..4], &[b'I', b'I', 42, 0]);
        let u16_at = |offset: usize| u16::from_le_bytes([bytes[offset], bytes[offset + 1]]);
        let u32_at = |offset: usize| {
            u32::from_le_bytes([
                bytes[offset],
                bytes[offset + 1],
                bytes[offset + 2],
                bytes[offset + 3],
            ])
        };
        let ifd_offset = u32_at(4) as usize;
        let entry_count = u16_at(ifd_offset) as usize;
        let entries = (0..entry_count).map(|index| ifd_offset + 2 + index * 12);
        let mut bits_per_sample = None;
        let mut sample_format = None;
        for entry in entries {
            match u16_at(entry) {
                258 => bits_per_sample = Some(u16_at(entry + 8)),
                339 => sample_format = Some(u16_at(entry + 8)),
                _ => {}
            }
        }
        (bits_per_sample, sample_format)
    }

    fn temporary_save_location(test_name: &str) -> (PathBuf, PathBuf) {
        let unique = SystemTime::now()
            .duration_since(UNIX_EPOCH)
            .unwrap()
            .as_nanos();
        let directory = std::env::temp_dir().join(format!(
            "omapmaker-{test_name}-{}-{unique}",
            std::process::id()
        ));
        std::fs::create_dir(&directory).unwrap();
        let save_location = directory.join("map.omap");
        (directory, save_location)
    }

    #[test]
    fn metric_writer_emits_a_float32_band() {
        let grid = DfmGrid::new(2, 2, 0.5, geo::coord! { x: 0., y: 0.5 }).unwrap();
        let mut raster = Dfm::<GroundRelief2m>::new(grid);
        raster.field.copy_from_slice(&[-1., 0.25, 1.5, 2.]);

        let (directory, save_location) = temporary_save_location("raw-geotiff-test");
        let path = write_merged_dfm_geotiff_f32(
            &save_location,
            "relief",
            &[raster],
            geo::coord! { x: 0., y: 0. },
            None,
        )
        .unwrap();

        let bytes = std::fs::read(&path).unwrap();
        assert_eq!(sample_tags(&bytes), (Some(32), Some(3))); // IEEE float

        std::fs::remove_file(path).unwrap();
        std::fs::remove_dir(directory).unwrap();
    }

    #[test]
    fn viewer_writer_emits_an_unsigned_8_bit_band() {
        let grid = DfmGrid::new(2, 2, 0.5, geo::coord! { x: 0., y: 0.5 }).unwrap();
        let mut raster = Dfm::<GroundRelief2m>::new(grid);
        raster.field.copy_from_slice(&[-1., 0.25, 1.5, 2.]);
        let (directory, save_location) = temporary_save_location("viewer-geotiff-test");
        let path = write_merged_dfm_geotiff(
            &save_location,
            "relief",
            &[raster],
            geo::coord! { x: 0., y: 0. },
            None,
        )
        .unwrap();

        let bytes = std::fs::read(&path).unwrap();
        assert_eq!(sample_tags(&bytes), (Some(8), Some(1))); // unsigned integer

        std::fs::remove_file(path).unwrap();
        std::fs::remove_dir(directory).unwrap();
    }
}
