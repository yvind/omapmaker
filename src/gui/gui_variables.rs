use std::collections::HashMap;

use geo::{Area, BooleanOps, Intersects, Validation};
use proj_core::{CrsDef, Transform};
use walkers::Position;

use super::terminal_like::TerminalLike;
use crate::{
    drawable::{DrawOrder, DrawableOmap},
    map_gen::egui_map::{AreaSymbol, Symbol},
    parameters::{FileParameters, MapParameters},
    statistics::LidarStats,
};

#[derive(Debug, thiserror::Error)]
pub enum StageValidationError {
    #[error("Choose at least one lidar file before continuing")]
    MissingLidarFiles,
    #[error("Choose an output .omap save location before continuing")]
    MissingSaveLocation,
    #[error("Select a test square before continuing")]
    MissingSelectedSquare,
    #[error("The selected test square is no longer available")]
    InvalidSelectedSquare,
    #[error("Lidar statistics are not ready yet")]
    MissingLidarStats,
    #[error("Finish or clear the polygon filter before continuing")]
    UnfinishedPolygonFilter,
    #[error("The polygon filter is not valid")]
    InvalidPolygonFilter,
}

#[derive(Debug, Default, Clone, Copy, PartialEq)]
pub enum TileProvider {
    #[default]
    OpenStreetMap,
    OpenTopoMap,
    ArcGIS,
}

#[derive(Clone)]
pub struct ProjectFiles {
    pub paths: Vec<std::path::PathBuf>,
    pub save_location: std::path::PathBuf,
    pub selected_file: Option<usize>,
    pub crs_epsg: Vec<Option<CrsDef>>,
    pub write_single_copc: bool,
    pub single_copc_path: Option<std::path::PathBuf>,
    pub worker_threads: usize,
    pub save_rasters: bool,
    pub save_slope_raster: bool,
    pub save_hillshade_raster: bool,
}

impl Default for ProjectFiles {
    fn default() -> Self {
        Self {
            paths: Default::default(),
            save_location: Default::default(),
            selected_file: Default::default(),
            crs_epsg: Default::default(),
            write_single_copc: Default::default(),
            single_copc_path: Default::default(),
            worker_threads: std::thread::available_parallelism()
                .map(|threads| threads.get())
                .unwrap_or(8)
                .max(1),
            save_rasters: Default::default(),
            save_slope_raster: Default::default(),
            save_hillshade_raster: Default::default(),
        }
    }
}

impl ProjectFiles {
    pub fn validate_welcome(&self) -> Result<ReadyForCrsCheck, StageValidationError> {
        if self.paths.is_empty() {
            return Err(StageValidationError::MissingLidarFiles);
        }
        if self.save_location.as_os_str().is_empty() {
            return Err(StageValidationError::MissingSaveLocation);
        }

        Ok(ReadyForCrsCheck {
            paths: self.paths.clone(),
        })
    }

    pub fn to_file_parameters(&self) -> FileParameters {
        if let Some(single_copc_path) = &self.single_copc_path {
            return FileParameters {
                paths: vec![single_copc_path.clone()],
                save_location: self.save_location.clone(),
                save_slope_raster: self.save_rasters && self.save_slope_raster,
                save_hillshade_raster: self.save_rasters && self.save_hillshade_raster,
                crs_epsg: vec![],
            };
        }

        FileParameters {
            paths: self.paths.clone(),
            save_location: self.save_location.clone(),
            save_slope_raster: self.save_rasters && self.save_slope_raster,
            save_hillshade_raster: self.save_rasters && self.save_hillshade_raster,
            crs_epsg: self.crs_epsg.clone(),
        }
    }
}

pub struct ReadyForCrsCheck {
    pub paths: Vec<std::path::PathBuf>,
}

#[derive(Default)]
pub struct LidarAnalysisState {
    pub boundaries: Vec<[Position; 4]>,
    pub boundary_areas: Vec<f64>,
    pub crs_less_search_strings: Vec<String>,
    pub unique_crs: Vec<CrsDef>,
    pub output_crs_string: String,
    pub connected_components: Vec<Vec<usize>>,
    pub drop_checkboxes: Vec<bool>,
    pub stats: Option<LidarStats>,
}

pub struct AreaSelectionState {
    pub polygon_filter: geo::LineString,
    pub drawing_polygon: bool,
}

impl Default for AreaSelectionState {
    fn default() -> Self {
        Self {
            polygon_filter: geo::LineString::new(vec![]),
            drawing_polygon: false,
        }
    }
}

pub struct TileSelectionState {
    pub test_area_projected: geo::MultiPolygon,
    pub test_area_display: geo::MultiPolygon,
    pub selected_square: Option<geo::Rect>,
    pub selected_square_boundary: Option<[walkers::Position; 4]>,
}

impl Default for TileSelectionState {
    fn default() -> Self {
        Self {
            test_area_projected: geo::MultiPolygon(vec![]),
            test_area_display: geo::MultiPolygon(vec![]),
            selected_square: None,
            selected_square_boundary: None,
        }
    }
}

impl TileSelectionState {
    pub fn validate_selected_square(&self) -> Result<geo::Rect, StageValidationError> {
        let selected_square = self
            .selected_square
            .ok_or(StageValidationError::MissingSelectedSquare)?;

        let overlap = self
            .test_area_projected
            .intersection(&selected_square.to_polygon());
        if overlap.unsigned_area() < selected_square.unsigned_area() * 0.5 {
            return Err(StageValidationError::InvalidSelectedSquare);
        }

        Ok(selected_square)
    }
}

pub struct MapPreviewState {
    pub visibility_checkboxes: HashMap<Symbol, bool>,
    pub generating_map_tile: bool,
    pub map_tile: Option<DrawableOmap>,
    pub map_opacity: f32,
    pub contour_score: (f32, f32),
}

impl Default for MapPreviewState {
    fn default() -> Self {
        let mut visibility_checkboxes = HashMap::new();
        for symbol in Symbol::draw_order() {
            visibility_checkboxes.insert(symbol, true);
        }
        visibility_checkboxes.insert(Symbol::Area(AreaSymbol::WhiteForest), true);

        Self {
            visibility_checkboxes,
            generating_map_tile: false,
            map_tile: None,
            map_opacity: 1.0,
            contour_score: (0.0, 0.0),
        }
    }
}

#[derive(Default)]
pub struct MapViewState {
    pub tile_provider: TileProvider,
}

#[derive(Default)]
pub struct GenerationState {
    pub params: MapParameters,
}

pub struct ReadyForCopcConversion {
    pub file_params: FileParameters,
    pub output_crs: Option<CrsDef>,
    pub save_location: std::path::PathBuf,
    pub boundaries: Vec<[walkers::Position; 4]>,
    pub polygon_filter: geo::LineString,
    pub write_single_copc: bool,
}

pub struct ReadyForMapPreview {
    pub paths: Vec<std::path::PathBuf>,
    pub test_area: geo::Rect,
    pub stats: LidarStats,
}

pub struct ReadyForFinalMap {
    pub map_params: MapParameters,
    pub file_params: FileParameters,
    pub polygon_filter: geo::LineString,
    pub stats: LidarStats,
}

#[derive(Default)]
pub struct GuiVariables {
    pub project: ProjectFiles,
    pub lidar: LidarAnalysisState,
    pub area: AreaSelectionState,
    pub tile: TileSelectionState,
    pub preview: MapPreviewState,
    pub map_view: MapViewState,
    pub generation: GenerationState,
    pub log_terminal: TerminalLike,
}

impl GuiVariables {
    pub fn get_most_popular_crs(&self) -> Option<CrsDef> {
        let mut crs_tally: Vec<(u32, u16, CrsDef)> = Vec::new();
        for crs in self.project.crs_epsg.iter().flatten() {
            let epsg = crs.epsg();
            if let Some((_, count, _)) = crs_tally.iter_mut().find(|(code, _, _)| *code == epsg) {
                *count += 1;
            } else {
                crs_tally.push((epsg, 1, crs.clone()));
            }
        }
        crs_tally
            .into_iter()
            .max_by(|(_, v1, _), (_, v2, _)| v1.cmp(v2))
            .map(|(_, _, crs)| crs)
    }

    pub fn update_unique_crs(&mut self) {
        self.lidar.unique_crs.clear();
        for crs in self.project.crs_epsg.iter() {
            if let Some(def) = crs
                && !self
                    .lidar
                    .unique_crs
                    .iter()
                    .any(|existing| existing.epsg() == def.epsg())
            {
                self.lidar.unique_crs.push(def.clone());
            }
        }
    }

    pub fn drop_small_graph_components(&mut self) -> Position {
        let mut drop_files = vec![];

        let mut biggest_component_index = 0;
        let mut biggest_component_size = 0;
        for (i, v) in self.lidar.connected_components.iter().enumerate() {
            if v.len() > biggest_component_size {
                biggest_component_size = v.len();
                biggest_component_index = i;
            }
        }

        for (i, v) in self.lidar.connected_components.iter().enumerate() {
            if i == biggest_component_index {
                continue;
            }
            for fi in v.iter() {
                drop_files.push(*fi);
            }
        }

        drop_files.sort_by(|a, b| b.cmp(a));

        for drop_file in drop_files {
            self.project.paths.remove(drop_file);
            self.project.crs_epsg.remove(drop_file);
            self.lidar.boundaries.remove(drop_file);
            self.lidar.boundary_areas.remove(drop_file);
        }

        let mut new_home = (0., 0.);
        for bound in self.lidar.boundaries.iter() {
            new_home.0 += (bound[0].x() + bound[2].x()) / 2.;
            new_home.1 += (bound[0].y() + bound[2].y()) / 2.;
        }
        new_home.0 /= self.lidar.boundaries.len() as f64;
        new_home.1 /= self.lidar.boundaries.len() as f64;

        walkers::lon_lat(new_home.0, new_home.1)
    }

    pub fn update_map(&mut self, other: DrawableOmap) {
        if let Some(map) = &mut self.preview.map_tile {
            map.update(other);
        } else {
            self.preview.map_tile = Some(other);
        }
    }

    pub fn validate_copc_conversion(&self) -> Result<ReadyForCopcConversion, StageValidationError> {
        if self.area.drawing_polygon {
            return Err(StageValidationError::UnfinishedPolygonFilter);
        }

        if !self.area.polygon_filter.0.is_empty() {
            if !self.area.polygon_filter.is_closed() {
                return Err(StageValidationError::UnfinishedPolygonFilter);
            }

            let polygon = geo::Polygon::new(self.area.polygon_filter.clone(), vec![]);
            if !polygon.is_valid() {
                return Err(StageValidationError::InvalidPolygonFilter);
            }
        }

        Ok(ReadyForCopcConversion {
            file_params: FileParameters {
                paths: self.project.paths.clone(),
                save_location: self.project.save_location.clone(),
                save_slope_raster: false,
                save_hillshade_raster: false,
                crs_epsg: self.project.crs_epsg.clone(),
            },
            output_crs: self.generation.params.output.crs.clone(),
            save_location: self.project.save_location.clone(),
            boundaries: self.lidar.boundaries.clone(),
            polygon_filter: self.area.polygon_filter.clone(),
            write_single_copc: self.project.write_single_copc,
        })
    }

    pub fn prepare_test_area(&mut self) -> crate::Result<()> {
        let polygon_filter = crate::project::polygon::from_walkers_map_coords(
            self.generation.params.output.crs.clone(),
            self.area.polygon_filter.clone(),
        )?;

        let mut test_area = geo::MultiPolygon(vec![]);
        for boundary in &self.lidar.boundaries {
            let boundary_polygon = boundary_to_projected_polygon(
                self.generation.params.output.crs.as_ref(),
                boundary,
            )?;

            let clipped = if let Some(polygon_filter) = &polygon_filter {
                if !boundary_polygon.intersects(polygon_filter) {
                    continue;
                }
                boundary_polygon.intersection(polygon_filter)
            } else {
                geo::MultiPolygon(vec![boundary_polygon])
            };

            test_area = if test_area.0.is_empty() {
                clipped
            } else {
                test_area.union(&clipped)
            };
        }

        if test_area.0.is_empty() {
            anyhow::bail!("The chosen polygon filter does not intersect the lidar files");
        }

        self.tile.test_area_display = projected_to_display_multipolygon(
            self.generation.params.output.crs.as_ref(),
            &test_area,
        )?;
        self.tile.test_area_projected = test_area;
        self.tile.selected_square = None;
        self.tile.selected_square_boundary = None;

        Ok(())
    }

    pub fn polygon_area(&self) -> Option<f64> {
        if self.area.polygon_filter.0.len() < 3 {
            return None;
        }

        let mut line = self.area.polygon_filter.clone();
        if !line.is_closed() {
            line.close();
        }

        crate::project::polygon::from_walkers_map_coords(
            self.generation.params.output.crs.clone(),
            line,
        )
        .ok()
        .flatten()
        .map(|polygon| polygon.unsigned_area())
    }

    pub fn validate_map_preview(&self) -> Result<ReadyForMapPreview, StageValidationError> {
        let test_area = self.tile.validate_selected_square()?;
        let stats = self
            .lidar
            .stats
            .clone()
            .ok_or(StageValidationError::MissingLidarStats)?;

        Ok(ReadyForMapPreview {
            paths: self.project.to_file_parameters().paths,
            test_area,
            stats,
        })
    }

    pub fn validate_final_map(&self) -> Result<ReadyForFinalMap, StageValidationError> {
        let stats = self
            .lidar
            .stats
            .clone()
            .ok_or(StageValidationError::MissingLidarStats)?;

        Ok(ReadyForFinalMap {
            map_params: self.generation.params.clone(),
            file_params: self.project.to_file_parameters(),
            polygon_filter: self.area.polygon_filter.clone(),
            stats,
        })
    }
}

fn boundary_to_projected_polygon(
    crs: Option<&CrsDef>,
    boundary: &[walkers::Position; 4],
) -> crate::Result<geo::Polygon> {
    let line = geo::LineString::new(vec![
        boundary[0].0,
        boundary[1].0,
        boundary[2].0,
        boundary[3].0,
        boundary[0].0,
    ]);

    let Some(crs) = crs else {
        return Ok(geo::Polygon::new(line, vec![]));
    };

    let transform = Transform::from_epsg(4326, crs.epsg())?;
    Ok(geo::Polygon::new(transform.convert_geometry(line)?, vec![]))
}

fn projected_to_display_multipolygon(
    crs: Option<&CrsDef>,
    multipolygon: &geo::MultiPolygon,
) -> crate::Result<geo::MultiPolygon> {
    let Some(crs) = crs else {
        return Ok(multipolygon.clone());
    };

    let transform = Transform::from_epsg(crs.epsg(), 4326)?;
    let mut out = Vec::with_capacity(multipolygon.0.len());
    for polygon in &multipolygon.0 {
        let exterior = transform.convert_geometry(polygon.exterior().clone())?;
        let interiors = polygon
            .interiors()
            .iter()
            .map(|line| transform.convert_geometry(line.clone()))
            .collect::<Result<Vec<_>, _>>()?;
        out.push(geo::Polygon::new(exterior, interiors));
    }

    Ok(geo::MultiPolygon(out))
}
