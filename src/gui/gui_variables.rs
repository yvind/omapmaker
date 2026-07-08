use std::collections::HashMap;

use geo::{Area, LineString, Validation};
use proj_core::CrsDef;
use walkers::Position;

use super::terminal_like::TerminalLike;
use crate::{
    drawable::{DrawOrder, DrawableOmap},
    map_gen::egui_map::{AreaSymbol, Symbol},
    neighbors::Neighborhood,
    parameters::{FileParameters, MapParameters},
    statistics::LidarStats,
};

#[derive(Debug, thiserror::Error)]
pub enum StageValidationError {
    #[error("Choose at least one lidar file before continuing")]
    MissingLidarFiles,
    #[error("Choose an output .omap save location before continuing")]
    MissingSaveLocation,
    #[error("Select a lidar file before continuing")]
    MissingSelectedFile,
    #[error("The selected lidar file is no longer available")]
    InvalidSelectedFile,
    #[error("Select a sub-tile before continuing")]
    MissingSelectedTile,
    #[error("The selected sub-tile is no longer available")]
    InvalidSelectedTile,
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

    pub fn validate_selected_file(&self) -> Result<ReadyForTileSelection, StageValidationError> {
        let selected_file = self
            .selected_file
            .ok_or(StageValidationError::MissingSelectedFile)?;
        if selected_file >= self.paths.len() || selected_file >= self.crs_epsg.len() {
            return Err(StageValidationError::InvalidSelectedFile);
        }

        Ok(ReadyForTileSelection {
            path: self.paths[selected_file].clone(),
            crs: self.crs_epsg[selected_file].clone(),
        })
    }

    pub fn to_file_parameters(&self) -> FileParameters {
        if let Some(single_copc_path) = &self.single_copc_path {
            return FileParameters {
                paths: vec![single_copc_path.clone()],
                save_location: self.save_location.clone(),
                crs_epsg: vec![],
            };
        }

        FileParameters {
            paths: self.paths.clone(),
            save_location: self.save_location.clone(),
            crs_epsg: self.crs_epsg.clone(),
        }
    }
}

pub struct ReadyForCrsCheck {
    pub paths: Vec<std::path::PathBuf>,
}

pub struct ReadyForTileSelection {
    pub path: std::path::PathBuf,
    pub crs: Option<CrsDef>,
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
    pub polygon_filter: LineString,
    pub drawing_polygon: bool,
}

impl Default for AreaSelectionState {
    fn default() -> Self {
        Self {
            polygon_filter: LineString::new(vec![]),
            drawing_polygon: false,
        }
    }
}

#[derive(Default)]
pub struct TileSelectionState {
    pub selected_tile: Option<usize>,
    pub subtile_boundaries: Vec<[walkers::Position; 4]>,
    pub subtile_neighbors: Vec<Neighborhood>,
}

impl TileSelectionState {
    pub fn validate_selected_tile(&self) -> Result<usize, StageValidationError> {
        let selected_tile = self
            .selected_tile
            .ok_or(StageValidationError::MissingSelectedTile)?;
        if selected_tile >= self.subtile_neighbors.len() {
            return Err(StageValidationError::InvalidSelectedTile);
        }

        Ok(selected_tile)
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
    pub polygon_filter: LineString,
    pub write_single_copc: bool,
}

pub struct ReadyForMapPreview {
    pub path: std::path::PathBuf,
    pub tile: Neighborhood,
    pub stats: LidarStats,
}

pub struct ReadyForFinalMap {
    pub map_params: MapParameters,
    pub file_params: FileParameters,
    pub polygon_filter: LineString,
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
                crs_epsg: self.project.crs_epsg.clone(),
            },
            output_crs: self.generation.params.output.crs.clone(),
            save_location: self.project.save_location.clone(),
            boundaries: self.lidar.boundaries.clone(),
            polygon_filter: self.area.polygon_filter.clone(),
            write_single_copc: self.project.write_single_copc,
        })
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
        let selected_file = self
            .project
            .selected_file
            .ok_or(StageValidationError::MissingSelectedFile)?;
        if selected_file >= self.project.paths.len() {
            return Err(StageValidationError::InvalidSelectedFile);
        }
        let selected_tile = self.tile.validate_selected_tile()?;
        let stats = self
            .lidar
            .stats
            .clone()
            .ok_or(StageValidationError::MissingLidarStats)?;

        Ok(ReadyForMapPreview {
            path: self.project.paths[selected_file].clone(),
            tile: self.tile.subtile_neighbors[selected_tile].clone(),
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

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn welcome_validation_requires_paths_and_save_location() {
        let project = ProjectFiles::default();

        assert!(matches!(
            project.validate_welcome(),
            Err(StageValidationError::MissingLidarFiles)
        ));
    }

    #[test]
    fn selected_file_validation_rejects_stale_index() {
        let project = ProjectFiles {
            paths: vec![std::path::PathBuf::from("one.laz")],
            selected_file: Some(1),
            ..Default::default()
        };

        assert!(matches!(
            project.validate_selected_file(),
            Err(StageValidationError::InvalidSelectedFile)
        ));
    }

    #[test]
    fn selected_tile_validation_rejects_stale_index() {
        let tile = TileSelectionState {
            selected_tile: Some(0),
            subtile_neighbors: vec![],
            ..Default::default()
        };

        assert!(matches!(
            tile.validate_selected_tile(),
            Err(StageValidationError::InvalidSelectedTile)
        ));
    }
}
