use std::collections::HashMap;

use geo::LineString;
use walkers::Position;

use super::terminal_like::TerminalLike;
use crate::{
    drawable::{DrawOrder, DrawableOmap},
    neighbors::Neighborhood,
    parameters::{FileParameters, MapParameters},
    statistics::LidarStats,
};

#[derive(Debug, Default, Clone, Copy, PartialEq)]
pub enum TileProvider {
    #[default]
    OpenStreetMap,
    OpenTopoMap,
}

pub struct GuiVariables {
    // lidar file overlay
    pub boundaries: Vec<[Position; 4]>,

    pub polygon_filter: LineString,

    // lidar crs's
    pub crs_less_search_strings: Vec<String>,
    pub unique_crs: Vec<u16>,

    // lidar stats
    pub lidar_stats: Option<LidarStats>,

    // set output crs
    pub output_crs_string: String,

    // perform connected component
    pub connected_components: Vec<Vec<usize>>,

    // checkboxes
    pub drop_checkboxes: Vec<bool>,
    pub visibility_checkboxes: HashMap<omap::symbols::Symbol, bool>,
    // true when the backend is busy generating a map tile
    pub generating_map_tile: bool,

    // logging to the in app "console"
    pub log_terminal: TerminalLike,

    pub map_params: MapParameters,
    pub file_params: FileParameters,

    // sub_tile parameters
    pub selected_tile: Option<usize>,
    pub subtile_boundaries: Vec<[walkers::Position; 4]>,
    pub subtile_neighbors: Vec<Neighborhood>,

    // for storing the generated map tile for drawing
    pub map_tile: Option<DrawableOmap>,
    pub map_opacity: f32,

    // the contour "score" (error, energy)
    pub contour_score: (f32, f32),

    // tile provider
    pub tile_provider: TileProvider,
}

impl Default for GuiVariables {
    fn default() -> Self {
        let mut visibility_checkboxes = HashMap::new();
        for symbol in omap::symbols::Symbol::draw_order() {
            visibility_checkboxes.insert(symbol, true);
        }

        Self {
            visibility_checkboxes,
            map_opacity: 1.0,
            polygon_filter: LineString::new(vec![]),

            boundaries: Default::default(),
            crs_less_search_strings: Default::default(),
            unique_crs: Default::default(),
            output_crs_string: Default::default(),
            connected_components: Default::default(),
            drop_checkboxes: Default::default(),
            log_terminal: Default::default(),
            map_params: Default::default(),
            file_params: Default::default(),
            map_tile: Default::default(),
            generating_map_tile: Default::default(),
            selected_tile: Default::default(),
            subtile_boundaries: Default::default(),
            subtile_neighbors: Default::default(),
            contour_score: Default::default(),
            tile_provider: Default::default(),
            lidar_stats: Default::default(),
        }
    }
}

impl GuiVariables {
    pub fn get_most_popular_crs(&self) -> Option<u16> {
        let mut crs_tally = std::collections::HashMap::new();
        for crs in self.file_params.crs_epsg.iter() {
            if let Some(val) = crs_tally.get_mut(crs) {
                *val += 1;
            } else {
                crs_tally.insert(crs, 1_u16);
            }
        }
        if let Some((max_crs, _)) = crs_tally.drain().max_by(|(_, v1), (_, v2)| v1.cmp(v2)) {
            Some(*max_crs)
        } else {
            None
        }
    }

    pub fn update_unique_crs(&mut self) {
        self.unique_crs.clear();
        for crs in self.file_params.crs_epsg.iter() {
            if *crs != u16::MAX && !self.unique_crs.contains(crs) {
                self.unique_crs.push(*crs);
            }
        }
    }

    pub fn drop_small_graph_components(&mut self) -> Position {
        let mut drop_files = vec![];

        let mut biggest_component_index = 0;
        let mut biggest_component_size = 0;
        for (i, v) in self.connected_components.iter().enumerate() {
            if v.len() > biggest_component_size {
                biggest_component_size = v.len();
                biggest_component_index = i;
            }
        }

        for (i, v) in self.connected_components.iter().enumerate() {
            if i == biggest_component_index {
                continue;
            }
            for fi in v.iter() {
                drop_files.push(*fi);
            }
        }

        drop_files.sort_by(|a, b| b.cmp(a));

        for drop_file in drop_files {
            self.file_params.paths.remove(drop_file);
            self.file_params.crs_epsg.remove(drop_file);
            self.boundaries.remove(drop_file);
        }

        let mut new_home = (0., 0.);
        for bound in self.boundaries.iter() {
            new_home.0 += (bound[0].x + bound[2].x) / 2.;
            new_home.1 += (bound[0].y + bound[2].y) / 2.;
        }
        new_home.0 /= self.boundaries.len() as f64;
        new_home.1 /= self.boundaries.len() as f64;

        walkers::pos_from_lon_lat(new_home.0, new_home.1)
    }

    pub fn update_map(&mut self, other: DrawableOmap) {
        if let Some(map) = &mut self.map_tile {
            map.update(other);
        } else {
            self.map_tile = Some(other);
        }
    }
}
