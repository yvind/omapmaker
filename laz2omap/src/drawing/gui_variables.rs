use std::path::PathBuf;
use walkers::Position;

use super::DrawableOmap;
use super::TerminalLike;

#[derive(Clone)]
pub struct GuiVariables {
    // path inputs
    pub paths: Vec<PathBuf>,
    pub save_location: Option<PathBuf>,
    pub tiff_location: Option<PathBuf>,

    // lidar file overlay
    pub selected_file: Option<usize>,
    pub boundaries: Vec<[Position; 4]>,
    pub polygon_filter: Vec<Position>,

    // lidar crs's
    pub crs_epsg: Vec<u16>,
    pub crs_less_search_strings: Vec<String>,
    pub unique_crs: Vec<u16>,

    // set output crs
    pub output_crs_string: String,
    pub output_epsg: Option<u16>,

    // perform connected component
    pub connected_components: Vec<Vec<usize>>,

    // map parameters
    pub simplification_distance: f64,
    pub bezier_error: f64,
    pub basemap_interval: f64,
    pub contour_interval: f64,
    pub green: (f64, f64, f64),
    pub yellow: f64,

    // debug params
    pub contour_algo_steps: u8,
    pub contour_algo_lambda: f64,

    // checkboxes
    pub drop_checkboxes: Vec<bool>,
    pub save_tiffs: bool,
    pub basemap_contour: bool,
    pub formlines: bool,
    pub bezier_bool: bool,

    // logging to in app console
    pub log_string: TerminalLike,

    // for storing the generated map tile for drawing
    pub map_tile: Option<Box<DrawableOmap>>,
}

impl Default for GuiVariables {
    fn default() -> Self {
        Self {
            paths: Default::default(),
            save_location: Default::default(),
            tiff_location: Default::default(),

            selected_file: Default::default(),
            boundaries: Default::default(),
            polygon_filter: Default::default(),

            crs_epsg: Default::default(),
            crs_less_search_strings: Default::default(),
            unique_crs: Default::default(),
            output_crs_string: Default::default(),
            output_epsg: None,
            connected_components: Default::default(),

            simplification_distance: 0.1,
            bezier_error: 0.5,
            basemap_interval: 0.5,
            contour_interval: 5.,
            green: (0.2, 0.4, 0.6),
            yellow: 0.1,
            contour_algo_steps: 5,
            contour_algo_lambda: 1.,

            drop_checkboxes: Default::default(),
            save_tiffs: false,
            basemap_contour: false,
            formlines: false,
            bezier_bool: true,

            log_string: Default::default(),
            map_tile: None,
        }
    }
}

impl GuiVariables {
    pub fn get_most_popular_crs(&self) -> Option<u16> {
        let mut crs_tally = std::collections::HashMap::new();
        for crs in self.crs_epsg.iter() {
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
        for crs in self.crs_epsg.iter() {
            if *crs != u16::MAX && !self.unique_crs.contains(crs) {
                self.unique_crs.push(*crs);
            }
        }
    }

    pub fn drop_small_graph_components(&mut self) {
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
            self.paths.remove(drop_file);
            self.crs_epsg.remove(drop_file);
            self.boundaries.remove(drop_file);
        }
    }

    pub fn update_map(&mut self, map: Box<DrawableOmap>) {
        self.map_tile = Some(map);
    }
}
