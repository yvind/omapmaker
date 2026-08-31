use std::{collections::VecDeque, sync::Arc};

use crate::parameters::{BuildingClassificationEvidence, BuildingParameters, MapParameters};

pub(super) struct BoundedCache<K, V, const N: usize> {
    pub(super) entries: VecDeque<(K, Arc<V>)>,
}

impl<K, V, const N: usize> Default for BoundedCache<K, V, N> {
    fn default() -> Self {
        Self {
            entries: VecDeque::new(),
        }
    }
}

impl<K: PartialEq, V, const N: usize> BoundedCache<K, V, N> {
    pub(super) fn get(&mut self, key: &K) -> Option<Arc<V>> {
        let position = self
            .entries
            .iter()
            .position(|(candidate, _)| candidate == key)?;
        let entry = self
            .entries
            .remove(position)
            .expect("cache position exists");
        let value = Arc::clone(&entry.1);
        self.entries.push_back(entry);
        Some(value)
    }

    pub(super) fn insert(&mut self, key: K, value: Arc<V>) {
        if self.entries.len() == N {
            self.entries.pop_front();
        }
        self.entries.push_back((key, value));
    }
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub(super) struct TerrainFitCacheKey {
    tile_revision: u64,
    slope_radius_bits: u64,
    curvature_radius_bits: u64,
}

impl TerrainFitCacheKey {
    pub(super) fn new(tile_revision: u64, params: &MapParameters) -> Self {
        Self {
            tile_revision,
            slope_radius_bits: params.contour.contour_field.slope_fit_radius_m.to_bits(),
            curvature_radius_bits: params
                .contour
                .contour_field
                .curvature_fit_radius_m
                .to_bits(),
        }
    }
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub(super) struct ContourFieldCacheKey {
    tile_revision: u64,
    interval_bits: u32,
    contour_field_fingerprint: u64,
}

impl ContourFieldCacheKey {
    pub(super) fn new(tile_revision: u64, params: &MapParameters) -> Self {
        Self {
            tile_revision,
            interval_bits: params.contour.interval.to_bits(),
            contour_field_fingerprint: params.contour.contour_field.fingerprint(),
        }
    }
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub(super) struct BuildingFitCacheKey {
    tile_revision: u64,
    minimum_roof_height_bits: u32,
    maximum_roof_height_bits: u32,
    plane_fit_radius_bits: u64,
    maximum_plane_residual_bits: u32,
    ransac_iterations: usize,
    ransac_sample_size: usize,
    minimum_plane_inliers: usize,
    maximum_roof_planes: usize,
    maximum_roof_slope_bits: u32,
    maximum_candidate_hole_area_bits: u64,
    merge_gap_bits: u64,
    class_6_evidence: u8,
}

impl BuildingFitCacheKey {
    pub(super) fn new(tile_revision: u64, params: &BuildingParameters) -> Self {
        Self {
            tile_revision,
            minimum_roof_height_bits: params.minimum_roof_height_m.to_bits(),
            maximum_roof_height_bits: params.maximum_roof_height_m.to_bits(),
            plane_fit_radius_bits: params.plane_fit_radius_m.to_bits(),
            maximum_plane_residual_bits: params.maximum_plane_residual_m.to_bits(),
            ransac_iterations: params.ransac_iterations,
            ransac_sample_size: params.ransac_sample_size,
            minimum_plane_inliers: params.minimum_plane_inliers,
            maximum_roof_planes: params.maximum_roof_planes,
            maximum_roof_slope_bits: params.maximum_roof_slope_degrees.to_bits(),
            maximum_candidate_hole_area_bits: params.maximum_candidate_hole_area_m2.to_bits(),
            merge_gap_bits: params.merge_gap_m.to_bits(),
            class_6_evidence: match params.class_6_evidence {
                BuildingClassificationEvidence::Authoritative => 0,
                BuildingClassificationEvidence::Supporting => 1,
                BuildingClassificationEvidence::Ignore => 2,
            },
        }
    }
}
