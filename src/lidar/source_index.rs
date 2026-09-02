use std::path::{Path, PathBuf};

use anyhow::Context;
use geo::Intersects;
use las::CopcReader;
use rstar::{AABB, RTree, RTreeObject};

use crate::{Error, MIN_TILE_OVERLAP_METERS, Result, geometry::MapRect};

#[derive(Clone)]
pub(crate) struct LidarSource {
    path: PathBuf,
    bounds: geo::Rect,
    elevation_midpoint: f64,
}

impl LidarSource {
    pub(crate) fn path(&self) -> &Path {
        &self.path
    }
}

#[derive(Clone, Copy)]
struct IndexedSource {
    envelope: AABB<[f64; 2]>,
    source_index: usize,
}

impl RTreeObject for IndexedSource {
    type Envelope = AABB<[f64; 2]>;

    fn envelope(&self) -> Self::Envelope {
        self.envelope
    }
}

/// Spatial index shared by preview and final generation.
///
/// COPC header bounds are the only spatial criterion. Every source whose bounds
/// overlap a query is returned, including all sources in overlap areas.
pub(crate) struct LidarSourceIndex {
    sources: Vec<LidarSource>,
    tree: RTree<IndexedSource>,
    processing_bounds: Vec<geo::Rect>,
    ref_point: geo::Coord,
    average_elevation: f64,
}

impl LidarSourceIndex {
    pub(crate) fn new(paths: &[PathBuf], polygon_filter: Option<&geo::Polygon>) -> Result<Self> {
        // Source order must never be an implicit overlap policy. Sorting also
        // makes exact-duplicate tie breaking stable across file-picker order.
        let mut paths = paths
            .iter()
            .map(|path| std::fs::canonicalize(path).unwrap_or_else(|_| path.clone()))
            .collect::<Vec<_>>();
        paths.sort();
        paths.dedup();

        let mut sources = Vec::with_capacity(paths.len());
        for path in paths {
            let reader = CopcReader::from_path(&path)
                .with_context(|| format!("Failed to index COPC source {}", path.display()))?;
            let header_bounds = reader.header().bounds();
            let bounds = geo::Rect::from_bounds(header_bounds);

            if polygon_filter.is_some_and(|polygon| !bounds.intersects(polygon)) {
                continue;
            }

            sources.push(LidarSource {
                path,
                bounds,
                elevation_midpoint: (header_bounds.min.z + header_bounds.max.z) / 2.,
            });
        }

        if sources.is_empty() {
            return Err(Error::MapAreaDistinctFromLidarArea.into());
        }

        let indexed_sources = sources
            .iter()
            .enumerate()
            .map(|(source_index, source)| IndexedSource {
                envelope: rect_envelope(source.bounds),
                source_index,
            })
            .collect();
        let tree = RTree::bulk_load(indexed_sources);

        let processing_bounds = connected_processing_bounds(&sources, &tree);
        let bounds = combined_bounds(&sources).context("COPC source bounds contain no area")?;
        let ref_point = geo::Coord {
            x: ((bounds.min().x + bounds.max().x) / 20.).round() * 10.,
            y: ((bounds.min().y + bounds.max().y) / 20.).round() * 10.,
        };
        let average_elevation = (sources
            .iter()
            .map(|source| source.elevation_midpoint)
            .sum::<f64>()
            / (10 * sources.len()) as f64)
            .round()
            * 10.;

        Ok(Self {
            sources,
            tree,
            processing_bounds,
            ref_point,
            average_elevation,
        })
    }

    pub(crate) fn sources_intersecting(&self, bounds: geo::Rect) -> Vec<&LidarSource> {
        let mut sources = self
            .tree
            .locate_in_envelope_intersecting(rect_envelope(bounds))
            .map(|indexed| &self.sources[indexed.source_index])
            .collect::<Vec<_>>();
        sources.sort_by(|a, b| a.path.cmp(&b.path));
        sources
    }

    pub(crate) fn intersects(&self, bounds: geo::Rect) -> bool {
        self.tree
            .locate_in_envelope_intersecting(rect_envelope(bounds))
            .next()
            .is_some()
    }

    pub(crate) fn processing_bounds(&self) -> &[geo::Rect] {
        &self.processing_bounds
    }

    pub(crate) fn ref_point(&self) -> geo::Coord {
        self.ref_point
    }

    pub(crate) fn average_elevation(&self) -> f64 {
        self.average_elevation
    }

    pub(crate) fn len(&self) -> usize {
        self.sources.len()
    }
}

fn rect_envelope(rect: geo::Rect) -> AABB<[f64; 2]> {
    AABB::from_corners([rect.min().x, rect.min().y], [rect.max().x, rect.max().y])
}

fn expanded_rect_envelope(rect: geo::Rect, margin: f64) -> AABB<[f64; 2]> {
    AABB::from_corners(
        [rect.min().x - margin, rect.min().y - margin],
        [rect.max().x + margin, rect.max().y + margin],
    )
}

fn combined_bounds(sources: &[LidarSource]) -> Option<geo::Rect> {
    let first = sources.first()?.bounds;
    Some(sources.iter().skip(1).fold(first, |bounds, source| {
        geo::Rect::new(
            (
                bounds.min().x.min(source.bounds.min().x),
                bounds.min().y.min(source.bounds.min().y),
            ),
            (
                bounds.max().x.max(source.bounds.max().x),
                bounds.max().y.max(source.bounds.max().y),
            ),
        )
    }))
}

/// Group source bounds that are close enough to share a processing halo.
///
/// Each group is tiled as one envelope. This prevents an outer cut margin from
/// being applied at an internal source-file boundary while still avoiding a
/// potentially huge grid between genuinely disconnected acquisitions.
fn connected_processing_bounds(
    sources: &[LidarSource],
    tree: &RTree<IndexedSource>,
) -> Vec<geo::Rect> {
    let mut visited = vec![false; sources.len()];
    let mut processing_bounds = Vec::new();

    for start in 0..sources.len() {
        if visited[start] {
            continue;
        }

        visited[start] = true;
        let mut pending = vec![start];
        let mut bounds = sources[start].bounds;
        while let Some(current) = pending.pop() {
            let source_bounds = sources[current].bounds;
            bounds = geo::Rect::new(
                (
                    bounds.min().x.min(source_bounds.min().x),
                    bounds.min().y.min(source_bounds.min().y),
                ),
                (
                    bounds.max().x.max(source_bounds.max().x),
                    bounds.max().y.max(source_bounds.max().y),
                ),
            );

            for candidate in tree.locate_in_envelope_intersecting(expanded_rect_envelope(
                source_bounds,
                MIN_TILE_OVERLAP_METERS,
            )) {
                if !visited[candidate.source_index] {
                    visited[candidate.source_index] = true;
                    pending.push(candidate.source_index);
                }
            }
        }
        processing_bounds.push(bounds);
    }

    processing_bounds
}

#[cfg(test)]
mod tests {
    use super::*;

    fn in_memory_index(bounds: Vec<(&str, geo::Rect)>) -> LidarSourceIndex {
        let sources = bounds
            .into_iter()
            .map(|(path, bounds)| LidarSource {
                path: PathBuf::from(path),
                bounds,
                elevation_midpoint: 0.,
            })
            .collect::<Vec<_>>();
        let tree = RTree::bulk_load(
            sources
                .iter()
                .enumerate()
                .map(|(source_index, source)| IndexedSource {
                    envelope: rect_envelope(source.bounds),
                    source_index,
                })
                .collect(),
        );
        let processing_bounds = connected_processing_bounds(&sources, &tree);
        LidarSourceIndex {
            sources,
            tree,
            processing_bounds,
            ref_point: geo::Coord::zero(),
            average_elevation: 0.,
        }
    }

    #[test]
    fn discovery_uses_the_complete_header_bounds() {
        let index = in_memory_index(vec![(
            "source.copc.laz",
            geo::Rect::new((0., 0.), (2., 2.)),
        )]);

        assert!(index.intersects(geo::Rect::new((0.25, 1.25), (0.75, 1.75))));
        assert!(index.intersects(geo::Rect::new((1.25, 1.25), (1.75, 1.75))));
        assert_eq!(
            index
                .sources_intersecting(geo::Rect::new((1.25, 1.25), (1.75, 1.75)))
                .len(),
            1
        );
        assert!(!index.intersects(geo::Rect::new((2.25, 1.25), (2.75, 1.75))));
    }

    #[test]
    fn query_returns_every_source_in_an_overlap_in_stable_order() {
        let first = geo::Rect::new((0., 0.), (2., 2.));
        let second = geo::Rect::new((1., 1.), (3., 3.));
        let index = in_memory_index(vec![("z.copc.laz", first), ("a.copc.laz", second)]);

        let paths = index
            .sources_intersecting(geo::Rect::new((1.25, 1.25), (1.75, 1.75)))
            .into_iter()
            .map(|source| source.path().to_owned())
            .collect::<Vec<_>>();

        assert_eq!(
            paths,
            vec![PathBuf::from("a.copc.laz"), PathBuf::from("z.copc.laz")]
        );
    }

    #[test]
    fn query_halo_crossing_a_file_edge_returns_both_files() {
        let index = in_memory_index(vec![
            ("left.copc.laz", geo::Rect::new((0., 0.), (10., 10.))),
            ("right.copc.laz", geo::Rect::new((10., 0.), (20., 10.))),
        ]);

        let paths = index
            .sources_intersecting(geo::Rect::new((8., 2.), (12., 8.)))
            .into_iter()
            .map(|source| source.path().to_owned())
            .collect::<Vec<_>>();

        assert_eq!(
            paths,
            vec![
                PathBuf::from("left.copc.laz"),
                PathBuf::from("right.copc.laz")
            ]
        );
    }

    #[test]
    fn adjacent_file_bounds_share_one_processing_envelope() {
        let index = in_memory_index(vec![
            ("left.copc.laz", geo::Rect::new((0., 0.), (10., 10.))),
            ("right.copc.laz", geo::Rect::new((10.5, 0.), (20.5, 10.))),
        ]);

        assert_eq!(
            index.processing_bounds(),
            &[geo::Rect::new((0., 0.), (20.5, 10.))]
        );
    }

    #[test]
    fn distant_sources_keep_separate_processing_envelopes() {
        let index = in_memory_index(vec![
            ("first.copc.laz", geo::Rect::new((0., 0.), (10., 10.))),
            ("second.copc.laz", geo::Rect::new((50., 50.), (60., 60.))),
        ]);

        assert_eq!(
            index.processing_bounds(),
            &[
                geo::Rect::new((0., 0.), (10., 10.)),
                geo::Rect::new((50., 50.), (60., 60.))
            ]
        );
    }
}
