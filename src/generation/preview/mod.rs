mod initialize;
mod regenerate;

pub(crate) use initialize::initialize_map_tile;
pub(crate) use regenerate::regenerate_map_tile;

#[derive(Clone, Copy, Debug)]
pub(crate) enum RegenerationScope {
    Changed,
    Section(MapPreviewSection),
}

#[derive(Clone, Copy, Debug, Eq, Ord, PartialEq, PartialOrd)]
pub(crate) enum MapPreviewSection {
    Contours,
    Openness,
    Vegetation,
    Buildings,
    Cliffs,
    Water,
    Marsh,
    Streams,
    Intensity,
}
