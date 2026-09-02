use geo::{Distance, Euclidean};

use crate::LIDAR_BOUNDS_TOUCH_MARGIN_METERS;

/// Find connected components of LiDAR header bounds using the application's
/// single shared definition of "touching".
pub(crate) fn connected_bounds_components(bounds: &[geo::Rect]) -> Vec<Vec<usize>> {
    connected_components(bounds.len(), |current, candidate| {
        Euclidean.distance(&bounds[current], &bounds[candidate]) <= LIDAR_BOUNDS_TOUCH_MARGIN_METERS
    })
}

/// Find connected components in a clipped map area using the same distance
/// tolerance as LiDAR bounds connectivity.
pub(crate) fn connected_polygon_components(polygons: &[geo::Polygon]) -> Vec<Vec<usize>> {
    connected_components(polygons.len(), |current, candidate| {
        Euclidean.distance(&polygons[current], &polygons[candidate])
            <= LIDAR_BOUNDS_TOUCH_MARGIN_METERS
    })
}

fn connected_components(
    len: usize,
    mut connected: impl FnMut(usize, usize) -> bool,
) -> Vec<Vec<usize>> {
    let mut visited = vec![false; len];
    let mut components = Vec::new();

    for start in 0..len {
        if visited[start] {
            continue;
        }

        visited[start] = true;
        let mut pending = vec![start];
        let mut component = Vec::new();
        while let Some(current) = pending.pop() {
            component.push(current);
            for candidate in 0..len {
                if !visited[candidate] && connected(current, candidate) {
                    visited[candidate] = true;
                    pending.push(candidate);
                }
            }
        }
        component.sort_unstable();
        components.push(component);
    }

    components
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn exact_and_near_touches_share_a_component_but_larger_gaps_do_not() {
        let margin = LIDAR_BOUNDS_TOUCH_MARGIN_METERS;
        let bounds = [
            geo::Rect::new((0., 0.), (10., 10.)),
            geo::Rect::new((10., 0.), (20., 10.)),
            geo::Rect::new((20. + margin, 0.), (30. + margin, 10.)),
            geo::Rect::new(
                (30. + 2. * margin + 1e-6, 0.),
                (40. + 2. * margin + 1e-6, 10.),
            ),
        ];

        assert_eq!(
            connected_bounds_components(&bounds),
            vec![vec![0, 1, 2], vec![3]]
        );
    }

    #[test]
    fn connectivity_is_transitive_for_non_rectangular_layouts() {
        let bounds = [
            geo::Rect::new((0., 0.), (10., 10.)),
            geo::Rect::new((10., 0.), (20., 10.)),
            geo::Rect::new((0., 10.), (10., 20.)),
            geo::Rect::new((100., 100.), (110., 110.)),
        ];

        assert_eq!(
            connected_bounds_components(&bounds),
            vec![vec![0, 1, 2], vec![3]]
        );
    }

    #[test]
    fn clipped_polygons_use_the_shared_touch_margin() {
        let margin = LIDAR_BOUNDS_TOUCH_MARGIN_METERS;
        let polygons = [
            geo::Rect::new((0., 0.), (10., 10.)).to_polygon(),
            geo::Rect::new((10. + margin, 0.), (20. + margin, 10.)).to_polygon(),
            geo::Rect::new(
                (20. + 2. * margin + 1e-6, 0.),
                (30. + 2. * margin + 1e-6, 10.),
            )
            .to_polygon(),
        ];

        assert_eq!(
            connected_polygon_components(&polygons),
            vec![vec![0, 1], vec![2]]
        );
    }
}
