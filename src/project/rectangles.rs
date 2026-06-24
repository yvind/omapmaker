use geo::{Coord, Point, Rect};
use proj_core::Transform;

pub fn to_walkers_map_points(epsg: Option<u16>, rects: &Vec<Rect>) -> Vec<[Point; 4]> {
    let mut out = Vec::with_capacity(rects.len());
    if epsg.is_none() {
        for rect in rects {
            out.push([
                Point(Coord {
                    x: rect.min().x,
                    y: rect.max().y,
                }),
                rect.min().into(),
                Point(Coord {
                    x: rect.max().x,
                    y: rect.min().y,
                }),
                rect.max().into(),
            ]);
        }
    } else if let Some(epsg) = epsg {
        let transform = Transform::from_epsg(epsg as u32, 4326).unwrap();

        for rect in rects {
            let transformed_polygon = transform.convert_geometry(rect.to_polygon()).unwrap();

            out.push([
                Point(transformed_polygon.exterior().0[0]),
                Point(transformed_polygon.exterior().0[1]),
                Point(transformed_polygon.exterior().0[2]),
                Point(transformed_polygon.exterior().0[3]),
            ]);
        }
    }
    out
}
