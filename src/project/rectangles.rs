use geo::{Coord, Point, Rect};
use proj_core::{CrsDef, Transform};

pub fn to_walkers_map_points(crs: Option<CrsDef>, rects: &Vec<Rect>) -> Vec<[Point; 4]> {
    let mut out = Vec::with_capacity(rects.len());
    if crs.is_none() {
        for rect in rects {
            out.push(rect_to_map_coords(rect).map(Point));
        }
    } else if let Some(crs) = crs {
        let transform = Transform::from_epsg(crs.epsg(), 4326).unwrap();

        for rect in rects {
            out.push(rect_to_map_coords(rect).map(|point| {
                let transformed = transform.convert((point.x, point.y)).unwrap();
                Point(Coord {
                    x: transformed.0,
                    y: transformed.1,
                })
            }));
        }
    }
    out
}

fn rect_to_map_coords(rect: &Rect) -> [Coord; 4] {
    [
        Coord {
            x: rect.min().x,
            y: rect.max().y,
        },
        rect.min(),
        Coord {
            x: rect.max().x,
            y: rect.min().y,
        },
        rect.max(),
    ]
}
