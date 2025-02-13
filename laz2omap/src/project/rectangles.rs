use geo::{Coord, Rect};
use proj4rs::{transform::transform, Proj};

pub fn to_walkers_map_coords(epsg: Option<u16>, rects: &Vec<Rect>) -> Vec<[Coord; 4]> {
    let mut out = Vec::with_capacity(rects.len());
    if epsg.is_none() {
        for rect in rects {
            out.push([
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
            ]);
        }
    } else if epsg.is_some() {
        let epsg = epsg.unwrap();

        let global_proj = Proj::from_epsg_code(4326).unwrap();
        let local_proj = Proj::from_epsg_code(epsg).unwrap();

        for rect in rects {
            let mut points = [
                (rect.min().x, rect.max().y),
                rect.min().x_y(),
                (rect.max().x, rect.min().y),
                rect.max().x_y(),
            ];

            transform(&local_proj, &global_proj, points.as_mut_slice()).unwrap();

            out.push([
                Coord {
                    x: points[0].0.to_degrees(),
                    y: points[0].1.to_degrees(),
                },
                Coord {
                    x: points[1].0.to_degrees(),
                    y: points[1].1.to_degrees(),
                },
                Coord {
                    x: points[2].0.to_degrees(),
                    y: points[2].1.to_degrees(),
                },
                Coord {
                    x: points[3].0.to_degrees(),
                    y: points[3].1.to_degrees(),
                },
            ]);
        }
    }
    out
}
