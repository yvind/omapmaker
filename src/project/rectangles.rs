use geo::{Coord, Point, Rect};
use proj_core::{CrsDef, Transform};

pub fn to_walkers_map_points(
    crs: Option<CrsDef>,
    rects: &Vec<Rect>,
) -> crate::Result<Vec<[Point; 4]>> {
    let mut out = Vec::with_capacity(rects.len());
    let Some(crs) = crs else {
        for rect in rects {
            out.push(rect_to_map_coords(rect).map(Point));
        }
        return Ok(out);
    };
    let transform = Transform::from_epsg(crs.epsg(), 4326)?;

    for rect in rects {
        let mut projected = [Point(Coord::default()); 4];
        for (i, point) in rect_to_map_coords(rect).into_iter().enumerate() {
            let transformed = transform.convert((point.x, point.y))?;
            projected[i] = Point(Coord {
                x: transformed.0,
                y: transformed.1,
            });
        }
        out.push(projected);
    }
    Ok(out)
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
