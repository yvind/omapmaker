pub use geo::Coord;
pub trait MapCoord {
    fn to_map_coordinates(self) -> Result<(i32, i32), &'static str>;
}

impl MapCoord for Coord {
    fn to_map_coordinates(self) -> Result<(i32, i32), &'static str> {
        // 1_000 map units = 15m
        // 1_000 / 15 = 66.66...

        let x = (self.x * 66.6666666).round();
        let y = -(self.y * 66.6666666).round();

        if (x > 2.0_f64.powi(31) - 1.) || (y > 2.0_f64.powi(31) - 1.) {
            Err("map coordinate overflow, double check that all lidar files are over the same general area and in the same coordinate refrence system. Or try fewer files at a time")
        } else {
            Ok((x as i32, y as i32))
        }
    }
}
