pub use geo::Coord;
pub trait MapCoord {
    fn to_map_coordinates(self) -> Result<(i32, i32), &'static str>;
}

// scale 1:15_000 and 1 map unit is 0.001mm on paper
// 1_000 map units on paper = 15m on ground
const CONVERSION: f64 = 1_000. / 15.;

impl MapCoord for Coord {
    fn to_map_coordinates(self) -> Result<(i32, i32), &'static str> {
        let x = (self.x * CONVERSION).round();
        let y = -(self.y * CONVERSION).round();

        if (x > 2.0_f64.powi(31) - 1.) || (y > 2.0_f64.powi(31) - 1.) {
            Err("Map coordinate overflow, double check that all lidar files are over the same general area and in the same coordinate refrence system. (Max size is 32_000km from the avg position of all file bounds given in the las header, i.e 3/4 earth's circumference)")
        } else {
            Ok((x as i32, y as i32))
        }
    }
}
