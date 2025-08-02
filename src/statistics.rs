use las::Reader;
use std::{ops::Div, path::Path};

const MAX_NUMBER_OF_RETURNS: u8 = 15;

#[derive(Debug, Clone)]
pub struct LidarStats {
    pub return_distr: Vec<u64>,
    pub return_number: Stat,
    pub intensity: Stat,
}

impl LidarStats {
    pub fn calculate_statistics(path: impl AsRef<Path>) -> crate::Result<LidarStats> {
        let mut reader = Reader::from_path(path)?;
        let header = reader.header();
        let num_points = header.number_of_points();

        let num_points_by_return = (1..=MAX_NUMBER_OF_RETURNS)
            .map(|i| header.number_of_points_by_return(i).unwrap_or(0))
            .collect::<Vec<_>>();

        let mut return_number_stat = Stat {
            num_points: num_points as f64,
            ..Default::default()
        };

        return_number_stat.min = num_points_by_return
            .iter()
            .enumerate()
            .find_map(|(i, &v)| if v > 0 { Some(i + 1) } else { None })
            .unwrap_or(0) as f64;

        return_number_stat.max = num_points_by_return
            .iter()
            .enumerate()
            .rev()
            .find_map(|(i, &v)| if v > 0 { Some(i + 1) } else { None })
            .unwrap_or(0) as f64;

        return_number_stat.mean = num_points_by_return
            .iter()
            .enumerate()
            .fold(0, |acc, (i, &v)| acc + ((i + 1) as u64 * v))
            as f64
            / return_number_stat.num_points;

        return_number_stat.std_dev = num_points_by_return
            .iter()
            .enumerate()
            .fold(0., |acc, (i, &n)| {
                acc + n as f64 * ((i + 1) as f64 - return_number_stat.mean).powi(2)
            })
            .div(return_number_stat.num_points)
            .sqrt();

        let mut intensities = Vec::with_capacity(num_points as usize);
        let mut intensity_stat = Stat {
            num_points: num_points as f64,
            ..Default::default()
        };

        for point in reader.points().filter_map(Result::ok) {
            let i = point.intensity as f64;
            intensities.push(i);
            if i < intensity_stat.min {
                intensity_stat.min = i;
            } else if i > intensity_stat.max {
                intensity_stat.max = i;
            }
            intensity_stat.mean += i;
        }
        intensity_stat.mean /= intensity_stat.num_points;

        intensity_stat.std_dev = intensities
            .into_iter()
            .fold(0., |acc, i| acc + (i - intensity_stat.mean).powi(2))
            .div(intensity_stat.num_points)
            .sqrt();

        Ok(LidarStats {
            return_distr: num_points_by_return,
            return_number: return_number_stat,
            intensity: intensity_stat,
        })
    }

    pub fn combine_stats(self, other: LidarStats) -> LidarStats {
        let total_return_distr = self
            .return_distr
            .into_iter()
            .zip(other.return_distr)
            .map(|(s, o)| s + o)
            .collect::<Vec<_>>();

        LidarStats {
            return_distr: total_return_distr,
            return_number: self.return_number.combine_stats(other.return_number),
            intensity: self.intensity.combine_stats(other.intensity),
        }
    }
}

#[derive(Debug, Clone, Copy)]
pub struct Stat {
    pub min: f64,
    pub max: f64,
    pub mean: f64,
    pub std_dev: f64,
    pub num_points: f64,
}

impl Stat {
    pub fn combine_stats(self, other: Stat) -> Stat {
        Stat {
            min: self.min.min(other.min),
            max: self.max.max(other.max),
            mean: (self.mean * self.num_points + other.mean * other.num_points)
                / (self.num_points + other.num_points),
            std_dev: (self.std_dev.powi(2) + other.std_dev.powi(2)).sqrt(),
            num_points: (self.num_points + other.num_points),
        }
    }
}

impl Default for Stat {
    fn default() -> Self {
        Self {
            min: f64::MAX,
            max: f64::MIN,
            mean: 0.,
            std_dev: 0.,
            num_points: 0.,
        }
    }
}
