use las::Reader;
use std::path::Path;

const STATS_SAMPLE_SIZE: usize = 10_000;
const MAX_NUMBER_OF_RETURNS: u8 = 15;

#[derive(Debug, Clone)]
pub struct LidarStats {
    pub return_distr: Vec<u64>,
    pub return_number: Stat,
    pub intensity: Stat,
    /// Average number of lidar points per square metre, weighted by the
    /// footprint area of every input file included in these statistics.
    pub average_density: f64,
    pub(crate) density_area: f64,
}

impl LidarStats {
    pub fn calculate_statistics(path: impl AsRef<Path>) -> crate::Result<LidarStats> {
        let mut reader = Reader::from_path(path)?;
        let header = reader.header();
        let num_points = header.number_of_points();
        let bounds = header.bounds();
        let density_area = ((bounds.max.x - bounds.min.x) * (bounds.max.y - bounds.min.y)).max(0.);
        let average_density = if density_area > f64::EPSILON {
            num_points as f64 / density_area
        } else {
            0.
        };

        let num_points_by_return = (1..=MAX_NUMBER_OF_RETURNS)
            .map(|i| header.number_of_points_by_return(i).unwrap_or(0))
            .collect::<Vec<_>>();

        let mut return_number_stat = Stat {
            num_points: num_points as f32,
            ..Default::default()
        };

        return_number_stat.min = num_points_by_return
            .iter()
            .enumerate()
            .find_map(|(i, &v)| if v > 0 { Some(i + 1) } else { None })
            .unwrap_or(0) as f32;

        return_number_stat.max = num_points_by_return
            .iter()
            .enumerate()
            .rev()
            .find_map(|(i, &v)| if v > 0 { Some(i + 1) } else { None })
            .unwrap_or(0) as f32;

        let return_mean = num_points_by_return
            .iter()
            .enumerate()
            .fold(0, |acc, (i, &v)| acc + ((i + 1) as u64 * v)) as f64
            / num_points as f64;
        return_number_stat.mean = return_mean as f32;

        return_number_stat.std_dev = (num_points_by_return
            .iter()
            .enumerate()
            .fold(0., |acc, (i, &n)| {
                acc + n as f64 * ((i + 1) as f64 - return_mean).powi(2)
            })
            / num_points as f64)
            .sqrt() as f32;

        let mut intensities = Vec::with_capacity(num_points as usize);
        let mut intensity_stat = Stat {
            num_points: num_points as f32,
            ..Default::default()
        };

        let mut intensity_sum = 0_f64;
        let mut num_taken_points = 0_u64;
        for point in reader
            .points()
            .filter_map(Result::ok)
            .take(STATS_SAMPLE_SIZE)
        {
            let i = f64::from(point.intensity);
            intensities.push(i);
            if i < f64::from(intensity_stat.min) {
                intensity_stat.min = i as f32;
            } else if i > f64::from(intensity_stat.max) {
                intensity_stat.max = i as f32;
            }
            intensity_sum += i;
            num_taken_points += 1;
        }
        let intensity_mean = intensity_sum / num_taken_points as f64;
        intensity_stat.mean = intensity_mean as f32;

        intensity_stat.std_dev = (intensities
            .into_iter()
            .fold(0., |acc, i| acc + (i - intensity_mean).powi(2))
            / num_taken_points as f64)
            .sqrt() as f32;

        Ok(LidarStats {
            return_distr: num_points_by_return,
            return_number: return_number_stat,
            intensity: intensity_stat,
            average_density,
            density_area,
        })
    }

    pub fn combine_stats(self, other: LidarStats) -> LidarStats {
        let total_return_distr = self
            .return_distr
            .into_iter()
            .zip(other.return_distr)
            .map(|(s, o)| s + o)
            .collect::<Vec<_>>();

        let density_area = self.density_area + other.density_area;
        let average_density = if density_area > f64::EPSILON {
            (self.average_density * self.density_area + other.average_density * other.density_area)
                / density_area
        } else {
            0.
        };

        LidarStats {
            return_distr: total_return_distr,
            return_number: self.return_number.combine_stats(other.return_number),
            intensity: self.intensity.combine_stats(other.intensity),
            average_density,
            density_area,
        }
    }
}

#[derive(Debug, Clone, Copy)]
pub struct Stat {
    pub min: f32,
    pub max: f32,
    pub mean: f32,
    pub std_dev: f32,
    pub num_points: f32,
}

impl Stat {
    pub fn combine_stats(self, other: Stat) -> Stat {
        let self_points = f64::from(self.num_points);
        let other_points = f64::from(other.num_points);
        let total_points = self_points + other_points;

        Stat {
            min: self.min.min(other.min),
            max: self.max.max(other.max),
            mean: ((f64::from(self.mean) * self_points + f64::from(other.mean) * other_points)
                / total_points) as f32,
            std_dev: (f64::from(self.std_dev).hypot(f64::from(other.std_dev))) as f32,
            num_points: total_points as f32,
        }
    }
}

impl Default for Stat {
    fn default() -> Self {
        Self {
            min: f32::MAX,
            max: f32::MIN,
            mean: 0.,
            std_dev: 0.,
            num_points: 0.,
        }
    }
}
