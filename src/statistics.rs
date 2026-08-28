use las::CopcReader;
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
        let mut reader = CopcReader::from_path(path)?;
        let header = reader.header();
        let num_points = header.number_of_points();
        anyhow::ensure!(
            num_points > 0,
            "Cannot calculate statistics for an empty lidar file"
        );
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
            num_points,
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

        let mut intensities = Vec::with_capacity(STATS_SAMPLE_SIZE);
        let mut intensity_stat = Stat::default();

        let mut intensity_sum = 0_f64;
        let mut num_taken_points = 0_u64;
        for point in reader
            .query(
                las::LodSelection::LevelMinMax(0, 2),
                las::BoundsSelection::All,
            )?
            .points()
            .filter_map(Result::ok)
            .take(STATS_SAMPLE_SIZE)
        {
            let i = f64::from(point.intensity);
            intensities.push(i);
            if i < f64::from(intensity_stat.min) {
                intensity_stat.min = i as f32;
            }
            if i > f64::from(intensity_stat.max) {
                intensity_stat.max = i as f32;
            }
            intensity_sum += i;
            num_taken_points += 1;
        }
        anyhow::ensure!(
            num_taken_points > 0,
            "Cannot calculate intensity statistics without readable finite points"
        );
        intensity_stat.num_points = num_taken_points;
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
    pub num_points: u64,
}

impl Stat {
    pub fn normalize(self, value: f32) -> f32 {
        if !value.is_finite() {
            return value;
        }
        let range = self.max - self.min;
        if !range.is_finite() || range <= f32::EPSILON {
            return 0.;
        }
        ((value - self.min) / range).clamp(0., 1.)
    }

    pub fn combine_stats(self, other: Stat) -> Stat {
        if self.num_points == 0 {
            return other;
        }
        if other.num_points == 0 {
            return self;
        }

        let self_points = self.num_points as f64;
        let other_points = other.num_points as f64;
        let total_points = self_points + other_points;
        let self_mean = f64::from(self.mean);
        let other_mean = f64::from(other.mean);
        let mean = (self_mean * self_points + other_mean * other_points) / total_points;
        let mean_delta = other_mean - self_mean;
        let second_moment = f64::from(self.std_dev).powi(2) * self_points
            + f64::from(other.std_dev).powi(2) * other_points
            + mean_delta.powi(2) * self_points * other_points / total_points;

        Stat {
            min: self.min.min(other.min),
            max: self.max.max(other.max),
            mean: mean as f32,
            std_dev: (second_moment / total_points).sqrt() as f32,
            num_points: self.num_points + other.num_points,
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
            num_points: 0,
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn normalization_uses_the_full_range() {
        let stat = Stat {
            min: 10.,
            max: 30.,
            ..Default::default()
        };
        assert_eq!(stat.normalize(10.), 0.);
        assert_eq!(stat.normalize(20.), 0.5);
        assert_eq!(stat.normalize(30.), 1.);
        assert_eq!(stat.normalize(40.), 1.);
    }

    #[test]
    fn constant_samples_have_a_stable_normalization() {
        let stat = Stat {
            min: 4.,
            max: 4.,
            ..Default::default()
        };
        assert_eq!(stat.normalize(4.), 0.);
        assert!(stat.normalize(f32::NAN).is_nan());
    }

    #[test]
    fn variance_merge_matches_the_combined_population() {
        let left = Stat {
            min: 1.,
            max: 2.,
            mean: 1.5,
            std_dev: 0.5,
            num_points: 2,
        };
        let right = Stat {
            min: 3.,
            max: 4.,
            mean: 3.5,
            std_dev: 0.5,
            num_points: 2,
        };
        let merged = left.combine_stats(right);
        assert_eq!(merged.num_points, 4);
        assert_eq!(merged.mean, 2.5);
        assert!((merged.std_dev - 1.25_f32.sqrt()).abs() < 1e-6);
    }

    #[test]
    fn empty_statistics_are_identity_for_merging() {
        let populated = Stat {
            min: 2.,
            max: 6.,
            mean: 4.,
            std_dev: 2.,
            num_points: 3,
        };
        assert_eq!(Stat::default().combine_stats(populated).num_points, 3);
        assert_eq!(populated.combine_stats(Stat::default()).mean, 4.);
    }
}
