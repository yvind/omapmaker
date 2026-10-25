/// Disjoint height bands above the same bilinearly sampled DEM. Lower
/// boundaries are included; upper boundaries belong to the next band.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub(in crate::generation::features) enum HeightBand {
    Ground,
    Undergrowth,
    Low,
    Medium,
    High,
}

pub(in crate::generation::features) fn height_band(height: f32) -> Option<HeightBand> {
    if !height.is_finite() {
        None
    } else if height < 0.2 {
        Some(HeightBand::Ground)
    } else if height < 0.8 {
        Some(HeightBand::Undergrowth)
    } else if height < 1.8 {
        Some(HeightBand::Low)
    } else if height < 3. {
        Some(HeightBand::Medium)
    } else {
        Some(HeightBand::High)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn boundaries_belong_to_exactly_one_band() {
        for (height, expected) in [
            (0.199, HeightBand::Ground),
            (0.2, HeightBand::Undergrowth),
            (0.799, HeightBand::Undergrowth),
            (0.8, HeightBand::Low),
            (1.499, HeightBand::Low),
            (1.5, HeightBand::Low),
            (1.799, HeightBand::Low),
            (1.8, HeightBand::Medium),
            (2.999, HeightBand::Medium),
            (3., HeightBand::High),
        ] {
            assert_eq!(height_band(height), Some(expected));
        }
        assert_eq!(height_band(f32::NAN), None);
        assert_eq!(height_band(f32::INFINITY), None);
    }
}
