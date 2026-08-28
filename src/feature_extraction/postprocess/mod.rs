use crate::feature_extraction::prediction::PredictionRaster;
use crate::geometry::MapMultiPolygon;
use crate::map_gen::egui_map::{LineSymbol, MapObject};
use crate::parameters::{MapParameters, OnnxStreamVectorizationParameters};
use crate::raster::{Dfm, ModelPrediction};
use geo::{Area, BooleanOps, Buffer, Euclidean, Length, Simplify};
use std::collections::HashMap;

pub fn stream_features(
    prediction: &PredictionRaster,
    cut_overlay: &geo::Polygon,
    parameters: &MapParameters,
) -> crate::Result<Vec<MapObject>> {
    let vectorization = &parameters.streams.onnx_vectorization;
    validate_vectorization_parameters(vectorization)?;
    let mask = winning_stream_mask(prediction, vectorization.confidence_threshold)?;
    let polygons = geo::MultiPolygon::from_contours(mask.marching_squares(0.5), cut_overlay, false);
    let polygons = buffer_prediction_polygons(polygons, vectorization.polygon_buffer_m);
    let polygons = cut_overlay.intersection(&polygons);
    let lines = stream_centerlines(polygons, vectorization);

    let minimum_length = LineSymbol::SmallCrossableWatercourse.min_length(parameters.scale, false)
        * parameters.scale.denominator()
        / 1_000_000.;
    Ok(cut_overlay
        .clip(&lines, false)
        .simplify(vectorization.simplification_tolerance_m)
        .into_iter()
        .filter(|line| line.0.len() >= 2 && Euclidean.length(line) >= minimum_length)
        .map(|object| MapObject::Line {
            object,
            symbol: LineSymbol::SmallCrossableWatercourse,
            tags: HashMap::new(),
        })
        .collect())
}

/// Reduce the model's buffered semantic labels to publication centerlines.
///
/// Training labels are a 1.5 m buffer around each reference channel (3 m
/// total width). A generic width-based polygon collapse therefore retains the
/// expected predictions as areas instead of producing lines. Every predicted
/// component is semantic line evidence, so extract its medial axis directly.
fn stream_centerlines(
    polygons: geo::MultiPolygon,
    parameters: &OnnxStreamVectorizationParameters,
) -> geo::MultiLineString {
    let lines = polygons
        .into_iter()
        .filter_map(|polygon| {
            let minimum_branch_length =
                if polygon.unsigned_area() < parameters.branch_length_exemption_area_m2 {
                    0.
                } else {
                    parameters.minimum_branch_length_m
                };
            crate::geometry::centerline::extract(
                &polygon,
                parameters.centerline_sampling_distance_m,
                minimum_branch_length,
            )
        })
        .flat_map(|centerlines| centerlines.0)
        .collect();
    geo::MultiLineString::new(lines)
}

fn buffer_prediction_polygons(polygons: geo::MultiPolygon, distance_m: f64) -> geo::MultiPolygon {
    if distance_m == 0. {
        polygons
    } else {
        polygons.buffer(distance_m)
    }
}

/// Convert the mutually exclusive model classes into one publication mask.
///
/// The reference inference implementation applies `argmax` over background,
/// ditch and stream. Because both foreground classes publish as the same map
/// symbol, a pixel is selected exactly when either foreground probability is
/// strictly greater than the background probability and reaches the configured
/// confidence threshold. Strict comparison gives background priority on ties,
/// matching NumPy's first-index `argmax`.
fn winning_stream_mask(
    prediction: &PredictionRaster,
    confidence_threshold: f32,
) -> crate::Result<Dfm<ModelPrediction>> {
    anyhow::ensure!(
        confidence_threshold.is_finite() && (0.0..=1.0).contains(&confidence_threshold),
        "ONNX stream confidence threshold must be in 0..=1"
    );
    let background = prediction
        .channels
        .first()
        .ok_or_else(|| anyhow::anyhow!("stream model has no output channel 0"))?;
    let ditch = prediction
        .channels
        .get(1)
        .ok_or_else(|| anyhow::anyhow!("stream model has no output channel 1"))?;
    let stream = prediction
        .channels
        .get(2)
        .ok_or_else(|| anyhow::anyhow!("stream model has no output channel 2"))?;
    anyhow::ensure!(
        background.name == "background" && ditch.name == "ditch" && stream.name == "stream",
        "stream model outputs must be named background, ditch and stream"
    );
    background
        .raster
        .grid
        .ensure_compatible(&ditch.raster.grid)?;
    ditch.raster.grid.ensure_compatible(&stream.raster.grid)?;

    let mut mask = Dfm::new_like(&ditch.raster);
    mask.field.fill(0.);
    for y in mask.grid.inner.top..mask.grid.inner.bottom {
        for x in mask.grid.inner.left..mask.grid.inner.right {
            let index = y * mask.width() + x;
            let background_probability = background.raster.field[index];
            let foreground_probability = ditch.raster.field[index].max(stream.raster.field[index]);
            mask.field[index] = f32::from(
                foreground_probability > background_probability
                    && foreground_probability >= confidence_threshold,
            );
        }
    }
    Ok(mask)
}

fn validate_vectorization_parameters(
    parameters: &OnnxStreamVectorizationParameters,
) -> crate::Result<()> {
    anyhow::ensure!(
        parameters.polygon_buffer_m.is_finite(),
        "ONNX stream polygon buffer must be finite"
    );
    anyhow::ensure!(
        parameters.centerline_sampling_distance_m.is_finite()
            && parameters.centerline_sampling_distance_m > 0.,
        "ONNX stream centerline sampling distance must be positive"
    );
    for (name, value) in [
        ("minimum branch length", parameters.minimum_branch_length_m),
        (
            "branch-length exemption area",
            parameters.branch_length_exemption_area_m2,
        ),
        (
            "simplification tolerance",
            parameters.simplification_tolerance_m,
        ),
        (
            "endpoint merge distance",
            parameters.endpoint_merge_distance_m,
        ),
    ] {
        anyhow::ensure!(
            value.is_finite() && value >= 0.,
            "ONNX stream {name} must be non-negative"
        );
    }
    anyhow::ensure!(
        parameters.confidence_threshold.is_finite()
            && (0.0..=1.0).contains(&parameters.confidence_threshold),
        "ONNX stream confidence threshold must be in 0..=1"
    );
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::feature_extraction::contract::{
        Activation, InputDescriptor, InvalidPolicy, ModelDescriptor, OutputDescriptor,
    };
    use crate::raster::{DfmGrid, DfmPixelBounds};
    use geo::{BoundingRect, polygon};

    static MODEL: ModelDescriptor = ModelDescriptor {
        schema_version: 1,
        contract_version: 1,
        id: "test_model",
        name: "Test model",
        revision: "1",
        onnx_sha256: "source",
        manifest_sha256: "manifest",
        input: InputDescriptor {
            width: 4,
            height: 4,
            cell_size: 1.,
            halo_cells: 1,
            channels: &["input"],
            normalization: &[],
            invalid_policy: InvalidPolicy::RejectTile,
        },
        output: OutputDescriptor {
            width: 4,
            height: 4,
            channels: &["ditch"],
            activation: Activation::Identity,
        },
    };

    static STREAM_MODEL: ModelDescriptor = ModelDescriptor {
        output: OutputDescriptor {
            width: 4,
            height: 4,
            channels: &["background", "ditch", "stream"],
            activation: Activation::Identity,
        },
        ..MODEL
    };

    #[test]
    fn highest_probability_class_wins_with_background_tie_priority() {
        let grid = DfmGrid::new(4, 4, 1., geo::coord! { x: 0., y: 3. })
            .unwrap()
            .with_inner(DfmPixelBounds {
                top: 1,
                bottom: 3,
                left: 1,
                right: 3,
            })
            .unwrap();
        let mut values = vec![0.1; 3 * 16];
        values[..16].fill(0.8);
        // Foreground sums to 0.55, but background is the winning class.
        values[5] = 0.45;
        values[16 + 5] = 0.30;
        values[32 + 5] = 0.25;
        // Ditch wins.
        values[6] = 0.35;
        values[16 + 6] = 0.40;
        values[32 + 6] = 0.25;
        // Stream wins.
        values[9] = 0.25;
        values[16 + 9] = 0.35;
        values[32 + 9] = 0.40;
        // Background ties ditch and therefore wins, like np.argmax.
        values[10] = 0.45;
        values[16 + 10] = 0.45;
        values[32 + 10] = 0.10;
        let prediction =
            PredictionRaster::from_nchw(&STREAM_MODEL, grid, [1, 3, 4, 4], values).unwrap();

        let mask = winning_stream_mask(&prediction, 0.).unwrap();

        assert_eq!(mask.field.iter().filter(|&&value| value == 1.).count(), 2);
        assert_eq!(mask[(1, 1)], 0.);
        assert_eq!(mask[(1, 2)], 1.);
        assert_eq!(mask[(2, 1)], 1.);
        assert_eq!(mask[(2, 2)], 0.);
        assert_eq!(mask[(0, 0)], 0.);

        let high_confidence_mask = winning_stream_mask(&prediction, 0.41).unwrap();
        assert_eq!(
            high_confidence_mask
                .field
                .iter()
                .filter(|&&value| value == 1.)
                .count(),
            0
        );
    }

    #[test]
    fn confidence_threshold_is_inclusive_for_the_winning_foreground_class() {
        let grid = DfmGrid::new(4, 4, 1., geo::coord! { x: 0., y: 3. })
            .unwrap()
            .with_inner(DfmPixelBounds {
                top: 1,
                bottom: 3,
                left: 1,
                right: 3,
            })
            .unwrap();
        let mut values = vec![0.1; 3 * 16];
        values[..16].fill(0.8);
        values[6] = 0.35;
        values[16 + 6] = 0.40;
        values[32 + 6] = 0.25;
        let prediction =
            PredictionRaster::from_nchw(&STREAM_MODEL, grid, [1, 3, 4, 4], values).unwrap();

        assert_eq!(winning_stream_mask(&prediction, 0.40).unwrap()[(1, 2)], 1.);
        assert_eq!(winning_stream_mask(&prediction, 0.41).unwrap()[(1, 2)], 0.);
        assert!(winning_stream_mask(&prediction, f32::NAN).is_err());
    }

    #[test]
    fn signed_polygon_buffer_grows_and_shrinks_predictions() {
        let square = polygon![
            (x: 0., y: 0.),
            (x: 10., y: 0.),
            (x: 10., y: 10.),
            (x: 0., y: 10.),
        ];
        let polygons = geo::MultiPolygon::new(vec![square]);

        let grown = buffer_prediction_polygons(polygons.clone(), 1.);
        let unchanged = buffer_prediction_polygons(polygons.clone(), 0.);
        let shrunk = buffer_prediction_polygons(polygons, -1.);

        assert!(grown.unsigned_area() > unchanged.unsigned_area());
        assert!(shrunk.unsigned_area() < unchanged.unsigned_area());
    }

    #[test]
    fn expected_three_metre_training_band_becomes_a_centerline() {
        let band = polygon![
            (x: 0., y: 0.),
            (x: 20., y: 0.),
            (x: 20., y: 3.),
            (x: 0., y: 3.),
        ];

        let lines = stream_centerlines(
            geo::MultiPolygon::new(vec![band]),
            &OnnxStreamVectorizationParameters::default(),
        );

        assert!(!lines.0.is_empty());
        let bounds = lines.bounding_rect().expect("centerline has bounds");
        assert!(
            bounds.width() > 15.,
            "centerline width was {}",
            bounds.width()
        );
    }
}
