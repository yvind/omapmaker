mod naive;
mod raw;
mod scalar_field;

pub(crate) use naive::compute_naive_contours;
pub(crate) use raw::extract_contours;
#[cfg(test)]
pub(crate) use scalar_field::produce_scalar_contour_field;
pub(crate) use scalar_field::{
    compute_scalar_field_contours_from_produced, produce_scalar_contour_field_from_fitted,
};
