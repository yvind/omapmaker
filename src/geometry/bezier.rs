use super::{Coord, LineString};
use geo::Vector2DOps;

#[derive(Clone, Debug)]
pub struct BezierSegment(pub (Coord, Option<Coord>, Option<Coord>, Coord));

pub enum BezierSegmentType {
    Polyline,
    Bezier,
}

impl BezierSegment {
    pub fn line_type(&self) -> BezierSegmentType {
        match (self.0 .1, self.0 .2) {
            (None, None) => BezierSegmentType::Polyline,
            (Some(_), Some(_)) => BezierSegmentType::Bezier,
            _ => panic!("Not possible"),
        }
    }
}

impl From<[Coord; 4]> for BezierSegment {
    fn from(value: [Coord; 4]) -> Self {
        BezierSegment((value[0], Some(value[1]), Some(value[2]), value[3]))
    }
}

#[derive(Debug)]
pub struct BezierString(pub Vec<BezierSegment>);

impl BezierString {
    pub fn num_points(&self) -> usize {
        let mut num_points = 0;

        for segment in self.0.iter() {
            match segment.line_type() {
                BezierSegmentType::Polyline => num_points += 1,
                BezierSegmentType::Bezier => num_points += 3,
            }
        }
        num_points + 1
    }

    pub fn from_polyline(polyline: &LineString, error: f64) -> BezierString {
        let n_pts = polyline.0.len();
        if n_pts <= 2 || (n_pts <= 3 && polyline.is_closed()) {
            //panic!("Degenerate line");
            let mut segments = vec![];

            let mut prev_coord = polyline.0[0];
            for p in polyline.0.iter().skip(1) {
                segments.push(BezierSegment((prev_coord, None, None, *p)));
                prev_coord = *p;
            }

            return BezierString(segments);
        }

        let mut bezier_segments = vec![];

        let mut tangent_right = Self::compute_right_tangent(&polyline.0, 0);
        let mut tangent_left = Self::compute_left_tangent(&polyline.0, n_pts - 1);

        if polyline.is_closed() {
            tangent_right = (tangent_right - tangent_left).try_normalize().unwrap();
            tangent_left = -tangent_right;
        }

        // recursivly tries to fit bezier segments to the polyline
        Self::fit_cubic(
            &polyline.0,
            0,
            n_pts - 1,
            tangent_right,
            tangent_left,
            error,
            &mut bezier_segments,
        );
        BezierString(bezier_segments)
    }

    // recursive function to fit bezier curve segments to linestring
    fn fit_cubic(
        polyline: &[Coord],
        first: usize,
        last: usize,
        tangent_start: Coord,
        tangent_end: Coord,
        error: f64,
        bezier_string: &mut Vec<BezierSegment>,
    ) {
        // Handle two-point case, recursion base case
        if last - first == 1 {
            bezier_string.push(BezierSegment((polyline[first], None, None, polyline[last])));
            return;
        }

        // Parameterize points and attempt to fit curve,
        let mut ts = Self::chord_length_parameterize(polyline, first, last);
        let mut bez_curve =
            Self::generate_bezier(polyline, first, last, &ts, &tangent_start, &tangent_end);

        // Find max deviation
        let (mut max_error, mut split_point) =
            Self::compute_max_error(polyline, first, last, &bez_curve, &ts);

        if max_error < error {
            bezier_string.push(bez_curve.into());
            return;
        }

        // If error not too large, try one step of newton-rhapson
        if max_error < 2. * error {
            ts = Self::reparameterize(polyline, first, last, &ts, &bez_curve);
            bez_curve =
                Self::generate_bezier(polyline, first, last, &ts, &tangent_start, &tangent_end);
            (max_error, split_point) =
                Self::compute_max_error(polyline, first, last, &bez_curve, &ts);

            if max_error < error {
                bezier_string.push(bez_curve.into());
                return;
            }
        }

        // Fitting failed, split at the point of max error and fit each part recursively
        let tangent_center = Self::compute_center_tangent(polyline, split_point);
        let tangent_center_neg = -tangent_center;
        Self::fit_cubic(
            polyline,
            first,
            split_point,
            tangent_start,
            tangent_center_neg,
            error,
            bezier_string,
        );
        Self::fit_cubic(
            polyline,
            split_point,
            last,
            tangent_center,
            tangent_end,
            error,
            bezier_string,
        );
    }

    // fit a bezier-segment to the polyline between first and last
    // using least-squares method to find the Bezier handles for region
    // with t-values for evaluation and end point tangents given
    fn generate_bezier(
        polyline: &[Coord],
        first: usize,
        last: usize,
        ts: &[f64],
        start_tangent: &Coord,
        end_tangent: &Coord,
    ) -> [Coord; 4] {
        let n_pts = last - first + 1;
        let mut a: Vec<[Coord; 2]> = Vec::with_capacity(n_pts);

        // Compute the A's
        for &t in ts {
            a.push([*start_tangent * Self::b1(t), *end_tangent * Self::b2(t)]);
        }

        // Create the C and X matrices
        // C is symmetric 2x2, sum of indecies in 2x2 gives index in flat array
        // X is a 2x1 vector
        let mut c = [0.0, 0.0, 0.0];
        let mut x = [0.0, 0.0];

        for (i, &t) in ts.iter().enumerate() {
            c[0] += a[i][0].dot_product(a[i][0]);
            c[1] += a[i][0].dot_product(a[i][1]);
            c[2] += a[i][1].dot_product(a[i][1]);

            let tmp = polyline[first + i]
                - (polyline[first] * (Self::b0(t) + Self::b1(t))
                    + polyline[last] * (Self::b2(t) + Self::b3(t)));

            x[0] += a[i][0].dot_product(tmp);
            x[1] += a[i][1].dot_product(tmp);
        }

        // Compute the determinants
        let det_c0_c1 = c[0] * c[2] - c[1] * c[1];
        let det_c0_x = c[0] * x[1] - c[1] * x[0];
        let det_x_c1 = x[0] * c[2] - x[1] * c[1];

        // Derive alpha values
        let alpha_l = if det_c0_c1 == 0.0 {
            0.0
        } else {
            det_x_c1 / det_c0_c1
        };
        let alpha_r = if det_c0_c1 == 0.0 {
            0.0
        } else {
            det_c0_x / det_c0_c1
        };

        // If alpha negative, use the Wu/Barsky heuristic
        let seg_length = (polyline[last] - polyline[first]).magnitude();
        let epsilon = 1.0e-6 * seg_length;

        if alpha_l < epsilon || alpha_r < epsilon {
            let dist = seg_length / 3.0;
            return [
                polyline[first],
                polyline[first] + *start_tangent * dist,
                polyline[last] + *end_tangent * dist,
                polyline[last],
            ];
        }

        [
            polyline[first],
            polyline[first] + *start_tangent * alpha_l,
            polyline[last] + *end_tangent * alpha_r,
            polyline[last],
        ]
    }

    // Bezier basis functions
    #[inline]
    fn b0(t: f64) -> f64 {
        (1.0 - t).powi(3)
    }

    #[inline]
    fn b1(t: f64) -> f64 {
        3.0 * t * (1.0 - t).powi(2)
    }

    #[inline]
    fn b2(t: f64) -> f64 {
        3.0 * t.powi(2) * (1.0 - t)
    }

    #[inline]
    fn b3(t: f64) -> f64 {
        t.powi(3)
    }

    // Vertex tangent functions
    #[inline]
    fn compute_right_tangent(polyline: &[Coord], end: usize) -> Coord {
        (polyline[end + 1] - polyline[end]).try_normalize().unwrap()
    }

    #[inline]
    fn compute_left_tangent(polyline: &[Coord], end: usize) -> Coord {
        (polyline[end - 1] - polyline[end]).try_normalize().unwrap()
    }

    #[inline]
    fn compute_center_tangent(polyline: &[Coord], center: usize) -> Coord {
        (polyline[center + 1] - polyline[center - 1])
            .try_normalize()
            .unwrap()
    }

    // normalized length along linestring from start of segment to every vertex in segment
    // the t values to be used for computing error of fitted bezier
    fn chord_length_parameterize(polyline: &[Coord], first: usize, last: usize) -> Vec<f64> {
        let mut ts = Vec::with_capacity(last - first + 1);

        ts.push(0.);
        for i in (first + 1)..=last {
            ts.push(ts[i - first - 1] + (polyline[i] - polyline[i - 1]).magnitude());
        }

        let t_last = ts[last - first];
        for t in ts.iter_mut() {
            *t /= t_last;
        }
        ts
    }

    // given a set of points and their parameterization on the bez curve
    // use newton rhapson to try and refine the parameterization
    fn reparameterize(
        polyline: &[Coord],
        first: usize,
        last: usize,
        ts: &[f64],
        bez_curve: &[Coord],
    ) -> Vec<f64> {
        let mut new_ts = vec![0.; last - first + 1];

        for i in first..=last {
            new_ts[i - first] = Self::newton_raphson(bez_curve, polyline[i], ts[i - first]);
        }
        new_ts
    }

    // bez_curve Q(u) at time t is supposed to be p, refine t
    fn newton_raphson(bez_curve: &[Coord], p: Coord, t: f64) -> f64 {
        // Q(t)
        let bez_t = Self::evaluate_bezier(3, bez_curve, t);

        // Cubic bez prime is quadratic
        let mut bez_prime = [Coord::default(), Coord::default(), Coord::default()];
        // Cubic bez double prime is linear
        let mut bez_double_prime = [Coord::default(), Coord::default()];

        // Generate control vertices for Q'
        for i in 0..3 {
            bez_prime[i].x = (bez_curve[i + 1].x - bez_curve[i].x) * 3.0;
            bez_prime[i].y = (bez_curve[i + 1].y - bez_curve[i].y) * 3.0;
        }

        // Generate control vertices for Q''
        for i in 0..2 {
            bez_double_prime[i].x = (bez_prime[i + 1].x - bez_prime[i].x) * 2.0;
            bez_double_prime[i].y = (bez_prime[i + 1].y - bez_prime[i].y) * 2.0;
        }

        // Compute Q'(t) and Q''(t)
        let qp_t = Self::evaluate_bezier(2, &bez_prime, t);
        let qpp_t = Self::evaluate_bezier(1, &bez_double_prime, t);

        // f(t) is Q linearized around the root Q(t) - p == 0

        // Compute f'(t)
        let denominator = (qp_t.x) * (qp_t.x)
            + (qp_t.y) * (qp_t.y)
            + (bez_t.x - p.x) * (qpp_t.x)
            + (bez_t.y - p.y) * (qpp_t.y);
        if denominator == 0.0 {
            return t;
        }
        // Compute f(t)
        let numerator = (bez_t.x - p.x) * (qp_t.x) + (bez_t.y - p.y) * (qp_t.y);

        // t = t - f(t)/f'(t)
        t - (numerator / denominator)
    }

    // max of distance between polyline points and the fitted curve
    // checks even the halfway point of every line segment
    fn compute_max_error(
        polyline: &[Coord],
        first: usize,
        last: usize,
        bez_curve: &[Coord],
        ts: &[f64],
    ) -> (f64, usize) {
        let mut split_point = (last + first + 1) / 2;
        let mut max_dist = 0.0;
        for i in first..last {
            let p = Self::evaluate_bezier(3, bez_curve, ts[i - first]);
            let dist = (p - polyline[i]).magnitude_squared();

            if dist >= max_dist {
                max_dist = dist;
                split_point = i;
            }

            let p = Self::evaluate_bezier(3, bez_curve, (ts[i - first] + ts[i - first + 1]) / 2.);
            let dist = (p - (polyline[i] + polyline[i + 1]) / 2.).magnitude_squared();
            if dist >= max_dist {
                max_dist = dist;
                split_point = i.max(first + 1); //if i == first { i + 1 } else { i };
            }
        }
        (max_dist, split_point)
    }

    // evaluate the bezier value at time t
    fn evaluate_bezier(degree: usize, bezier_segment: &[Coord], t: f64) -> Coord {
        // Create a temporary vector to store the control points
        let mut v_temp = bezier_segment[..=degree].to_vec();

        // De Casteljau algorithm, just lerp-ing between lower degree beziers
        for i in 1..=degree {
            for j in 0..=(degree - i) {
                v_temp[j] = v_temp[j] * (1. - t) + v_temp[j + 1] * t;
            }
        }
        v_temp[0]
    }
}
