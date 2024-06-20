#[derive(Clone)]
pub struct PointCloud5D{
    pub points: Vec<Point5D>,
    pub bounds: &Bounds,
}

impl PointCloud5D{
    pub fn new(v: Vec<Point5D>, b: &Bounds) -> PointCloud5D{
        PointCloud5D{points: v, bounds: b}
    }

    pub fn to_2D_slice(&self) -> Vec<[f64; 2]>{
        self.points.iter().map(|p| [p.x, p.y]).collect()
    }

    pub fn len(&self) -> usize{
        self.points.len()
    }

    pub fn get_dfm_dimensions(&self, cell_size: f64) -> (usize, usize, Bounds){
        let dx: f64 = self.bounds.max.x - self.bounds.min.x;
        let dy: f64 = self.bounds.max.y - self.bounds.min.y;

        let width: f64 = (dx / cell_size).round() + 1.;
        let height: f64 = (dy / cell_size).round() + 1.;

        let offset_x: f64 = (dx - (width-1.)*cell_size) / 2.;
        let offset_y: f64 = (dy - (height-1.)*cell_size) / 2.;

        let dfm_bounds: Bounds = Bounds{
            min: Vector{x: self.bounds.min.x + offset_x, y: self.bounds.min.y + offset_y, z: 0.,},
            max: Vector{x: self.bounds.max.x - offset_x, y: self.bounds.max.y - offset_y, z: 0.,}
        };
        (width as usize, height as usize, dfm_bounds)
    }

    pub fn calculate_simple_convex_hull(&mut self) -> Contour {

        let point_compare_position = |a: &Point5D, b: &Point5D| -> Ordering{
            if a.y == b.y {
                a.x.partial_cmp(&b.x).unwrap()
            }
            else {
                a.y.partial_cmp(&b.y).unwrap()
            }
        }
        let most_south_west_point = self.points.iter().min_by(point_compare_position).unwrap();

        let point_compare_angle = |a: &Point5D, b: &Point5D| -> Ordering {
            let orientation = most_south_west_point.consecutive_orientation(a, b);
            if orientation < 0.0 {
                Ordering::Greater
            } else if orientation > 0.0 {
                Ordering::Less
            } else {
                let a_dist = most_south_west_point.dist_squared(a);
                let b_dist = most_south_west_point.dist_squared(b);

                b_dist.partial_cmp(&a_dist).unwrap()
            }
        };
        self.points.sort_by(point_compare_angle);

        let mut convex_hull: Contour = Contour{elevation: f64::MIN, vertices: Vec<Vertex>::new(), id: 0};

        convex_hull.push(most_south_west_point.clone());
        convex_hull.push(points[0].clone());
        let mut hull_length = 1;
        for point in points.iter().skip(1) {
            if most_south_west_point.consecutive_orientation(point, &convex_hull[hull_length]) == 0.0 {
                // Remove consecutive points with the same angle. We make sure include the furthest point in the convex hull in the sort comparator.
                continue;
            }
            while (hull_length > 1) {
                // If segment(i+1, i+2) turns right relative to segment(i, i+1), point(i+1) is not part of the convex hull.
                let orientation = convex_hull[hull_length - 1].consecutive_orientation(&convex_hull[hull_length], point);
                if orientation <= 0.0 {
                    hull_length -= 1;
                    convex_hull.pop();
                } else {
                    break;
                }
            }
            convex_hull.push(point.clone());
            hull_length += 1;
        }

        for mut point in hull.vertices{
            if point.x - cell_size <= self.bounds.min.x{
                point.x = self.bounds.min.x;
            }
            else if point.x + cell_size >= self.bounds.max.x{
                point.x = self.bounds.max.x;
            }
            if point.y - cell_size <= self.bounds.min.y{
                point.y = self.bounds.min.y;
            }
            else if point.y + cell_size >= self.bounds.max.y{
                point.y = self.bounds.max.y;
            }
        }
        convex_hull.close();
        convex_hull
    }

    pub fn interpolate_field(&self, field: FieldType, neighbours: &Vec<usize>, point: &[f64; 2], smoothing: f64) -> (f64, f64){
        let nrows = neighbours.len();

        let mut mean: [f64; 3] = [0., 0., 0.];
        for n in neighbours{
            mean[0] += self.points[*n].x;
            mean[1] += self.points[*n].y;

            match field{
                FieldType::Elevation => mean[2] += self.points[*n].z,
                FieldType::ReturnNumber => mean[2] += self.points[*n].r,
                FieldType::Intensity => mean[2] += self.points[*n].i,
            }
        }
        mean = [mean[0]/nrows as f64, mean[1]/nrows as f64, mean[2]/nrows as f64];
        
        let mut std: [f64; 3] = [0., 0., 0.];
        for n in neighbours{
            std[0] += (self.points[*n].x - mean[0]).powi(2);
            std[1] += (self.points[*n].y - mean[1]).powi(2);

            match field{
                FieldType::Elevation => std[2] += (self.points[*n].z - mean[2]).powi(2);,
                FieldType::ReturnNumber => std[2] += (self.points[*n].r - mean[2]).powi(2);,
                FieldType::Intensity => std[2] += (self.points[*n].i - mean[2]).powi(2);,
            }
        }
        std = [(std[0]/nrows as f64).sqrt(), (std[1]/nrows as f64).sqrt(), (std[2]/nrows as f64).sqrt()];

        if std[2] < 0.01{
            return (mean[2], 0.0);
        }
        
        let mut xy: Matrix32x6 = Matrix32x6::zeros();
        let mut z: Vector32 = Vector32::zeros();
        for (i, n) in neighbours.iter().enumerate(){
            let x = (self.points[*n].x-mean[0])/std[0];
            let y = (self.points[*n].y-mean[1])/std[1];

            xy.insert_row([1.0, x, y, x*x, y*y, x*y], i);

            match field{
                FieldType::Elevation => z.data[i] = (self.points[*n].z - mean[2])/std[2],
                FieldType::ReturnNumber => z.data[i] = (self.points[*n].r - mean[2])/std[2],
                FieldType::Intensity => z.data[i] = (self.points[*n].i - mean[2])/std[2],
            }
        }

        // slow matrix inversion
        let beta: Vector6 = (xy.tdot_self().add_to_diag(smoothing)).inverse_spd().dot_vec(xy.tdot_vec(z));
        
        let nx = (point[0] - mean[0])/std[0];
        let ny = (point[1] - mean[1])/std[1];

        let x0: Vector6 = Vector6::new([1.0, nx, ny, nx*nx, ny*ny, nx*ny]);
        let dx: Vector6 = Vector6::new([0.0, 1.0, 0.0, 2.0*nx, 0.0, ny]);
        let dy: Vector6 = Vector6::new([0.0, 0.0, 1.0, 0.0, 2.0*ny, nx]);

        let value: f64 = x0.dot(&beta);
        let gradient_x: f64 = dx.dot(&beta)*std[2]/std[0];
        let gradient_y: f64 = dy.dot(&beta)*std[2]/std[1];
        let gradient_size: f64 = (gradient_x.powi(2) + gradient_y.powi(2)).sqrt();

        (value*std[2] + mean[2], gradient_size)
    }
}