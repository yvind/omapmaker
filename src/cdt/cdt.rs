#[derive(Debug, Clone)]
pub struct Cdt{
    pub vertices: Vec<Vertex>,
    pub edges: Vec<[usize; 2]>,
    pub faces: Vec<[usize; 3]>,
}

impl Cdt{
    pub fn new(contours: &Vec<Contour>, bounding_box: &Bounds) -> Cdt{
        Cdt{
            vertices: ,
            edges: ,
            faces: ,
        }
    }

    pub fn insert_vertex(&mut self, vertex: &Vertex){

    }

    pub fn insert_contour(&mut self, contour: &Contour){

    }

    pub fn query_face(&self, point: &Vertex) -> usize{

    }

    pub fn interpolate_value(&self, point: &Vertex) -> f64{
        
    }
}