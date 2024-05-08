use crate::geometry::vertex::Vertex;

use std::hash::{Hash, Hasher};
use las::Bounds;

#[derive(Clone, Debug)]
pub struct Contour{
    pub elevation: f64,
    pub vertices: Vec<Vertex>,
    pub id: usize,
}

impl Contour{
    pub fn new(elevation: f64, vert1: Vertex, vert2: Vertex, count: usize) -> Contour{
        Contour{
            elevation,
            vertices: vec![vert1, vert2],
            id: count,
        }
    }

    pub fn is_closed(&self) -> bool{
        return self.first_vertex == self.last_vertex;
    }

    pub fn push(&mut self, vert: Vertex){
        self.vertices.push(vert);
    }

    pub fn close(&mut self){
        self.vertices.push(self.first_vertex());
    }

    pub fn close_by_boundary(&mut self, bounding_box: &Bounds){
        let first_vertex = self.first_vertex();
        let last_vertex = self.last_vertex();

        let first_vertex_index = first_vertex.get_box_edge_index(bounding_box).unwrap();
        let last_vertex_index = last_vertex.get_box_edge_index(bounding_box).unwrap();

        if first_vertex_index == 3{ // xmin
            if last_vertex_index == 0{ // -> ymax
                let v1: Vertex = Vertex{x: bounding_box.min.x, y: bounding_box.max.y, id: 0};
                self.append(v1);
            }
            else if last_vertex_index == 1{ // -> xmax
                let v1: Vertex = Vertex{x: bounding_box.max.x, y: bounding_box.max.y, id: 0};
                let v2: Vertex = Vertex{x: bounding_box.min.x, y: bounding_box.max.y, id: 0};
                self.append(v1);
                self.append(v2);
            }
            else if last_vertex_index == 2{ // -> ymin
                let v1: Vertex = Vertex{x: bounding_box.max.x, y: bounding_box.min.y, id: 0};
                let v2: Vertex = Vertex{x: bounding_box.max.x, y: bounding_box.max.y, id: 0};
                let v3: Vertex = Vertex{x: bounding_box.min.x, y: bounding_box.max.y, id: 0};
                self.append(v1);
                self.append(v2);
                self.append(v3);
            }
            else if last_vertex_index == 3{ // -> xmin
                if first_vertex.y > last_vertex.y{
                    let v1 = Vertex{x: bounding_box.min.x, y: bounding_box.min.y, id: 0};
                    let v2 = Vertex{x: bounding_box.max.x, y: bounding_box.min.y, id: 0};
                    let v3 = Vertex{x: bounding_box.max.x, y: bounding_box.max.y, id: 0};
                    let v4 = Vertex{x: bounding_box.min.x, y: bounding_box.max.y, id: 0};
                    self.append(v1);
                    self.append(v2);
                    self.append(v3);
                    self.append(v4);
                }
            }
        }
        else if first_vertex_index == 0{ // ymax
            if last_vertex_index == 1{ // -> xmax
                let v1: Vertex = Vertex{x: bounding_box.max.x, y: bounding_box.max.y, id: 0};
                self.append(v1);
            }
            else if last_vertex_index == 2{ // -> ymin
                let v1: Vertex = Vertex{x: bounding_box.max.x, y: bounding_box.min.y, id: 0};
                let v2: Vertex = Vertex{x: bounding_box.max.x, y: bounding_box.max.y, id: 0};
                self.append(v1);
                self.append(v2);
            }
            else if last_vertex_index == 3{ // -> xmin
                let v1: Vertex = Vertex{x: bounding_box.min.x, y: bounding_box.min.y, id: 0};
                let v2: Vertex = Vertex{x: bounding_box.max.x, y: bounding_box.min.y, id: 0};
                let v3: Vertex = Vertex{x: bounding_box.max.x, y: bounding_box.max.y, id: 0};
                self.append(v1);
                self.append(v2);
                self.append(v3);
            }
            else if last_vertex_index == 0{ // -> ymax
                if first_vertex.x > last_vertex.x{
                    let v1 = Vertex{x: bounding_box.min.x, y: bounding_box.max.y, id: 0};
                    let v2 = Vertex{x: bounding_box.min.x, y: bounding_box.min.y, id: 0};
                    let v3 = Vertex{x: bounding_box.max.x, y: bounding_box.min.y, id: 0};
                    let v4 = Vertex{x: bounding_box.max.x, y: bounding_box.max.y, id: 0};
                    self.append(v1);
                    self.append(v2);
                    self.append(v3);
                    self.append(v4);
                }
            }
        }
        else if first_vertex_index == 1{ // xmax
            if last_vertex_index == 2{ // -> ymin
                let v1: Vertex = Vertex{x: bounding_box.max.x, y: bounding_box.min.y, id: 0};
                self.append(v1);
            }
            else if last_vertex_index == 3{ // -> xmin
                let v1: Vertex = Vertex{x: bounding_box.min.x, y: bounding_box.min.y, id: 0};
                let v2: Vertex = Vertex{x: bounding_box.max.x, y: bounding_box.min.y, id: 0};
                self.append(v1);
                self.append(v2);
            }
            else if last_vertex_index == 0{ // -> ymax
                let v1: Vertex = Vertex{x: bounding_box.min.x, y: bounding_box.max.y, id: 0};
                let v2: Vertex = Vertex{x: bounding_box.min.x, y: bounding_box.min.y, id: 0};
                let v3: Vertex = Vertex{x: bounding_box.max.x, y: bounding_box.min.y, id: 0};
                self.append(v1);
                self.append(v2);
                self.append(v3);
            }
            else if last_vertex_index == 1{ // -> xmax
                if first_vertex.y < last_vertex.y{
                    let v1 = Vertex{x: bounding_box.max.x, y: bounding_box.max.y, id: 0};
                    let v2 = Vertex{x: bounding_box.min.x, y: bounding_box.max.y, id: 0};
                    let v3 = Vertex{x: bounding_box.min.x, y: bounding_box.min.y, id: 0};
                    let v4 = Vertex{x: bounding_box.max.x, y: bounding_box.min.y, id: 0};
                    self.append(v1);
                    self.append(v2);
                    self.append(v3);
                    self.append(v4);
                }
            }
        }
        else if first_vertex_index == 2{ // ymin
            if last_vertex_index == 3{ // -> xmin
                let v1: Vertex = Vertex{x: bounding_box.min.x, y: bounding_box.min.y, id: 0};
                self.append(v1);
            }
            else if last_vertex_index == 0{ // -> ymax
                let v1: Vertex = Vertex{x: bounding_box.min.x, y: bounding_box.max.y, id: 0};
                let v2: Vertex = Vertex{x: bounding_box.min.x, y: bounding_box.min.y, id: 0};
                self.append(v1);
                self.append(v2);
            }
            else if last_vertex_index == 1{ // -> xmax
                let v1: Vertex = Vertex{x: bounding_box.max.x, y: bounding_box.max.y, id: 0};
                let v2: Vertex = Vertex{x: bounding_box.min.x, y: bounding_box.max.y, id: 0};
                let v3: Vertex = Vertex{x: bounding_box.min.x, y: bounding_box.min.y, id: 0};
                self.append(v1);
                self.append(v2);
                self.append(v3);
            }
            else if last_vertex_index == 2{ // -> ymin
                if first_vertex.x < last_vertex.x{
                    let v1 = Vertex{x: bounding_box.max.x, y: bounding_box.min.y, id: 0};
                    let v2 = Vertex{x: bounding_box.max.x, y: bounding_box.max.y, id: 0};
                    let v3 = Vertex{x: bounding_box.min.x, y: bounding_box.max.y, id: 0};
                    let v4 = Vertex{x: bounding_box.min.x, y: bounding_box.min.y, id: 0};
                    self.append(v1);
                    self.append(v2);
                    self.append(v3);
                    self.append(v4);
                }
            }
        }

        self.append(self.first_vertex());
        self.is_closed = true;
    }
    
    pub fn contains(&self, point: &Point2D) -> Result<bool, &'static str>{
        if !self.is_closed(){
            return Err("Cannot compute area of unclosed contour");
        }

        let mut intersection_count = 0;
        for i in 0..self.vertices.len()-1 {
            let vertex1 = &self.vertices[i];
            let vertex2 = &self.vertices[i+1];
    
            if (point.y <= vertex2.y && point.y > vertex1.y) || (point.y >= vertex2.y && point.y < vertex1.y){
                if vertex1.y == vertex2.y{
                    continue;
                }
                else if point.x < ((point.y - vertex1.y)/(vertex2.y-vertex1.y))*(vertex2.x-vertex1.x) + vertex1.x{
                    intersection_count += 1;
                }
            }
        }
        return Ok(intersection_count % 2 != 0);
    }

    pub fn first_vertex(&self) -> Vertex{
        return self.vertices[0].clone();
    }

    pub fn append(&mut self, other: &mut Contour){
        self.vertices.append(&mut other.vertices);
    }

    pub fn append_by_boundary(&mut self, other: &mut Contour, bounding_box: &Bounds){
        let bl = Vertex{x: bounding_box.min.x, y: bounding_box.min.y, id: 0};
        let br = Vertex{x: bounding_box.max.x, y: bounding_box.min.y, id: 0};
        let tl = Vertex{x: bounding_box.min.x, y: bounding_box.max.y, id: 0};
        let tr = Vertex{x: bounding_box.max.x, y: bounding_box.max.y, id: 0};

        let dist = self.last_vertex().get_boundary_dist(&other.first_vertex(), bounding_box).unwrap();
        let edge_index1 = self.last_vertex().get_box_edge_index(bounding_box).unwrap();
        let edge_index2 = other.first_vertex().get_box_edge_index(bounding_box).unwrap();

        let side_length_y = bounding_box.max.y - bounding_box.min.y;
        let side_length_x = bounding_box.max.x - bounding_box.min.x;
    
        if edge_index1 == 0{ // ymax
            if edge_index2 == 0{ // ymax
                if dist > side_length_x{
                    self.push(tl);
                    self.push(bl);
                    self.push(br);
                    self.push(tr);
                }
            }
            else if edge_index2 == 1{ // xmax
                self.push(tl);
                self.push(bl);
                self.push(br);
            }
            else if edge_index2 == 2{ // ymin
                self.push(tl);
                self.push(bl);
            }
            else if edge_index2 == 3{ // xmin
                self.push(tl);
            }
        }
        else if edge_index1 == 1{ // xmax
            if edge_index2 == 0{
                self.push(tr);
            }
            else if edge_index2 == 1{
                if dist > side_length_y{
                    self.push(tr);
                    self.push(tl);
                    self.push(bl);
                    self.push(br);
                }
            }
            else if edge_index2 == 2{
                self.push(tr);
                self.push(tl);
                self.push(bl);
            }
            else if edge_index2 == 3{
                self.push(tr);
                self.push(tl);
            }
        }
        else if edge_index1 == 2{ // ymin
            if edge_index2 == 0{
                self.push(br);
                self.push(tr);
            }
            else if edge_index2 == 1{
                self.push(br);
            }
            else if edge_index2 == 2{
                if dist > side_length_x{
                    self.push(br);
                    self.push(tr);
                    self.push(tl);
                    self.push(bl);
                }
            }
            else if edge_index2 == 3{
                self.push(br);
                self.push(tr);
                self.push(tl);
            }
        }
        else if edge_index1 == 3{ // xmin
            if edge_index2 == 0{
                self.push(bl);
                self.push(br);
                self.push(tr);
            }
            else if edge_index2 == 1{
                self.push(bl);
                self.push(br);
            }
            else if edge_index2 == 2{
                self.push(bl);
            }
            else if edge_index2 == 3{
                if dist > side_length_y{
                    self.push(bl);
                    self.push(br);
                    self.push(tr);
                    self.push(tl);
                }
            }
        }
        else{
            panic!("Error when joining vertices by boundary!")
        }

        self.vertices.append(&mut other.vertices);
    }

    pub fn last_vertex(&self) -> Vertex{
        self.vertices[self.len()-1].clone()
    }

    pub fn len(&self) -> usize{
        self.vertices.len()
    }
    
    pub fn prepend(&mut self, vert: Vertex){
        let mut verts = vec![vert];
        verts.append(&mut self.vertices);
        self.vertices = verts;
    }

    pub fn signed_area(&self) -> Result<f64, &'static str>{
        if !self.is_closed{
            return Err("Cannot compute area of unclosed contour");
        }
        let mut area: f64 = 0.;
        for i in 0..self.len()-1{
            area += 0.5*(self.vertices[i].x*self.vertices[i+1].y - self.vertices[i].y*self.vertices[i+1].x);
        }
        return Ok(area);
    }
}

impl Eq for Contour{}

impl PartialEq for Contour{
    fn eq(&self, other: &Contour) -> bool{
        self.id == other.id
    }
}

impl Hash for Contour{
    fn hash<H: Hasher>(&self, state: &mut H){
        self.id.hash(state);
    }
}