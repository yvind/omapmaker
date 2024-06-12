pub struct Tag{
    key: String,
    value: String,
}

impl Tag{
    pub fn new(k: &str, v: &str){
        Tag{
            key: k.to_string(),
            value: v.to_string(),
        }
    }
}

pub struct PointObject{
    symbol: u32,
    coordinates: Point2D,
    rotation: f64,
    tags: Vec<Tag>,
}

pub struct LineObject{
    symbol: u32,
    coordinates: Contour,
    tags: Vec<Tag>,
}

pub struct AreaObject{
    symbol: u32,
    coordinates: Vec<Contour>,
    tags: Vec<Tag>,
}

trait Object{
    fn write_object(&self, f: &BufWriter);

    fn add_tag(&self, k: &str, v: &str);

    fn add_auto_tag(&self){
        self.tags
    }
}

impl PointObjkect{
    pub fn new
}

impl Object for PointObject{
    
}