pub enum ObjectKind{
    Point = 0,
    Path = 1,
    Text = 4,
}

pub struct Tag{
    key: String,
    value: String,
}

impl Tag{
    pub fn new(k: String, v: String){
        Tag{
            key: k,
            value: v,
        }
    }
}

pub struct Object{
    kind: ObjectKind,
    symbol: Symbol,
    coordinates: Vec<Point2D>,
    tags: Vec<Tag>,
}