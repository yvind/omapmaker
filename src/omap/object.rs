pub struct Tag {
    key: String,
    value: String,
}

impl Tag {
    pub fn new(k: &str, v: &str) {
        Tag {
            key: k.to_string(),
            value: v.to_string(),
        }
    }
}

pub struct PointObject {
    symbol: u32,
    coordinates: Point2D,
    rotation: f64,
    tags: Vec<Tag>,
}

pub struct LineObject {
    symbol: u32,
    coordinates: Contour,
    tags: Vec<Tag>,
}

pub struct AreaObject {
    symbol: u32,
    coordinates: Vec<Contour>,
    tags: Vec<Tag>,
}

pub trait MapObject {
    fn write_object(&self, f: &BufWriter);

    fn add_tag(&self, k: &str, v: &str);

    fn add_auto_tag(&self) {
        self.add_tag("generator", "laz2omap");
    }

    fn write_coords(&self);
}

impl PointObject {
    pub fn new(symbol: u32, coordinates: Point2D, rotation: f64) -> PointObject {
        PointObject {
            symbol,
            coordinates,
            rotation,
            tags: vec![],
        }
    }
}

impl MapObject for PointObject {
    fn write_object(&self, f: &Bufwriter) {
        f.write(format!("<object type=\"0\" symbol={}>",));
        if (!self.tags.is_empty()) {
            f.write("<tags>");
            for tag in self.tags {
                f.write(format!("<t k=\"{}\">{}</t>", tag.key(), tag.value()));
            }
            f.write("</tags>");
        }
        f.write(format!("<coords count=\"1\">{} {};</coords></object>\n",));
    }

    fn add_tag(&self, k: &str, v: &str) {
        self.tags.push(Tag::new(k, v));
    }
}
