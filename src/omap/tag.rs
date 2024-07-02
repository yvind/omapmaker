pub struct Tag {
    key: String,
    value: String,
}

impl Tag {
    pub fn new(k: &str, v: &str) -> Self {
        Tag {
            key: k.to_string(),
            value: v.to_string(),
        }
    }

    pub fn to_string(&self) -> String {
        format!("<t k=\"{}\">{}</t>", self.key, self.value)
    }
}
