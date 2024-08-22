use std::fmt::{self, Display};

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
}

impl Display for Tag {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(f, "<t k=\"{}\">{}</t>", self.key, self.value)
    }
}
