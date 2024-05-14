#![allow(dead_code)]

use strum::IntoEnumIterator;
use strum_macros::EnumIter;

// the enum is ordered after the ISOM-defined color order
#[derive(EnumIter, PartialOrd, Ord, PartialEq, Eq)]
pub enum ColorKind{
    PurpleUpperCourseOverprint,
    WhiteRailway,
    Black100,
    BluePoint100,
    BrownPoint100,
    GreenPoint100,
    BlueLine100,
    DarkGreenLine100,
    BrownLine100,
    PurpleLowerCourseOverprint,
    BrownRoad50,
    BlackRoad100,
    Black50,
    Black20,
    BlueArea100,
    Blue70,
    Blue50,
    WhiteOverGreenBrown,
    BrownArea50,
    Olive,
    Green100,
    Green60,
    Green30,
    Black30,
    WhiteOverYellow,
    BlackSandField100,
    Yellow100,
    Yellow75,
    Yellow50,
}

impl ColorKind{
    pub fn get_color(&self) -> Color {
        match &self { 
            ColorKind::PurpleUpperCourseOverprint   => Color{ name: String::from("Upper purple for course overprint"),     colors: vec![SpotColorKind::Purple],                       strengths: vec![1.0],       knock_out: true },
            ColorKind::WhiteRailway                 => Color{ name: String::from("White for railroad"),                    colors: vec![SpotColorKind::Black],                        strengths: vec![0.0],       knock_out: true },
            ColorKind::Black100                     => Color{ name: String::from("Black 100%"),                            colors: vec![SpotColorKind::Black],                        strengths: vec![1.0],       knock_out: false },
            ColorKind::BluePoint100                 => Color{ name: String::from("Blue 100% for point symbols"),           colors: vec![SpotColorKind::Blue],                         strengths: vec![1.0],       knock_out: true },
            ColorKind::BrownPoint100                => Color{ name: String::from("Brown 100% for point symbols"),          colors: vec![SpotColorKind::Brown],                        strengths: vec![1.0],       knock_out: true },
            ColorKind::GreenPoint100                => Color{ name: String::from("Green 100% for point symbols"),          colors: vec![SpotColorKind::Green],                        strengths: vec![1.0],       knock_out: true },
            ColorKind::BlueLine100                  => Color{ name: String::from("Blue 100% for line symbols"),            colors: vec![SpotColorKind::Blue],                         strengths: vec![1.0],       knock_out: true },
            ColorKind::DarkGreenLine100             => Color{ name: String::from("Dark green for line symbols"),           colors: vec![SpotColorKind::DarkGreen],                    strengths: vec![1.0],       knock_out: true },
            ColorKind::BrownLine100                 => Color{ name: String::from("Brown 100% for line symbols"),           colors: vec![SpotColorKind::Brown],                        strengths: vec![1.0],       knock_out: false },
            ColorKind::PurpleLowerCourseOverprint   => Color{ name: String::from("Lower purple for course overprint"),     colors: vec![SpotColorKind::Purple],                       strengths: vec![1.0],       knock_out: false },
            ColorKind::BrownRoad50                  => Color{ name: String::from("Brown 50% for road infill"),             colors: vec![SpotColorKind::Brown],                        strengths: vec![0.5],       knock_out: true },
            ColorKind::BlackRoad100                 => Color{ name: String::from("Black 100% for road outline"),           colors: vec![SpotColorKind::Black],                        strengths: vec![1.0],       knock_out: false },
            ColorKind::Black50                      => Color{ name: String::from("Black 50%"),                             colors: vec![SpotColorKind::Black],                        strengths: vec![0.5],       knock_out: true },
            ColorKind::Black20                      => Color{ name: String::from("Black 20%"),                             colors: vec![SpotColorKind::Black],                        strengths: vec![0.2],       knock_out: true },
            ColorKind::BlueArea100                  => Color{ name: String::from("Blue 100% for area symbols"),            colors: vec![SpotColorKind::Blue],                         strengths: vec![1.0],       knock_out: true },
            ColorKind::Blue70                       => Color{ name: String::from("Blue 70%"),                              colors: vec![SpotColorKind::Blue],                         strengths: vec![0.7],       knock_out: true },
            ColorKind::Blue50                       => Color{ name: String::from("Blue 50%"),                              colors: vec![SpotColorKind::Blue],                         strengths: vec![0.5],       knock_out: true },
            ColorKind::WhiteOverGreenBrown          => Color{ name: String::from("White over green and brown"),            colors: vec![SpotColorKind::Black],                        strengths: vec![0.0],       knock_out: true },
            ColorKind::BrownArea50                  => Color{ name: String::from("Brown 50% for area symbols"),            colors: vec![SpotColorKind::Brown],                        strengths: vec![0.5],       knock_out: true },
            ColorKind::Olive                        => Color{ name: String::from("Yellow 100%/Green 50%"),                 colors: vec![SpotColorKind::Yellow, SpotColorKind::Green], strengths: vec![1.0, 0.5],  knock_out: true },
            ColorKind::Green100                     => Color{ name: String::from("Green 100% for area symbols"),           colors: vec![SpotColorKind::Green],                        strengths: vec![1.0],       knock_out: true },
            ColorKind::Green60                      => Color{ name: String::from("Green 60%"),                             colors: vec![SpotColorKind::Green],                        strengths: vec![0.6],       knock_out: true },
            ColorKind::Green30                      => Color{ name: String::from("Green 30%"),                             colors: vec![SpotColorKind::Green],                        strengths: vec![0.3],       knock_out: true },
            ColorKind::Black30                      => Color{ name: String::from("Black 30%"),                             colors: vec![SpotColorKind::Black],                        strengths: vec![0.3],       knock_out: true },
            ColorKind::WhiteOverYellow              => Color{ name: String::from("White over yellow"),                     colors: vec![SpotColorKind::Black],                        strengths: vec![0.0],       knock_out: true },
            ColorKind::BlackSandField100            => Color{ name: String::from("Black for sandy and cultivated land"),   colors: vec![SpotColorKind::Black],                        strengths: vec![1.0],       knock_out: false },
            ColorKind::Yellow100                    => Color{ name: String::from("Yellow 100%"),                           colors: vec![SpotColorKind::Yellow],                       strengths: vec![1.0],       knock_out: true },
            ColorKind::Yellow75                     => Color{ name: String::from("Yellow 75%"),                            colors: vec![SpotColorKind::Yellow],                       strengths: vec![0.75],      knock_out: true },
            ColorKind::Yellow50                     => Color{ name: String::from("Yellow 50%"),                            colors: vec![SpotColorKind::Yellow],                       strengths: vec![0.5],       knock_out: true },
        }
    }
}

#[derive(EnumIter)]
pub enum SpotColorKind{
    Black,
    Blue,
    Yellow,
    Green,
    DarkGreen,
    Brown,
    Purple,
}

impl SpotColorKind {
    pub fn get_spotcolor(&self) -> SpotColor{
        match &self {
            SpotColorKind::Black        => SpotColor{ c: 0.0,    m: 0.0,     y: 0.0,     k: 1.0,    name: String::from("BLACK") },
            SpotColorKind::Blue         => SpotColor{ c: 1.0,    m: 0.0,     y: 0.0,     k: 0.0,    name: String::from("BLUE") },
            SpotColorKind::Yellow       => SpotColor{ c: 0.0,    m: 0.27,    y: 0.79,    k: 0.0,    name: String::from("YELLOW") },
            SpotColorKind::Green        => SpotColor{ c: 0.76,   m: 0.0,     y: 0.91,    k: 0.0,    name: String::from("GREEN") },
            SpotColorKind::DarkGreen    => SpotColor{ c: 1.0,    m: 0.0,     y: 0.8,     k: 0.3,    name: String::from("DARKGREEN") },
            SpotColorKind::Brown        => SpotColor{ c: 0.0,    m: 0.56,    y: 1.0,     k: 0.18,   name: String::from("BROWN") },
            SpotColorKind::Purple       => SpotColor{ c: 0.35,   m: 0.85,    y: 0.0,     k: 0.0,    name: String::from("PURPLE") },
        }
    }
}

pub struct SpotColor{
    pub name: String,
    pub c: f64,
    pub m: f64,
    pub y: f64,
    pub k: f64,
}

impl SpotColor {
    pub fn cmyk2rgb(c: f64, m: f64, y: f64, k: f64) -> (f64, f64, f64) {
        let r = (1.0 - c) * (1.0 - k);
        let g = (1.0 - m) * (1.0 - k);
        let b = (1.0 - y) * (1.0 - k);
        return (r, g, b);
    }
}

pub struct Color{
    pub name: String,
    pub knock_out: bool,
    pub colors: Vec<SpotColorKind>,
    pub strengths: Vec<f64>,
}