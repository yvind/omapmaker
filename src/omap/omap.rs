#![allow(dead_code)]

use std::io::{BufWriter, Write};
use std::{fs, fs::File, io::BufWriter, path::Path};

pub struct Omap{
    filepath: Path,
    ref_point: &Point2D,
    colors: Vec<ColorKind>,
    symbols: Vec<SymbolKind>,
    objects: Vec<MapObject>,
}

impl Omap{
    pub fn new(&self, filename: &str, output_dir: &str, georef_point: &Point2D) -> Self{
        Omap{
            filepath: Path::new(format!("{}/{}.omap", output_dir, filename)),
            ref_point: georef_point,
            colors: vec![],
            symbols: vec![],
            objects: vec![],
        };
    }

    fn add_color(&mut self, c: ColorKind){
        if !self.colors.contains(c) {
            self.colors.push(c);
        }
    }

    fn add_symbol(&mut self, s: SymbolKind){
        if !self.symbols.contains(s) {
            self.symbols.push(s);
            let sym = s.get_symbol();

        }
    }

    pub fn add_all_symbols(&mut self){


    }

    pub fn write_to_file(&self){
        let f = File::create(&self.filepath).expect("Unable to create omap file");
        let mut f = BufWriter::new(f);

        self.write_header(&f);
        self.write_colors(&f);
        self.write_symbols(&f);
        self.write_objects(&f);
        self.write_end_of_file(&f);
    }

    fn write_header(&self, f: &BufWriter){
        f.write("<?xml version=\"1.0\" encoding=\"UTF-8\"?>\n<map xmlns=\"http://openorienteering.org/apps/mapper/xml/v2\" version=\"9\">\n<notes></notes>\n");
        f.write(format!("<georeferencing scale=\"15000\"><projected_crs id=\"Local\"><ref_point x=\"{}\" y=\"{}\"/></projected_crs></georeferencing>\n", self.ref_point.x, self.ref_point.y));
    }

    fn write_colors(&self, f: &BufWriter){
        self.colors.sort();
        f.write(format!("<colors count=\"{}\">\n", colors.len() + SpotColor::iter().len()));

        for (prio, color) in self.colors.iter().enumerate(){
            let col = color.get_color();

            let mut c = 0.;
            let mut m = 0.;
            let mut y = 0.;
            let mut k = 0.;

            for (sc, s) in zip(col.colors, col.strengths){
                let spot = sc.get_spotcolor();
                c += s*spot.c;
                m += s*spot.m;
                y += s*spot.y;
                k += s*spot.k;
            }

            c = c.min(1.0);
            m = m.min(1.0);
            y = y.min(1.0);
            k = k.min(1.0);

            let (r, g, b) = SpotColor::cmyk2rgb(c, m, y, k);

            f.write(format!("<color priority=\"{}\" name=\"{}\" c=\"{}\" m=\"{}\" y=\"{}\" k=\"{}\" opacity=\"1\">", prio, col.name, c, m, y, k));
            match col.knock_out{
                true => f.write("<spotcolors knockout=\"true\">"),
                false => f.write("<spotcolors>"),
            }
            for for (sc, s) in zip(col.colors, col.strengths){
                f.write(format!("<component factor=\"{}\" spotcolor=\"{}\"/>", s, sc as usize + self.colors.len()));
            }
            f.write(format!("<spotcolors/><cmyk method=\"spotcolor\"/><rgb method=\"spotcolor\" r=\"{}\" g=\"{}\" b=\"{}\"\n", r, g, b));
        }

        for (i, spot) in SpotColorKind::iter().enumerate(){
            let prio = i + self.colors.len();

            let sc = spot.get_spotcolor();
            let (r, g, b) = SpotColor::cmyk2rgb(sc.c, sc.m, sc.y, sc.k);

            f.write(format!("<color priority=\"{}\" name=\"SPOTCOLOR {}\" c=\"{}\" m=\"{}\" y=\"{}\" k=\"{}\" opacity=\"1\">", prio, sc.name, sc.c, sc.m, sc.y, sc.k));
            f.write(format!("<spotcolors knockout=\"true\"><namedcolor>{}</namedcolor><spotcolors/>", sc.name)),
            f.write(format!("<cmyk method=\"custom\"/><rgb method=\"cmyk\" r=\"{}\" g=\"{}\" b=\"{}\"\n", r, g, b));
        }
    }

    fn write_symbols(&self, f: &BufWriter){

    }

    fn write_objects(&self, f: &BufWriter){
        f.write(format!("<parts count=\"1\" current=\"0\">\n<part name=\"map\"><objects count=\"{}\">\n", self.objects.len()));

        for object in self.objects{

        }
    }

    fn write_end_of_file(&self, f: &BufWriter){
        f.write("<templates count=\"0\" first_front_template=\"0\">\n<defaults use_meters_per_pixel=\"true\" meters_per_pixel=\"0\" dpi=\"0\" scale=\"0\"/></templates>\n<view>\n");
        f.write("<grid color=\"#646464\" display=\"0\" alignment=\"0\" additional_rotation=\"0\" unit=\"1\" h_spacing=\"500\" v_spacing=\"500\" h_offset=\"0\" v_offset=\"0\" snapping_enabled=\"true\"/>\n");
        f.write("<map_view zoom=\"1\" position_x=\"0\" position_y=\"0\"><map opacity=\"1\" visible=\"true\"/><templates count=\"0\"/></map_view></view></barrier></map>");
    }
}