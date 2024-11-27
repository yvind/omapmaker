use std::path::PathBuf;

use clap::Parser;
/// Extract an orienteering map from a ground-classified point cloud
#[derive(Parser, Clone)]
pub struct Args {
    /// Path to input, accepts .las/.laz-files or a folder containing .las/.laz-files
    #[arg(short, long, value_name = "[DIR | file.laz | file.las]")]
    pub in_file: PathBuf,

    /// Path to output directory, creates a new dir if given path doesn't exist, defaults to current working directory
    #[arg(short, long, value_name = "DIR", default_value = ".")]
    pub output_directory: PathBuf,

    /// Contour interval of map output in meters, must be at least 2m with formlines enabled else 1m
    #[arg(short, long, default_value_t = 5.)]
    pub contour_interval: f64,

    /// Compute the contours without formlines
    #[clap(short, long, action = clap::ArgAction::SetFalse)]
    pub no_form_lines: bool,

    /// Contour interval in meters of basemap (analytic contours) min value 0.1, default no basemap
    #[arg(short, long, default_value_t = 0.)]
    pub basemap_contours: f64,

    /// Simplifies the geometries in the map, min value 0.1
    #[arg(short, long, default_value_t = 0.1)]
    pub simplification_distance: f64,

    /// Write the geometries as polylines instead of bezier curves
    #[arg(long="no-bezier", action = clap::ArgAction::SetFalse)]
    pub bezier: bool,

    /// Write elevation, intensity and return_number and their gradients to uncompressed Tiff-files
    #[clap(short, long)]
    pub write_tiff: bool,

    /// Number of threads used in computation, defaults to all available threads
    #[arg(short, long, default_value_t = std::thread::available_parallelism().unwrap().get())]
    pub threads: usize,
}

impl Args {
    pub fn parse_cli() -> Args {
        let mut args = Args::parse();

        if !args.no_form_lines {
            assert!(args.contour_interval >= 2.);
        } else {
            assert!(args.contour_interval >= 1.);
        }

        args.output_directory.push("");
        args.simplification_distance = args.simplification_distance.max(0.1);

        args
    }
}
