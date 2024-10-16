use std::path::PathBuf;

use clap::Parser;
/// Extract an orienteering map from a ground-classified point cloud
#[derive(Parser, Clone)]
pub struct Args {
    /// Path to input, accepts .las/.laz-files or a folder containing .las/.laz-files
    #[arg(short, long)]
    pub in_file: PathBuf,

    /// Path to output directory, creates a new dir if given path doesn't exist, defaults to current working directory
    #[arg(short, long, default_value = ".")]
    pub output_directory: PathBuf,

    /// Contour interval in meters of map output, default 5.0
    #[arg(short, long, default_value_t = 5.)]
    pub contour_interval: f64,

    /// Grid cell size in meters for DFM generation, default 0.5
    #[arg(short, long, default_value_t = 0.5)]
    pub grid_size: f64,

    /// Contour interval in meters of basemap (analytic contours) min value 0.1, default no basemap
    #[arg(short, long, default_value_t = 0.)]
    pub basemap_contours: f64,

    /// Write elevation, intensity and return_number and their gradients to Tiff-files
    #[clap(short, long, action)]
    pub write_tiff: bool,

    /// Compute the contours with formlines, defaults to no form lines
    #[clap(short, long, action)]
    pub form_lines: bool,

    /// Number of threads used in computation, defaults to all available threads
    #[arg(short, long, default_value_t = std::thread::available_parallelism().unwrap().get())]
    pub threads: usize,

    /// Simplifies the geometries in the map, default and min value 0.1
    /// For any set of three vertices in a row the middle vertex is removed
    /// if the distance in meters to the line through the other two is less than this value
    #[arg(long, action, default_value_t = 0.1)]
    pub simplification_distance: f64,
}

impl Args {
    pub fn parse_cli() -> Args {
        let mut args = Args::parse();

        assert!(args.contour_interval >= 1.);

        if args.form_lines {
            args.contour_interval /= 2.;
        }

        args.output_directory.push("");
        args.simplification_distance = args.simplification_distance.max(0.1);

        args
    }
}
