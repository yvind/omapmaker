use std::path::PathBuf;

use clap::Parser;
/// Extract contours and open areas from a classified point cloud
#[derive(Parser)]
pub struct Args {
    /// Path to input, accepts .las or .laz files
    #[arg(short, long)]
    pub in_file: PathBuf,

    /// Path to output directory, creates a new dir if given path doesn't exist, defaults to current working directory
    #[arg(short, long, default_value = "./")]
    pub output_directory: PathBuf,

    /// Contour interval in meters of map output, default 5.0
    #[arg(short, long, default_value_t = 5.)]
    pub contour_interval: f64,

    /// Grid cell size in meters for DFM generation, default 0.5
    #[arg(short, long, default_value_t = 0.5)]
    pub grid_size: f64,

    /// Contour interval in meters of basemap (analytic contours), default 0 ie no basemap
    #[arg(short, long, default_value_t = 0.)]
    pub basemap_contours: f64,

    /// Write elevation, intensity and return_number and their gradients to Tiff-files
    #[clap(short, long, action)]
    pub write_tiff: bool,

    /// Compute the contours with formlines, defaults to no form lines
    #[clap(short, long, action)]
    pub form_lines: bool,

    /// Number of threads used in computation, defaults to all available threads
    #[arg(short, long, default_value_t = 0)]
    pub threads: usize,

    /// Use SIMD intrinsics, unstable but possible speed up
    #[clap(long, action)]
    pub simd: bool,

    /// Pass this flag to not simplify any geometries, makes enormous file-sizes
    #[clap(long, action)]
    pub not_simplify: bool,
}

impl Args {
    pub fn parse_cli() -> (PathBuf, PathBuf, f64, f64, f64, usize, bool, f64, bool) {
        let args = Args::parse();

        let contour_interval = if args.form_lines {
            args.contour_interval / 2.
        } else {
            args.contour_interval
        };
        let simplify_epsilon = 0.1 * (1 - args.not_simplify as u8) as f64;

        let threads = if args.threads > 0 {
            args.threads
        } else {
            std::thread::available_parallelism().unwrap().get()
        };

        assert!(contour_interval >= 1.);

        (
            args.in_file,
            args.output_directory,
            contour_interval,
            args.grid_size,
            args.basemap_contours,
            threads,
            args.simd,
            simplify_epsilon,
            args.write_tiff,
        )
    }
}
