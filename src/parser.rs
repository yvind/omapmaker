use clap::Parser;
/// Extract contours and open areas from a classified point cloud
#[derive(Parser)]
pub struct Args{

    /// Path to input, accepts .las or .laz files
    #[arg(short, long)]
    pub in_file: String,

    /// Path to output directory, creates a new dir if given path doesn't exist, defaults to current working directory
    #[arg(short, long, default_value = "./")]
    pub output_directory: String,

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

    /// Number of threads used in computation, defaults to 4
    #[arg(short, long, default_value_t = 4)]
    pub threads: usize
}