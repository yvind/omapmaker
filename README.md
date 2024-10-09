A command line application to generate an orienteering map (.omap file) directly from a .laz or .las file

-h or --help for help

Mandatory arguments:

    -i <in_file>, path to input, accepts .las or .laz files or folder containing .las and/or .laz files

Optional arguments:

    -o <output_directory>, path to output directory, creates a new dir if given dir doesn't exist, defaults to current working directory

    -g <grid_size>, side length in meters of cells in raster generation, default value 0.5

    -b <basemap_contours>, contour interval in meters of basemap (analytic contours), default 0 ie no basemap, min value 0.1

    -w, passing this flag saves the raster images produced from the laz-input to Tiff-files

    -t <threads>, number of threads to run on, default value all available threads

    --simd, passing this flag enables intrinsics (unstable, but possible speed ups)

    --not_simplify, passing this flag opts to not simplify the geometries, makes huge files

To do:

    add building detection

    add water detection

    add stream detection

    add marsh detection

    add boulder detection

    -c <contour_interval>, contour interval in meters of output map, default value 5.0

    --no_formlines, passing this flag makes the map without formlines

    add road/path detection

    -bezier, passing this flag converts geometries to bezier curves, makes smaller file sizes