A rust command line application to generate an orienteering map (.omap file) directly from a .laz or .las file

-h or --help for help

Mandatory arguments:

    -i <in_file>, path to input, accepts .las or .laz files

Optional arguments:

    -o <output_directory>, path to output directory, creates a new dir if given dir doesn't exist, defaults to current working directory

    -c <contour_interval>, contour interval in meters of output map, default value 5

    -g <grid_size>, side length in meters of cells in raster generation, default value 0.5

    -b <basemap_contours>, contour interval in meters of basemap (analytic contours), default 0 ie no basemap

    -w, passing this flag saves the raster images produced from the laz-input to Tiff-files
    
    -f, passing this flag adds formlines to the output map


To do:

    -t <threads>, number of threads to run on

    -GPU, passing this flag runs the calculations on the GPU