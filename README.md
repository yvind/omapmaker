# OmapMaker
### Generate georeferenced and magnetic north aligned .omap files directly from lidar data

An application for generating orienteering maps (.omap file) from ground-classified lidar data.

With a GUI with parameter tuning, area filtering and lidar conversion tools.
The written omap-file is automatically georeferenced.

Overlapping lidar files not yet handled

### Implemented:
- GUI with live map parameter tuning
- Raw, smoothed and interpolation-based (experimental) contours
- Basemap-contours (with marked depressions)
- Vegetation
- Writes Omap-files that are 
    - georeferenced (if a CRS is detected in the lidar files or provided)
    - aligned towards the Magnetic North according to the world magnetic model based on the maps creation date and geographical position
    - correctly scaled including calculating the auxiliary scale factor based on the map center's elevation
    - in scales 1:10_000 or 1:15_000 with minimum symbol size filtering
- Supports both bezier and polyline output
- Polygon filter for only mapping parts of the provided lidar files
- .las and .laz to .copc.laz conversion
- Coordinate system assignment tool for CRS-less lidar-files (Lantmäteriet in Sweden uses EPSG:3006, but often skips writing the __mandatory__ CRS-VLR to their files)
- CRS-less files are supported if the CRS is unknown
- Non-connected lidar file detection (Useful when accidentally adding a file that should not have been added or assigning the wrong CRS to a file)
- OpenStreetMap or OpenTopoMap background map
- Experimental lidar-intensity filter

### WIP:
- AI contours
- Form lines
- Buffering on polygons to remove small holes and too thin areas or exaggerating small details

### Wish List:
- Water detection
- Building detection
- Vegetation boundaries
- Stream detection
- Boulder detection
- Road/path detection
- Marsh detection
- Lidar CRS transformation
- Overlapping Lidar handling

### Parameter tuning in OmapMaker
Tune the map parameters on a test tile before generating the whole map

![Parameter tuning in OmapMaker](./readme_images/parameter_tuning.png)

### Adding a polygon filter to lidar files
Add a polygon filter. Only lidar points inside the filter are used in map generation

![Polygon filter in OmapMaker](./readme_images/polygon_filter.png)