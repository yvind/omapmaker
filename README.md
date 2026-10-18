# OmapMaker
### Generate georeferenced and magnetic north aligned .omap files directly from lidar data

An application for generating orienteering maps (.omap file) from ground-classified lidar data.

With a GUI with parameter tuning, area filtering and lidar conversion tools.
The written omap-file is automatically georeferenced.

### Implemented:
- GUI with live map parameter tuning
- Preview uses the exported OMAP symbol definitions for colours, dashed contours, area patterns, water boundaries and point shapes
- Connected and overlapping lidar files are queried together, with exact duplicate returns removed
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
- OpenStreetMap, OpenTopoMap or ESRI satellite background map
- Experimental lidar-intensity filter
- Hydrologically corrected D8 stream detection, with an optional AI stream-detection in `deep-learning` builds
- Buffering on polygons to remove small holes and too thin areas or exaggerating small details

### WIP:
- AI contours
- Form lines
- Water detection
- Building detection
- Boulder detection (experimental tree-filtered, opaque non-ground surfaces; large boulders ≥2 m by default, small boulders ≥1 m with a lower height threshold)

### Wish List:
- Vegetation boundaries
- Road/path detection
- Marsh detection
- Lidar CRS transformation

## Step-by-step
### Add lidar files
Add your files, select output location and optionally adjust some advanced setting.
![Welcome screen in OmapMaker](./readme_images/welcome.png)

### Add an optional polygon filter to the files
Add a polygon filter. Only the area inside the filter is processed.
![Polygon filter in OmapMaker](./readme_images/polygon.png)

### Parameter tuning
OmapMaker let's the user adjust parameters live.

Every generation step is optional and enabled by default. Use **Enable this
generation step** in each adjustment panel to choose which features to generate.
Disabled steps are omitted from both the
preview and the exported map; their parameter settings are kept for re-enabling.

The preview renderer lives in `src/rendering`, adapted from the private
`omap-projects` renderer and its paper-geometry helpers. Its source is included
in this repository, so building OmapMaker requires no sibling checkout.
Symbol visibility and map opacity can still be adjusted over the background map;
maps without a CRS use a local planar preview.

#### Contour tuning
![Contour tuning in OmapMaker](./readme_images/contours.png)

#### Openness tuning
![Openness tuning in OmapMaker](./readme_images/yellow.png)

#### Vegetation tuning
![Vegetation tuning in OmapMaker](./readme_images/vegetation.png)

#### Cliff tuning
![Cliff tuning](./readme_images/cliffs.png)

#### Water detection
![Water detection](./readme_images/water.png)

### Writes to omap
Here is the finished map in OpenOrienteering Mapper (geo-referenced and magnetic north aligned).
The generation process is on the order of minutes, depending on point density and map size.
![Map in OOmapper](./readme_images/omapper.png)

### Writes geo-referenced geotiffs
OmapMaker writes viewer-scaled geotiffs for the rasters that are checked on the welcome screen.
![Hillshade in OOmapper](./readme_images/hillshade.png)
