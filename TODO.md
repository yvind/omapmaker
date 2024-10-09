1. Upgrade the interpolation algorithm

    - add weights that depend on distance to interpolation point and effect on cog for the 31 other points
    if the cog is pulled towards the interpolation point the weight increases

    - if the distance between the interpolation point and the closest point is to big return nan

    - Maybe just go back to delauney interpolation, the ground points are dense enough so that linear interpolation might be ok

2. Fix batch processing smooth transition between tiles

    - increase size of dfms that are caluclated for each tile to include the neighbouring tile's border points

    - cut the outlines of all geometries crossing the old tile bound before building polygons,
    if a outline crosses the tile bounds multiple times it need be cut and divided into multiple parts

    - build polygons again, hopefully the geometries are now smoother across outlines

3. Adaptive thresholds for calculation of cliffs and yellow based on the average value of the surrounding area

4. Build a contour hierarchy

5. Complete the cdt module, build an interpolater between contour hierarchy and dem

6. Add water detection

    - use a flat (z = const) plane and check for density of on plane points (count fractional points if they are close)

    - add fake ground points to the point cloud on the water surface

7. Add a quick vegetation layer

    - calculate proportion of non-ground points in a radius a round the point of interest

8. Building detection

    - Large blocks of only last return points that are above the ground

    - Planar roofs? or perfectly circular?

9. Path detection

    - Based on Raffael Bienz forest track identifier

10. wetland

    - Based on 

