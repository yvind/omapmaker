1. Upgrade the interpolation algorithm

    - add weights that depend on distance to interpolation point and effect on cog for the 31 other points
    if the cog is pulled towards the interpolation point the weight increases

    - if the distance between the interpolation point and the closest point is to big return nan

    - Use delauney interpolation, the ground points are dense enough so that linear interpolation might be ok

3. Adaptive thresholds for calculation of cliffs and yellow based on the average value of the surrounding area

4. Build a contour hierarchy

5. Complete the cdt module, build an interpolater between contour hierarchy and dem

6. Add water detection

    - use a flat (z = const) plane and check for density of on plane points (count fractional points if they are close)


7. Add a quick vegetation layer

    - calculate proportion of non-ground points in a radius a round the point of interest

8. Building detection

    - Large blocks of only last return points that are above the ground

    - Planar roofs? or perfectly circular?

9. Path detection

    - Based on Raffael Bienz forest track identifier

10. wetland

    - Based on 

