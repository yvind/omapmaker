**Steps:**

    - Process input folder if it contains more than one laz-file, skip to next step if only one file

        - Read bounds of all laz-files and find all neighbouring laz-files

    - For every laz-file

        - read all non-noise points and points in the boundary zone from the neighbours

        - build last return KD-tree

        - Analyze last return tree

            - find holes => water, mark all points on the plane as water class

            - find planar structures => buildings, 

            - find big non planar structures => boulders

        - build ground KD-tree

        - compute dems

            - compute dem and gradients

            - compute drm and gradients

            - compute last return dem and gradients with boulders and buildings filtered out

        - subtract dem from last return dem to get a ground roughness model

        - compute basemap contours

        - compute contours

        - compute vegetation

        - compute streams using ditch net

        - compute water map

        - use dem and water map to compute wetness model

        - compute marshes