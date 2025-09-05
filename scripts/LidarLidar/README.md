This C++ package is used to perform an extrinsic calibration of two LiDAR sensors. The lidarCal.cpp file creates a segmentation object, which
is responsible for isolating vertical clusters in the point cloud. This is used to extract the cluster of points that are associated with
the checkerboard calibration object. A list of centroids, the centroids.csv file, is updated with the centroid location of the checkerboard
in each point cloud. Once all desired point clouds have been analyzed, the list of centroids is used to generate an initial transformation
between the two LiDARs. This transformation matrix is used as the initial guess in the ICP algorithm, which is used to further refine the 
transformation matrix.
