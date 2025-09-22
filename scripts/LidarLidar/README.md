# LidarLidar Calibration
This C++ package is used to perform an extrinsic calibration of two LiDAR sensors. The lidarCal.cpp file creates a segmentation object, which
is responsible for isolating vertical clusters in the point cloud. This is used to extract the cluster of points that are associated with
the checkerboard calibration object. A list of centroids, the centroids.csv file, is updated with the centroid location of the checkerboard
in each point cloud. Once all desired point clouds have been analyzed, the list of centroids is used to generate an initial transformation
between the two LiDARs. This transformation matrix is used as the initial guess in the ICP algorithm, which is used to further refine the 
transformation matrix.

## Script Descriptions
**segmentation.cpp / segmentation.h**
- Implements ROI filtering, surface normal estimation, vertical plane detection, and Euclidean clustering.
- Interactive ROI selection allows users to filter out the checkerboard region.
- Appends checkerboard centroids to centroids.csv for calibration

**generateTransform.cpp / generateTransform.h**
- Reads centroids.csv and separates centroids by LiDAR ID.
- Computes the initial rigid-body transformation between LiDARs using SVD-based alignment.
- Provides helper methods for matrix construction and string parsing.


**ICP.cpp / ICP.h**
- Handles timestamp extraction and file matching between LiDAR directories.
- Runs the Iterative Closest Point (ICP) algorithm, starting from the initial guess provided by generateTransform.
- Iteratively refines the transformation across multiple timestamp-matched scans.

**lidarCal.cpp**
- The main driver script.
- Toggles between segmentation mode (extract centroids from checkerboard point clouds) and calibration mode (generate transformation and run ICP).
- Loads .pcd files, applies segmentation or calibration depending on the workflow toggle

## Data Flow

### Input
- LiDAR .pcd files ex... ../pcds/233/ and ../pcds/234/.
- Checkerboard target in the point cloud.

### Intermediate
- centroids.csv: stores centroids of checkerboard detections.

### Output
- Initial transformation (rotation + translation) printed to console.
- Refined transformation from ICP iterations.
- centroids from the segmentation script if run.

## Usage
...
### Configuration
...

## Dependencies
See header files or script includes

## Build Notes
a legacy CMakeLists.txt is included for convenience.  It helps build all the core modules into an exexutable but ignores the header files.