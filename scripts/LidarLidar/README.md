# LidarLidar Calibration
This C++ package is used to perform an extrinsic calibration of two LiDAR sensors. The lidarCal.cpp file creates a segmentation object, which
is responsible for isolating vertical clusters in the point cloud. This is used to extract the cluster of points that are associated with
the checkerboard calibration object. A list of centroids, the centroids.csv file, is updated with the centroid location of the checkerboard
in each point cloud. Once all desired point clouds have been analyzed, the list of centroids is used to generate an initial transformation
between the two LiDARs. This transformation matrix is used as the initial guess in the ICP algorithm, which is used to further refine the 
transformation matrix.

## Script Descriptions
**lidarCal.cpp**
- The main driver script.
- Toggles between segmentation mode (extract centroids from checkerboard point clouds) and calibration mode (generate transformation and run ICP).
- Loads .pcd files, applies segmentation or calibration depending on the workflow toggle

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

**CMakeLists.txt**
- A legacy CMakeLists.txt file included for convenience when compliling.  This legacy script builds all the core modules into a single executable but doesn't make use of the header files.  Compiling the .cpp files directly with your own build system is recommended.

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

## Usage and Configuration

### lidarCal.cpp
The lidarCal.cpp script has a config section that must be updated to set the desired behavior and values prior being compiled and run.  These settings are summarized in the table below.

|Parameter|Example Value|Description|
|:---:|:---:|:---|
|PERFORM_SEGMENTATION|true / false|Workflow mode toggle. true = extract checkerboard centroids into centroids.csv. false = generate & refine transformation matrix using centroids + ICP.|
|LIDAR_ID|"233" / "234"|The LiDAR sensor being processed. Must match the folder name under ../pcds/.|
|POINT_CLOUD_TIMESTAMP|"15_31_54_1707424549.5918"|Timestamp portion of the .pcd filename to process in segmentation mode. Format: HH_MM_SS_<epoch>.|
|CENTROID_CSV|"../centroids.csv"|Path to the CSV file where checkerboard centroids are stored. Used to generate the initial transformation.|
|PCD_DIR_233|"../pcds/233/"|Directory containing .pcd files for LiDAR 233. Files must follow naming convention alignedPointCloud_<timestamp>.pcd.|
|PCD_DIR_234|"../pcds/234/"|Directory containing .pcd files for LiDAR 234. Same naming convention as above.|
|ROI_233|{7.75f, 12.0f, -8.0f, -3.75f}|Region-of-interest bounds (xMin, xMax, yMin, yMax, in meters) for checkerboard segmentation on LiDAR 233.|
|ROI_234|{19.0f, 23.0f, -27.0f, -23.0f}|ROI bounds for checkerboard segmentation on LiDAR 234.|

**Notes**
**workflow toggle / Perform Segmentation**
Set to true to run segmentation mode and false if you want to run calibration mode to generate the initial transformation from the centroids and refine them with ICP.

**Lidar Setting and File Paths**
The script expects LIDAR ID to be named 233 or 234 but you can use your own as long as you update the script and follow the naming convention. `alignedPointCloud_<timestamp>.pcd`

### segmentation.cpp
The segmentation script contains additional configurations parameters that should be set based on your desired behavior.  These are as follows.

|Parameter|Example Value|Description|
|:---:|:---:|:---|
|CENTROID_CSV_PATH|"../centroids.csv"|File path where segmentation appends checkerboard centroids.|
|CLUSTER_TOLERANCE|0.6|Maximum distance (m) between neighboring points to be considered in the same cluster. Key parameter to tune when clustering fails.|
|MIN_CLUSTER_SIZE|20|Minimum number of points required to form a valid cluster. Helps filter out noise.|
|NORMAL_RADIUS|1|Search radius (m) used when estimating surface normals. Larger values smooth more, smaller values preserve detail.|
|ROI|{ -10.0, 10.0, -10.0, 10.0 } (default template)|Region-of-interest bounds for filtering point clouds. Final ROI values are set in lidarCal.cpp for each LiDAR.|

**Notes**
- The ROI bounds in segmentation.cpp are placeholders — the actual ROI is tuned in lidarCal.cpp.

- In segmentation mode, you will be prompted interactively for X/Y/Z min/max bounds to further refine the checkerboard region before saving the centroid.

- If clustering does not correctly isolate the checkerboard, try adjusting CLUSTER_TOLERANCE and MIN_CLUSTER_SIZE.  If no clusters are found or if too much noise is saved try raising MIN_CLUSTER_SIZE.

### generateTransform.cpp
Generate transform controls how centroids are read and assigned from centroids.csv.  It includes the following parameters, which assume the CSV layout is lidarID, timestamp, x, y, z. 

- LIDAR_ID_1
- LIDAR_ID_2
- CSV_ID_COLUMN
- CSV_X_COLUMN
- CSV_Y_COLUMN
- CSV_Z_COLUMN

\* *only update if you make changes to centroid layout.*

If you make changes to this layout, in segmentation for instance, you'll need to update these constants to match.

### ICP.cpp
The ICP module determines how strictly the Iterative Closest Point (ICP) algorithm aligns LiDAR scans.  The relevant parameters are listed below.  Tuning ICP_MAX_CORRESPONDENCE is generally the most important parameter if ICP fails to converge.  You should also ensure teh filenames follow this convention `alignedPointCloud_<HH_MM_SS_epoch>.pcd`

|Parameter|Type|Description|
|:---:|:---:|:---|
|ICP_MAX_ITERATIONS|int|Maximum number of ICP iterations. Higher = more accurate but slower.|
|ICP_TRANSFORMATION_EPSILON|double|Convergence threshold; ICP stops when the transformation update is smaller than this value.|
|ICP_MAX_CORRESPONDENCE|double|Maximum distance (m) for two points to be considered corresponding. Increase if scans are far apart, decrease if alignment drifts.|
|TIMESTAMP_PATTERN|regex|Regex used to extract timestamps from .pcd filenames. Must match your file naming convention.|


## Dependencies
See header files or script includes