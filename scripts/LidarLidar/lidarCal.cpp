#include "segmentation.h"
#include "generateTransform.h"
#include "ICP.h"

#include <iostream>
#include <string>
#include <utility>
#include <vector>
#include <algorithm>

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

#include <Eigen/Dense>
#include <Eigen/Geometry>

// ================= Configuration Parameters =================

// Workflow toggle

//Boolean for either performing segmentation or generating transformation matrices
constexpr bool PERFORM_SEGMENTATION = false;

// LiDAR settings
//
// Set this member variable to the LiDAR being worked with (233 or 234)
// should match the IDs in the directory under PCD data folders
const std::string LIDAR_ID = "234";
// The timestamp in the point cloud filename    
const std::string POINT_CLOUD_TIMESTAMP = "15_31_54_1707424549.5918";

// File paths
//
// Get the checkerboard centroids CSV file
const std::string CENTROID_CSV = "../centroids.csv";
// set the directories for .pcd files for each LIDAR
// assumes the following convention ../pcds/<your_dir>/alignedPointCloud_<timestamp>.pcd
const std::string PCD_DIR_233 = "../pcds/233/";
const std::string PCD_DIR_234 = "../pcds/234/";

// Region-of-interest bounds for checkerboard segmentation
//
// Each LIDAR may need different bounds
// bounds are min/max, units are in meters
struct ROI {
    float xMin;
    float xMax;
    float yMin;
    float yMax;
};

// ROI bounds tuned for LiDAR 233
// numbers declared as floats
const ROI ROI_233 = {7.75f, 12.0f, -8.0f, -3.75f};
// ROI bounds tuned for LiDAR 234
const ROI ROI_234 = {19.0f, 23.0f, -27.0f, -23.0f};

// ============================================================

int main()
{ 
    //Boolean for either performing segmentation or generating transformation matrices
    if (PERFORM_SEGMENTATION) {
        segmentation segmentationObject;
        // the variable for the LiDAR being worked with (233 or 234)
        segmentationObject.currentLidar = LIDAR_ID;

        // Load the desired point cloud file and timestamp in the filename
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
        if (pcl::io::loadPCDFile<pcl::PointXYZ>(
                "../pcds/" + segmentationObject.currentLidar + "/alignedPointCloud_" + POINT_CLOUD_TIMESTAMP + ".pcd", 
                *cloud) == -1)
        {
            PCL_ERROR ("Couldn't read file \n");
            return (-1);
        }
        std::cout << "Loaded point cloud with " << cloud->width * cloud->height << " data points" << std::endl;

        // Select ROI based on LiDAR ID
        // Creates a filtered point cloud based on the checkerboard region of interest set in the config section
        pcl::PointCloud<pcl::PointXYZ>::Ptr filterPointCloud (new pcl::PointCloud<pcl::PointXYZ>);
        if (segmentationObject.currentLidar == "233") {
            filterPointCloud = segmentationObject.filterPointCloud(cloud, ROI_233.xMin, ROI_233.xMax, ROI_233.yMin, ROI_233.yMax);
        }
        else {
            filterPointCloud = segmentationObject.filterPointCloud(cloud, ROI_234.xMin, ROI_234.xMax, ROI_234.yMin, ROI_234.yMax);
        }

        // Segmentation workflow
        //Perform a series of operations on the filtered point cloud: estimate the surface normals, find the vertical planes in the point cloud (based on
        //the surface normals), perform a Euclidean distance based clustering operation, display the clusters with color on the original point cloud
        segmentationObject.pointCloudNormals = segmentationObject.normalEstimation(filterPointCloud);
        segmentationObject.pointCloudVerticalSurfaces = segmentationObject.findVerticalPlanes(filterPointCloud, segmentationObject.pointCloudNormals);
        segmentationObject.pointCloudClusters = segmentationObject.euclideanClustering(filterPointCloud, segmentationObject.pointCloudVerticalSurfaces);
        segmentationObject.createClusterCloud(filterPointCloud, segmentationObject.clusterIndices, segmentationObject.pointCloudVerticalSurfaces);
        
        // Interactive ROI selection
        //Store the minimum and maximum x,y,z locations for the region of interest surrounding the checkerboard
        float lidarXMin, lidarXMax, lidarYMin, lidarYMax, lidarZMin, lidarZMax;
        std::cout << "X minimum?" << std::endl; std::cin >> lidarXMin;
        std::cout << "X maximum?" << std::endl; std::cin >> lidarXMax;
        std::cout << "Y minimum?" << std::endl; std::cin >> lidarYMin;
        std::cout << "Y maximum?" << std::endl; std::cin >> lidarYMax;
        std::cout << "Z minimum?" << std::endl; std::cin >> lidarZMin;
        std::cout << "Z maximum?" << std::endl; std::cin >> lidarZMax;

        // Save centroid
        //Add the centroid of the checkerboard to the centroids.csv file, which will be used for generating an initial transformation matrix
        if (segmentationObject.currentLidar == "233") {
            segmentationObject.filterCentroidList(segmentationObject.lidarOneAllCentroids,
                                                  lidarXMin, lidarXMax, lidarYMin, lidarYMax, lidarZMin, lidarZMax,
                                                  POINT_CLOUD_TIMESTAMP);
        }
        else if (segmentationObject.currentLidar == "234") {
            segmentationObject.filterCentroidList(segmentationObject.lidarTwoAllCentroids,
                                                  lidarXMin, lidarXMax, lidarYMin, lidarYMax, lidarZMin, lidarZMax,
                                                  POINT_CLOUD_TIMESTAMP);
        }
    }

    //Uses the current centroids.csv file to generate a transformation matrix and then initiate ICP algorithm
    else {
        //Create a generateTransform object which will generate a transformation matrix using the centroids of the checkerboard in the various 
        //point cloud files analyzed above
        generateTransform generateTransformObject;

        //Get the checkerboard centroids
        std::pair<Eigen::MatrixXd, Eigen::MatrixXd> filteredLidarPoints = generateTransformObject.getCentroidsFromCSV(CENTROID_CSV);

        //Generate the transformation matrix and output to the terminal
        std::cout << "Computing the transformation using the centroids" << std::endl;
        Eigen::Affine3d transformationMatrix = 
            generateTransformObject.computeTransformation(filteredLidarPoints.first, filteredLidarPoints.second);
        std::cout << "Initial Rotation :\n" << transformationMatrix.rotation() << std::endl;
        std::cout << "Initial Translation:\n" << transformationMatrix.translation() << std::endl;

        //Create an ICP object which will be used to further refine the transformation that was generated above
        ICP icpObject;
        std::vector<std::string> timestamps = icpObject.extractTimestampsFromFilenames(PCD_DIR_233);
        std::vector<std::string> lidarOneFilenames = icpObject.getFilenames(PCD_DIR_233);
        std::vector<std::string> lidarTwoFilenames = icpObject.getFilenames(PCD_DIR_234);

        //Sort the timestamps from smallest to largest
        std::sort(timestamps.begin(), timestamps.end());

        // Iterate through the list of timestamps
        for (const std::string& timestamp : timestamps) {
            for (const std::string& lidarOnePCD : lidarOneFilenames) {
                for (const std::string& lidarTwoPCD : lidarTwoFilenames) {

                    //Find the two point cloud files for LiDAR one and two that have the closest matching timestamps
                    if (lidarOnePCD.find(timestamp) != std::string::npos &&
                        lidarTwoPCD.find(timestamp) != std::string::npos)
                    {
                        std::cout << "lidar one: " << lidarOnePCD << std::endl;
                        std::cout << "lidar two: " << lidarTwoPCD << std::endl;

                        //Load the two pointcloud files                        
                        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
                        if (pcl::io::loadPCDFile<pcl::PointXYZ>(PCD_DIR_233 + lidarOnePCD, *cloud) == -1) {
                            PCL_ERROR ("Couldn't read first PCD \n");
                            return (-1);
                        }

                        pcl::PointCloud<pcl::PointXYZ>::Ptr cloudTwo (new pcl::PointCloud<pcl::PointXYZ>);
                        if (pcl::io::loadPCDFile<pcl::PointXYZ>(PCD_DIR_234 + lidarTwoPCD, *cloudTwo) == -1) {
                            PCL_ERROR ("Couldn't read second PCD \n");
                            return (-1);
                        }

                        //Perform the ICP operation and store the output transformation matrix. This transformation matrix will then be used
                        //as the initial guess for the next iteration in the loop                        
                        Eigen::Matrix4f icpOutput = icpObject.performICP(cloud, cloudTwo, transformationMatrix);
                        
                        Eigen::Matrix4d transformationMatrixDouble = icpOutput.cast<double>();

                        // Now, use this matrix to initialize an Eigen::Affine3d object                        
                        transformationMatrix = Eigen::Affine3d(transformationMatrixDouble);

                        std::cout << "Rotation:\n" << transformationMatrix.rotation() << std::endl;
                        std::cout << "Translation:\n" << transformationMatrix.translation() << std::endl;
                    }
                }
            }
        }
    }

    return (0);
}