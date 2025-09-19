#include "ICP.h"

// Standard Library
#include <iostream>
#include <filesystem>
#include <regex>
#include <vector>
#include <string>

// Eigen
#include <Eigen/Dense>
#include <Eigen/Geometry>

// PCL
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>
#include <pcl/visualization/pcl_visualizer.h>


// ================= Configuration Parameters =================

// ICP parameters
constexpr int    ICP_MAX_ITERATIONS         = 3000;   // Maximum number of iterations
constexpr double ICP_TRANSFORMATION_EPSILON = 1e-9;   // Convergence threshold
constexpr double ICP_MAX_CORRESPONDENCE     = 0.2;    // Maximum correspondence distance (meters)

// Timestamp regex
// Matches strings like "15_31_54_1707424549.5918" from filenames
const std::regex TIMESTAMP_PATTERN(R"((\d{2}_\d{2}_\d{2}_\d+))");

// ============================================================

/**
 * The constructor for the ICP class
*/
ICP::ICP() {
    std::cout << "Creating ICP object" << std::endl;

}

/**
 * This method will get the filenames of the PCDs in an input directory
 * 
 * @param directoryPath The directory with the PCD files for one LiDAR 
 * @return std::vector<std::string> A vector of filenames
 */
std::vector<std::string> ICP::getFilenames(const std::string& directoryPath) {
    std::vector<std::string> filenames;

    for (const auto& entry : std::filesystem::directory_iterator(directoryPath)) {
        std::string filename = entry.path().filename().string();

        filenames.push_back(filename);
    }

    return filenames;
}

/**
 * This method will extract the timestamps from the PCD files in the input directory path. This is used
 * to find the corresponding PCD from the other LiDAR with the closest timestamp match.
 * 
 * @param directoryPath The directory with the PCD files for one LiDAR 
 * @return std::vector<std::string> A vector of timestamps
 */
std::vector<std::string> ICP::extractTimestampsFromFilenames(const std::string& directoryPath) {
    std::vector<std::string> timestamps;
    for (const auto& entry : std::filesystem::directory_iterator(directoryPath)) {
        std::string filename = entry.path().filename().string();
        std::smatch matches;
        if (std::regex_search(filename, matches, TIMESTAMP_PATTERN)) {
            if (!matches.empty()) {
                // Assuming the first match is the timestamp
                timestamps.push_back(matches[1].str());
            }
        }
    }

    return timestamps;
}

/**
 * Perform the Iterative Closest Point algorithm point cloud alignment using an initial guess for the transformation matrix 
 * that should help the algorithm's performance.
 * 
 * @param cloud_source Source point cloud
 * @param cloud_target Target point cloud
 * @param initial_guess Initial transformation matrix guess
 * @return Eigen::Matrix4f The transformation matrix that aligns the source cloud to the target cloud
 */
Eigen::Matrix4f ICP::performICP(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_source, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_target,
    Eigen::Affine3d initial_guess) {

    // Convert Eigen::Affine3d to Eigen::Matrix4f for PCL
    Eigen::Matrix4f initial_guess_matrix = initial_guess.matrix().cast<float>();

    // Set up ICP
    pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
    icp.setInputSource(cloud_source);
    icp.setInputTarget(cloud_target);
    // using config constants
    icp.setMaximumIterations(ICP_MAX_ITERATIONS);
    icp.setTransformationEpsilon(ICP_TRANSFORMATION_EPSILON);
    icp.setMaxCorrespondenceDistance(ICP_MAX_CORRESPONDENCE);

    // Use the initial guess
    icp.align(*cloud_source, initial_guess_matrix);

    //If the ICP algorithm has converged, print the fitness score, which should be the average squared distance between
    //corresponding points in the point clouds
    if (icp.hasConverged()) {
        std::cout << "ICP converged." << std::endl
                  << "The score is " << icp.getFitnessScore() << std::endl;

        initial_guess_matrix = icp.getFinalTransformation();
    } else {
        std::cout << "ICP did not converge, returning initial." << std::endl;
    }

    return initial_guess_matrix;

}