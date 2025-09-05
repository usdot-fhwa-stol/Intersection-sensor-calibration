#ifndef SEGMENTATION_H
#define SEGMENTATION_H


#include <iostream>
#include <pcl/ModelCoefficients.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/features/normal_3d.h>
#include <pcl/PointIndices.h>
#include <pcl/segmentation/region_growing.h>
#include <pcl/filters/passthrough.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/filters/conditional_removal.h>
#include <Eigen/Dense>
#include <thread>


class segmentation
{

    public:
        segmentation();
        static void keyboardEventOccurred(const pcl::visualization::KeyboardEvent &event, void* viewer_void);
        static void visualizeCloud(const pcl::PointCloud<pcl::PointXYZ>::Ptr& original_cloud, const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud, const std::string& viewer_name);
        pcl::PointCloud<pcl::Normal>::Ptr normalEstimation(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud);
        pcl::PointCloud<pcl::PointXYZ>::Ptr findVerticalPlanes(const pcl::PointCloud<pcl::PointXYZ>::Ptr& input_cloud, const pcl::PointCloud<pcl::Normal>::Ptr& cloud_normals);
        void appendPoint(Eigen::MatrixXd& matrix, const Eigen::Vector3d& pointVector);
        void addToCentroidList(pcl::PointXYZ centroid);
        static pcl::PointXYZ calculateClusterCenter(const pcl::PointCloud<pcl::PointXYZ>::Ptr& vertical_surfaces_cloud, const pcl::PointIndices& cluster);
        std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> euclideanClustering(const pcl::PointCloud<pcl::PointXYZ>::Ptr& input_cloud, 
            const pcl::PointCloud<pcl::PointXYZ>::Ptr& vertical_surfaces);
        void createClusterCloud(const pcl::PointCloud<pcl::PointXYZ>::Ptr& original_cloud, std::vector<pcl::PointIndices> cluster_indices, 
            const pcl::PointCloud<pcl::PointXYZ>::Ptr& vertical_surfaces);
        void appendToCSV(Eigen::Vector3d centroid, std::string pointCloudName);
        void filterCentroidList(Eigen::MatrixXd centroids, float lidarXMin, float lidarXMax, float lidarYMin, float lidarYMax, float lidarZMin, 
            float lidarZMax, std::string pointCloudName);
        Eigen::Affine3d computeTransformation(const Eigen::MatrixXd& src, const Eigen::MatrixXd& dst);

        pcl::PointCloud<pcl::PointXYZ>::Ptr filterPointCloud(const pcl::PointCloud<pcl::PointXYZ>::Ptr& original_cloud, float xMin, float xMax, float yMin, float yMax);

        std::string currentLidar;
        const int POINT_SIZE = 2; //Size of points in visualized pointcloud
        pcl::PointCloud<pcl::Normal>::Ptr pointCloudNormals;
        pcl::PointCloud<pcl::PointXYZ>::Ptr pointCloudVerticalSurfaces;
        std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> pointCloudClusters;
        std::vector<pcl::PointIndices> clusterIndices;
        Eigen::MatrixXd lidarOneAllCentroids; //LiDAR 233
        Eigen::MatrixXd lidarTwoAllCentroids; //LiDAR 234
        std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> clusters; //clusters of vertical planes identified
        std::vector<float> lidarOneBounds;
        std::vector<float> lidarTwoBounds;
        
};

#endif