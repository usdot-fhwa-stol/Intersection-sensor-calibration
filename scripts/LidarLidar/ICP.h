#ifndef ICP_H
#define ICP_H

#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <Eigen/Dense>
#include <iostream>
#include <filesystem>
#include <regex>
#include <vector>
#include <string>


class ICP
{
    public:
        ICP();
        std::vector<std::string> extractTimestampsFromFilenames(const std::string& directoryPath);
        Eigen::Matrix4f performICP(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_source, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_target,
            Eigen::Affine3d initial_guess);
        std::vector<std::string> getFilenames(const std::string& directoryPath);

};

#endif