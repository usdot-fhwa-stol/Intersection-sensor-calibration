#ifndef GENERATETRANSFORM_H
#define GENERATETRANSFORM_H

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
#include <Eigen/Dense>

class generateTransform
{
    public:
        generateTransform();
        std::vector<std::string> split(const std::string &s, char delimiter);
        std::pair<Eigen::MatrixXd, Eigen::MatrixXd> getCentroidsFromCSV(std::string centroidsFilePath);
        static void appendPoint(Eigen::MatrixXd& matrix, const Eigen::Vector3d& pointVector);
        Eigen::Affine3d computeTransformation(const Eigen::MatrixXd& src, const Eigen::MatrixXd& dst);
};

#endif