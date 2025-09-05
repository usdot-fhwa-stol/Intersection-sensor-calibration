#include "segmentation.h"

/**
 * The constructor for the segmentation class. Initializes the two centroid matrices to 3x3 to store the 3D centroid locations.
*/
segmentation::segmentation() {
    std::cout << "Creating segmentation object" << std::endl;   

    this->lidarOneAllCentroids.resize(3, 3); //LiDAR 233
    this->lidarTwoAllCentroids.resize(3, 3); //LiDAR 234

}

/**
 * Filter the point cloud to a specific region of interest surrounding the checkerboard. Determining the optimal set of bounds
 * is using a trial and error process for each PCD.
 * 
 * @param original_cloud The original point cloud to be filtered
 * @param xMin Minimum x bound
 * @param xMax Maximum x bound
 * @param yMin Minimum y bound
 * @param yMax Maximum y bound
 * @return pcl::PointCloud<pcl::PointXYZ>::Ptr The filtered point cloud
 */
pcl::PointCloud<pcl::PointXYZ>::Ptr segmentation::filterPointCloud(const pcl::PointCloud<pcl::PointXYZ>::Ptr& original_cloud, float xMin, float xMax, float yMin, float yMax) {
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);

    // Create the filtering condition based on the bounds
    pcl::ConditionAnd<pcl::PointXYZ>::Ptr range_cond(new pcl::ConditionAnd<pcl::PointXYZ>());
    range_cond->addComparison(pcl::FieldComparison<pcl::PointXYZ>::ConstPtr(new
    pcl::FieldComparison<pcl::PointXYZ>("x", pcl::ComparisonOps::GE, xMin))); // Greater than min_x
    range_cond->addComparison(pcl::FieldComparison<pcl::PointXYZ>::ConstPtr(new
    pcl::FieldComparison<pcl::PointXYZ>("x", pcl::ComparisonOps::LE, xMax))); // Less than max_x
    range_cond->addComparison(pcl::FieldComparison<pcl::PointXYZ>::ConstPtr(new
    pcl::FieldComparison<pcl::PointXYZ>("y", pcl::ComparisonOps::GE, yMin))); // Greater than min_y
    range_cond->addComparison(pcl::FieldComparison<pcl::PointXYZ>::ConstPtr(new
    pcl::FieldComparison<pcl::PointXYZ>("y", pcl::ComparisonOps::LE, yMax))); // Less than max_y

    // Apply the filter
    pcl::ConditionalRemoval<pcl::PointXYZ> condrem;
    condrem.setCondition(range_cond);
    condrem.setInputCloud(original_cloud);
    condrem.setKeepOrganized(false);
    condrem.filter(*cloud_filtered);

    return cloud_filtered;
}

/**
 * This registers an attempt to close the window with the Esc key
 * 
 * @param event  the keyboard press
 * @param viewer_void viewer object
 */
void segmentation::keyboardEventOccurred(const pcl::visualization::KeyboardEvent &event, void* viewer_void) {
    //Closing callback is activated with pressing 'Esc'
    if (event.getKeySym() == "Escape" && event.keyDown()) {
        std::cout << "esc key pressed" << std::endl;
        pcl::visualization::PCLVisualizer *viewer = static_cast<pcl::visualization::PCLVisualizer *>(viewer_void);
        viewer->close();
    }
}

/**
 * Method to visualize a desired point cloud 
 * 
 * @param cloud Name of the pcd file
 * @param viewer_name Name of the PCD cloud viewer object
 */
void segmentation::visualizeCloud(const pcl::PointCloud<pcl::PointXYZ>::Ptr& original_cloud, const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud, 
    const std::string& viewer_name) {
    // Create PCL Visualizer object and assign keyboard callback function 
    pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer(viewer_name));

    viewer->registerKeyboardCallback(keyboardEventOccurred, (void*)viewer.get());

    // Add the point cloud passed in to the Visualizer
    viewer->removeAllPointClouds();
    viewer->removeAllShapes();
    viewer->addPointCloud<pcl::PointXYZ>(original_cloud, "original");
    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "original");
    viewer->addPointCloud<pcl::PointXYZRGB>(cloud, viewer_name);
    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, viewer_name);
    viewer->addCoordinateSystem(1.0, "axis", 0);

    //Add visual labels for x,y, and z axes tick marks to locate approximate region of an observed surface
    for (int i = -50; i <= 50; ++i) {
        if (i != 0) { 
            viewer->addText3D(std::to_string(i), pcl::PointXYZ(i, 0, 0), 0.3, 1.0, 0, 0, "x_label_" + std::to_string(i));
            viewer->addText3D(std::to_string(i), pcl::PointXYZ(0, i, 0), 0.3, 0, 1.0, 0, "y_label_" + std::to_string(i));
        }
    }
    for (int j = -10; j <= 10; ++j) {
        if (j != 0) { 
            viewer->addText3D(std::to_string(j), pcl::PointXYZ(0, 0, j), 0.3, 0, 0, 1.0, "z_label_" + std::to_string(j));
        }
    }
    
    // Keep viewer open until ESC key hit
    while (!viewer->wasStopped()) {
        viewer->spinOnce(100);
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
}

/**
 * @brief Generate the normals to the surfaces in the point cloud.
 * 
 * @param cloud The original point cloud
 * @return pcl::PointCloud<pcl::Normal>::Ptr The normals
 */
pcl::PointCloud<pcl::Normal>::Ptr segmentation::normalEstimation(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud) {
    std::cout << "Generating normals in the point cloud" << std::endl;
    // Create the normals, the vectors that are perpendicular to the tangent plane at a point on a surface
    pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
    pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> normal_estimator;
    normal_estimator.setInputCloud(cloud);
    // normal_estimator.setRadiusSearch(0.25); //in meters, works for lidar-cam data
    normal_estimator.setRadiusSearch(1); //in meters, kinda works for lidar-lidar data

     // Create KD tree for nearest neighbors search
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
    normal_estimator.setSearchMethod(tree);
    normal_estimator.compute(*normals);


    // Initialize visualizer for surface normal verification
    // pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer("test"));
    
    // // Add the point cloud to the visualizer
    // viewer->addPointCloud<pcl::PointXYZ>(cloud, "cloud");
    // // Add the normals to the visualizer
    // // Here, 0.02 is the length of the normals, 1 is the step size (show every normal)
    // viewer->addPointCloudNormals<pcl::PointXYZ, pcl::Normal>(cloud, normals, 1, 0.25, "normals");

    // // Add visual labels for x and y axes tick marks to locate approximate region of an observed surface
    // for (int i = -50; i <= 50; ++i) {
    //     if (i != 0) { // Assuming the origin is already clear without a label
    //         viewer->addText3D(std::to_string(i), pcl::PointXYZ(i, 0, 0), 0.3, 1.0, 0, 0, "x_label_" + std::to_string(i));
    //         viewer->addText3D(std::to_string(i), pcl::PointXYZ(0, i, 0), 0.3, 0, 1.0, 0, "y_label_" + std::to_string(i));
    //     }
    // }

    // // Keep viewer open until ESC key hit
    // while (!viewer->wasStopped()) {
    //     viewer->spinOnce(100);
    //     std::this_thread::sleep_for(std::chrono::milliseconds(100));
    // }

    return normals;
}

/**
 * This method finds the vertical surfaces in the pointcloud by analyzing the angle of the surface normals versus the z axis
 * 
 * @param input_cloud The original point cloud
 * @param cloud_normals The surface normals of the original point cloud
 * @return pcl::PointCloud<pcl::PointXYZ>::Ptr A point cloud with only the vertical surfaces
 */
pcl::PointCloud<pcl::PointXYZ>::Ptr segmentation::findVerticalPlanes(const pcl::PointCloud<pcl::PointXYZ>::Ptr& input_cloud, const pcl::PointCloud<pcl::Normal>::Ptr& cloud_normals)
{
    std::cout << "Searching for vertical planes using the cloud normals" << std::endl;

    std::vector<int> vertical_points_indices;

    for (size_t i = 0; i < cloud_normals->size(); ++i) {
        float nx = cloud_normals->points[i].normal_x;
        float ny = cloud_normals->points[i].normal_y;
        float nz = cloud_normals->points[i].normal_z;
        
        // Compute the angle between the normal and the Z-axis
        float angle = std::acos(nz / std::sqrt(nx*nx + ny*ny + nz*nz)) * 180.0 / M_PI;
        
        // Define a threshold for verticality, e.g., normals within [75, 105] degrees are considered vertical
        if (angle > 50 && angle < 130) {
            vertical_points_indices.push_back(i);
        }
    }

    //Create a point cloud consisting of only the vertical surfaces
    pcl::PointCloud<pcl::PointXYZ>::Ptr vertical_surfaces_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    for (int index : vertical_points_indices) {
        vertical_surfaces_cloud->points.push_back(input_cloud->points[index]);
    }
    vertical_surfaces_cloud->width = vertical_surfaces_cloud->points.size();
    vertical_surfaces_cloud->height = 1;
    vertical_surfaces_cloud->is_dense = true;

    // visualizeTest(vertical_surfaces_cloud, this->currentLidar);  

    return vertical_surfaces_cloud;
}

/**
 * Resize the matrix containing the centroids of vertical surfaces observed by the LiDAR
 * and add a new centroid to it.
 * 
 * @param matrix The matrix with 3D centroids for this LiDAR
 * @param point The new centroid to add
 */
void segmentation::appendPoint(Eigen::MatrixXd& matrix, const Eigen::Vector3d& pointVector) {
    // Resize the matrix to accommodate one more row
    matrix.conservativeResize(matrix.rows() + 1, Eigen::NoChange);
    // Set the last row to the new point
    matrix.row(matrix.rows() - 1) = pointVector;
}

/**
 * @brief Add the new centroid to the appropriate matrix
 * 
 * @param centroid The new centroid
 */
void segmentation::addToCentroidList(pcl::PointXYZ centroid) {

    Eigen::Vector3d newVector = Eigen::Vector3d(centroid.x, centroid.y, centroid.z);
     // Append the new point to the set
    if (this->currentLidar == "233"){
        this->appendPoint(this->lidarOneAllCentroids, newVector);
    }
    else if (this->currentLidar == "234") {
        this->appendPoint(this->lidarTwoAllCentroids, newVector);
    }
}

/**
 * This method is used to calculate the three-dimensional centroid of a cluster of points in the original
 * point cloud.
 * 
 * @param vertical_surfaces_cloud The original point cloud
 * @param cluster The indices of the cluster points in the original point cloud ref frame
 * @return pcl::PointXYZ 
 */
pcl::PointXYZ segmentation::calculateClusterCenter(const pcl::PointCloud<pcl::PointXYZ>::Ptr& vertical_surfaces_cloud, const pcl::PointIndices& cluster) {
    //Create a point which will be used for the centroid calculations
    pcl::PointXYZ center;
    center.x = 0;
    center.y = 0;
    center.z = 0;
    
    //Iterate through every index in the cluster and add its location (in the original cloud) to the previously
    //created point
    for (const auto& index : cluster.indices) {
        center.x += vertical_surfaces_cloud->points[index].x;
        center.y += vertical_surfaces_cloud->points[index].y;
        center.z += vertical_surfaces_cloud->points[index].z;
    }

    //Divide the three dimensions by the number of indices to get the centroid
    center.x /= cluster.indices.size();
    center.y /= cluster.indices.size();
    center.z /= cluster.indices.size();

    return center;
}

/**
 * @brief Determine the clusters of points that belong to the same object in the point cloud using a 
 * Euclidean distance clustering technique.
 * 
 * @param input_cloud The original point cloud
 * @param vertical_surfaces The previously generated vertical surfaces in the point cloud
 * @return std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> 
 */
std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> segmentation::euclideanClustering(const pcl::PointCloud<pcl::PointXYZ>::Ptr& input_cloud, 
    const pcl::PointCloud<pcl::PointXYZ>::Ptr& vertical_surfaces) {
    
    std::cout << "Beginning Euclidean clustering" << std::endl;

    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
    tree->setInputCloud(vertical_surfaces);
    pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
    ec.setClusterTolerance(0.6); //in meters, THIS IS THE MAIN PARAMETER TO TWEAK WHEN FORMING CLUSTERS
    ec.setMinClusterSize(20); // Minimum size of a cluster (adjust based on your needs)
    // ec.setMaxClusterSize(300); // Maximum size of a cluster (adjust based on your needs)
    ec.setSearchMethod(tree);
    ec.setInputCloud(vertical_surfaces);
    ec.extract(this->clusterIndices);

    //Iterate through each vector of point indices
    for (std::vector<pcl::PointIndices>::const_iterator it = this->clusterIndices.begin(); it != this->clusterIndices.end(); ++it) {

        //Create a new point cloud for the cluster
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster(new pcl::PointCloud<pcl::PointXYZ>);
        for (const auto& idx : it->indices)
            cloud_cluster->points.push_back(vertical_surfaces->points[idx]); //*

        cloud_cluster->width = cloud_cluster->points.size();
        cloud_cluster->height = 1;
        cloud_cluster->is_dense = true;

        pcl::PointXYZ clusterCenter = calculateClusterCenter(vertical_surfaces, *it);

        this->addToCentroidList(clusterCenter);
        
        clusters.push_back(cloud_cluster);
    }

    return clusters;
}

/**
 * @brief Creates a visualization of the vertical object clusters
 * 
 * @param cluster_indices The indices of the points associated with each vertical cluster
 * @param vertical_surfaces The vertical surface points
 */
void segmentation::createClusterCloud(const pcl::PointCloud<pcl::PointXYZ>::Ptr& original_cloud, std::vector<pcl::PointIndices> cluster_indices,
    const pcl::PointCloud<pcl::PointXYZ>::Ptr& vertical_surfaces) {

    std::cout << "Creating visualization of vertical object clusters" << std::endl;

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr colored_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    srand(static_cast<unsigned int>(time(0))); // Ensure different colors for each run

    for (const auto& cluster : cluster_indices) {
        // Generate a random color
        uint8_t r = static_cast<uint8_t>(rand() % 256);
        uint8_t g = static_cast<uint8_t>(rand() % 256);
        uint8_t b = static_cast<uint8_t>(rand() % 256);

        for (const auto& idx : cluster.indices) {
            pcl::PointXYZRGB point;
            point.x = vertical_surfaces->points[idx].x;
            point.y = vertical_surfaces->points[idx].y;
            point.z = vertical_surfaces->points[idx].z;
            point.r = r;
            point.g = g;
            point.b = b;
            colored_cloud->points.push_back(point);
        }
    }

    colored_cloud->width = colored_cloud->points.size();
    colored_cloud->height = 1;
    colored_cloud->is_dense = true;

    visualizeCloud(original_cloud, colored_cloud, this->currentLidar);  
}

/**
 * @brief Add the centroid of the cluster to the csv file containing all tracked centroids
 * 
 * @param centroid The centroid to be added
 */
void segmentation::appendToCSV(Eigen::Vector3d centroid, std::string pointCloudName) {
    // Path to the CSV file
    std::string filePath = "../centroids.csv";

    // Open the file in append mode
    std::ofstream fileStream(filePath, std::ios::app);

    if (fileStream.is_open()) {
        // Append the data to the file and close the file
        std::cout << "Appending " << centroid << " to csv file" << std::endl;
        fileStream << this->currentLidar << "," << pointCloudName << "," << centroid.x() << "," << centroid.y() << "," << centroid.z() << "\n";

        // Close the file
        fileStream.close();
    }
    else {
        std::cout << "Could not open file" << std::endl;
    }    

}

/**
 * @brief This method filters the list of centroids based on a set of x and y bounds. This is used to find
 * filter out all of the other vertical surfaces besides the checkerboard.
 * 
 * @param centroids List of centroids
 * @param lidarXMin Mininum x bound
 * @param lidarXMax Maximumum x bound
 * @param lidarYMin Minimum y bound
 * @param lidarYMax Maximum y bound
 */
void segmentation::filterCentroidList(Eigen::MatrixXd centroids, float lidarXMin, float lidarXMax,
    float lidarYMin, float lidarYMax, float lidarZMin, float lidarZMax, std::string pointCloudName) {
    
    //Iterate through the list of centroids
    for (int i = 0; i < centroids.rows(); ++i) {
        Eigen::Vector3d point = centroids.row(i);
        std::cout << "clusters:" << point << std::endl;

        //Filter based on the x and y location
        if (point.x() <= lidarXMax && point.x() >= lidarXMin && point.y() <= lidarYMax && point.y() >= lidarYMin && point.z() <= lidarZMax && point.z() >= lidarZMin) {
            std::cout << "Keeping LiDAR point " << point.transpose() << std::endl;

            //If the point satisfies the conditions, add it to the csv
            this->appendToCSV(point, pointCloudName);
        }
    }
}