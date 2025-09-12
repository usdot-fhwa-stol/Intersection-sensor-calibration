#include "segmentation.h"
#include "generateTransform.h"
#include "ICP.h"

int main()
{ 
    //Boolean for either performing segmentation or generating transformation matrices
    bool performingSegmentation = false;

    if (performingSegmentation) {
        //Create the segmentation object which is used to perform a variety of operations on the point cloud, such as filtering,
        //normal estimation, clustering, and more
        segmentation segmentationObject;

        //The timestamp in the point cloud filename    
        std::string pointCloudStr = "15_31_54_1707424549.5918";

        //Set this member variable to the LiDAR being worked with (233 or 234)
        segmentationObject.currentLidar = "234";

        //Load the desired point cloud file
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
        if (pcl::io::loadPCDFile<pcl::PointXYZ> ("../pcds/"+segmentationObject.currentLidar+"/alignedPointCloud_"+pointCloudStr+".pcd", *cloud) == -1)
        {
            PCL_ERROR ("Couldn't read file \n");
            return (-1);
        }
        std::cout << "Loaded point cloud with " << cloud->width * cloud->height << " data points" << std::endl;

        //Create a filtered point cloud based on a region of interest that is focused on the checkerboard. The x and y min/max values passed in to the
        //filterPointCloud function most likely will change based on the specific point cloud file
        pcl::PointCloud<pcl::PointXYZ>::Ptr filterPointCloud (new pcl::PointCloud<pcl::PointXYZ>);
        if (segmentationObject.currentLidar == "233") {
            filterPointCloud = segmentationObject.filterPointCloud(cloud, 7.75, 12, -8, -3.75);
        }
        else {
            filterPointCloud = segmentationObject.filterPointCloud(cloud, 19, 23, -27, -23);
        }


        //Perform a series of operations on the filtered point cloud: estimate the surface normals, find the vertical planes in the point cloud (based on
        //the surface normals), perform a Euclidean distance based clustering operation, display the clusters with color on the original point cloud
        segmentationObject.pointCloudNormals = segmentationObject.normalEstimation(filterPointCloud);
        segmentationObject.pointCloudVerticalSurfaces = segmentationObject.findVerticalPlanes(filterPointCloud, segmentationObject.pointCloudNormals);
        segmentationObject.pointCloudClusters = segmentationObject.euclideanClustering(filterPointCloud, segmentationObject.pointCloudVerticalSurfaces);
        segmentationObject.createClusterCloud(filterPointCloud, segmentationObject.clusterIndices, segmentationObject.pointCloudVerticalSurfaces);
        
        //Store the minimum and maximum x,y,z locations for the region of interest surrounding the checkerboard
        float lidarXMin, lidarXMax, lidarYMin, lidarYMax, lidarZMin, lidarZMax;
        std::cout << "X minimum?" << std::endl;
        std::cin >> lidarXMin;
        std::cout << "X maximum?" << std::endl;
        std::cin >> lidarXMax;
        std::cout << "Y minimum?" << std::endl;
        std::cin >> lidarYMin;
        std::cout << "Y maximum?" << std::endl;
        std::cin >> lidarYMax;
        std::cout << "Z minimum?" << std::endl;
        std::cin >> lidarZMin;
        std::cout << "Z maximum?" << std::endl;
        std::cin >> lidarZMax;

        //Add the centroid of the checkerboard to the centroids.csv file, which will be used for generating an initial transformation matrix
        if (segmentationObject.currentLidar == "233") {
            segmentationObject.filterCentroidList(segmentationObject.lidarOneAllCentroids, lidarXMin, lidarXMax, lidarYMin, lidarYMax, lidarZMin, lidarZMax, pointCloudStr);
        }
        else if (segmentationObject.currentLidar == "234") {
            segmentationObject.filterCentroidList(segmentationObject.lidarTwoAllCentroids, lidarXMin, lidarXMax, lidarYMin, lidarYMax, lidarZMin, lidarZMax, pointCloudStr);
        }
    }
    
    //Use the current centroids.csv file to generate a transformation matrix and then initiate ICP algorithm
    else {
        //Create a generateTransform object which will generate a transformation matrix using the centroids of the checkerboard in the various 
        //point cloud files analyzed above
        generateTransform generateTransformObject;

        //Get the checkerboard centroids
        std::string centroidCSVName = "../centroids.csv";
        std::pair<Eigen::MatrixXd, Eigen::MatrixXd> filteredLidarPoints = generateTransformObject.getCentroidsFromCSV(centroidCSVName);

        //Generate the transformation matrix and output to the terminal
        std::cout << "Computing the transformation using the centroids" << std::endl;
        Eigen::Affine3d transformationMatrix = generateTransformObject.computeTransformation(filteredLidarPoints.first, filteredLidarPoints.second);
        std::cout << "Initial Rotation :\n" << transformationMatrix.rotation() << std::endl;
        std::cout << "Initial Translation:\n" << transformationMatrix.translation() << std::endl;

        //Create an ICP object which will be used to further refine the transformation that was generated above
        ICP icpObject;
        std::vector<std::string> timestamps = icpObject.extractTimestampsFromFilenames("../pcds/233/");
        
        std::vector<std::string> lidarOneFilenames = icpObject.getFilenames("../pcds/233/");
        std::vector<std::string> lidarTwoFilenames = icpObject.getFilenames("../pcds/234/");

        //Sort the timestamps from smallest to largest
        std::sort(timestamps.begin(), timestamps.end());


        // Iterate through the list of timestamps
        for (std::string timestamp: timestamps) {
            for (std::string lidarOnePCD: lidarOneFilenames) {
                for (std::string lidarTwoPCD: lidarTwoFilenames) {

                    //Find the two point cloud files for LiDAR one and two that have the closest matching timestamps
                    if (lidarOnePCD.find(timestamp) != std::string::npos && lidarTwoPCD.find(timestamp) != std::string::npos)
                    {
                        std::cout << "lidar one: " << lidarOnePCD << std::endl;
                        std::cout << "lidar two: " << lidarTwoPCD << std::endl;

                        //Load the two pointcloud files
                        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
                        if (pcl::io::loadPCDFile<pcl::PointXYZ> ("../pcds/233/"+lidarOnePCD, *cloud) == -1)
                        {
                            PCL_ERROR ("Couldn't read first PCD \n");
                            return (-1);
                        }
                        pcl::PointCloud<pcl::PointXYZ>::Ptr cloudTwo (new pcl::PointCloud<pcl::PointXYZ>);
                        if (pcl::io::loadPCDFile<pcl::PointXYZ> ("../pcds/234/"+lidarTwoPCD, *cloudTwo) == -1)
                        {
                            PCL_ERROR ("Couldn't read second PCD \n");
                            return (-1);
                        }

                        //Perform the ICP operation and store the output transformation matrix. This transformation matrix will then be used
                        //as the initial guess for the next iteration in the loop
                        Eigen::Matrix4f icpOutput = icpObject.performICP(cloud, cloudTwo, transformationMatrix);

                        Eigen::Matrix4d transformationMatrixDouble = icpOutput.cast<double>();

                        // Now, use this matrix to initialize an Eigen::Affine3d object
                        transformationMatrix = Eigen::Affine3d(transformationMatrixDouble);

                        std::cout << "Rotation:\n" << transformationMatrix.rotation() << std::endl;\
                        std::cout << "Translation:\n" << transformationMatrix.translation() << std::endl;

                    }

                }
            }
        }    

    }        
            
    return (0);
}