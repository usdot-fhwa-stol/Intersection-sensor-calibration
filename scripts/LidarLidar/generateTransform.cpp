#include "generateTransform.h"

/**
 * The constructor for the generateTransform class
*/
generateTransform::generateTransform() {
    std::cout << "Creating generateTransform object" << std::endl;   
}

/**
 * Resize the matrix containing the centroids of vertical surfaces observed by the LiDAR
 * and add a new centroid to it.
 * 
 * @param matrix The matrix with 3D centroids for this LiDAR
 * @param point The new centroid to add
 */
void generateTransform::appendPoint(Eigen::MatrixXd& matrix, const Eigen::Vector3d& pointVector) {
    // Resize the matrix to accommodate one more row
    matrix.conservativeResize(matrix.rows() + 1, Eigen::NoChange);
    // Set the last row to the new point
    matrix.row(matrix.rows() - 1) = pointVector;
}

/**
 * Split a string from a csv file by a delimiter 
 * 
 * @param s The string to split
 * @param delimiter The delimiter, a comma in the case of a csv file
 * @return A vector of the split strings
 */
std::vector<std::string> generateTransform::split(const std::string &s, char delimiter) {
    std::vector<std::string> tokens;
    std::string token;
    std::istringstream tokenStream(s);
    while (std::getline(tokenStream, token, delimiter)) {
        tokens.push_back(token);
    }
    return tokens;
}

/**
 * Get the centroids from the centroids.csv file corresponding to the centroid location of the
 * checkerboard in the analyzed point clouds 
 * 
 * @return The matching centroids as a std::pair of Eigen::MatrixXd
 */
std::pair<Eigen::MatrixXd, Eigen::MatrixXd> generateTransform::getCentroidsFromCSV(std::string centroidsFilePath) {

    //Attempt to open the centroids csv file
    bool readSuccess = false;
    std::ifstream centroidFile;
    try
    {
        std::cout << "Reading the csv file" << std::endl;
        centroidFile.open(centroidsFilePath);

        readSuccess = true;

    }
    catch(const std::exception& e)
    {
        std::cout << "Error reading the csv file" << std::endl;
    }

    std::pair<Eigen::MatrixXd, Eigen::MatrixXd> lidarPoints;
    
    //If the file is opened successfully, begin to read it line by line
    if (readSuccess == true) {
        std::string line;

        //Create vectors of Eigen::Vector3d for each three-dimensional centroid
        std::vector<Eigen::Vector3d> pointsLiDAR233, pointsLiDAR234;

        // Skip header line
        std::getline(centroidFile, line);

        // Read each line from the CSV
        while (std::getline(centroidFile, line)) {

            //Split the line by comma delimiter and extract the lidar name and centroid
            auto tokens = split(line, ',');
            int lidarID = std::stoi(tokens[0]);
            Eigen::Vector3d point(std::stod(tokens[2]), std::stod(tokens[3]), std::stod(tokens[4]));

            //Add each centroid to its associated vector
            if (lidarID == 233) {
                pointsLiDAR233.push_back(point);
            } else if (lidarID == 234) {
                pointsLiDAR234.push_back(point);
            }
        }

        //Create two Eigen::MatrixXd and add the centroids
        Eigen::MatrixXd matrixLiDAR233(0, 3);
        Eigen::MatrixXd matrixLiDAR234(0, 3);

        for (size_t i = 0; i < pointsLiDAR233.size(); ++i) {
            appendPoint(matrixLiDAR233, pointsLiDAR233[i]);
        }

        for (size_t i = 0; i < pointsLiDAR234.size(); ++i) {
            appendPoint(matrixLiDAR234, pointsLiDAR234[i]);
        }

        // Output the matrices for verification
        std::cout << "Centroids for LiDAR 233:\n" << matrixLiDAR233 << "\n\n";
        std::cout << "Centroids for LiDAR 234:\n" << matrixLiDAR234 << std::endl;

        //Return the centroids as a std::pair
        lidarPoints.first = matrixLiDAR233;
        lidarPoints.second = matrixLiDAR234;
    }    

    return lidarPoints;
}

/**
 * Generate the transformation matrix between two sets of points
 * 
 * @param src The first list of points corresponding to the centroids from Lidar one
 * @param dst The second list of points corresponding to the centroids from Lidar two
 * @return Eigen::Affine3d The transformation matrix
 */
Eigen::Affine3d generateTransform::computeTransformation(const Eigen::MatrixXd& src, const Eigen::MatrixXd& dst) {
    // Ensure setA and setB have the same size
    if (src.rows() != dst.rows() || src.cols() != dst.cols()) {
        throw std::runtime_error("Input sets must have the same size.");
    }

    // Step 1: Compute centroids
    Eigen::Vector3d centroidSrc = src.colwise().mean();
    Eigen::Vector3d centroidDst = dst.colwise().mean();

    // Step 2: Subtract centroids
    Eigen::MatrixXd srcAdjusted = src.rowwise() - centroidSrc.transpose();
    Eigen::MatrixXd dstAdjusted = dst.rowwise() - centroidDst.transpose();

    // Step 3: Compute cross-covariance matrix
    Eigen::Matrix3d H = srcAdjusted.transpose() * dstAdjusted;

    // Step 4: SVD
    Eigen::JacobiSVD<Eigen::MatrixXd> svd(H, Eigen::ComputeFullU | Eigen::ComputeFullV);
    Eigen::Matrix3d R = svd.matrixV() * svd.matrixU().transpose();

    // Ensure a right-handed coordinate system
    if (R.determinant() < 0) {
        Eigen::Matrix3d V = svd.matrixV();
        V.col(2) *= -1;
        R = V * svd.matrixU().transpose();
    }

    // Step 5: Compute translation
    Eigen::Vector3d t = centroidDst - R * centroidSrc;

    // Create affine transformation
    Eigen::Affine3d transformation;
    transformation.linear() = R;
    transformation.translation() = t;

    return transformation;
}