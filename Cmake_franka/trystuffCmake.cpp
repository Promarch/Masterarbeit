#include <iostream>
#include <iomanip>  // For set print decimal precision
#include <fstream>  // Read files
#include <sstream>
#include <vector>
#include <cstdlib>  // for rand() and srand()
#include <ctime>    // for time()
#include <Eigen/Core>
#include <Eigen/Dense>


// Function to read in quaternion file
Eigen::MatrixXd readFileToMatrix(const std::string& filename) {
    std::vector<std::array<double, 4>> data;
    std::ifstream file(filename);

    if (!file) {
        std::cerr << "Error: Unable to open file " << filename << std::endl;
        return Eigen::MatrixXd(); // Return an empty matrix if the file cannot be opened
    }

    std::string line;
    while (std::getline(file, line)) {
        std::istringstream lineStream(line);
        std::array<double, 4> row;
        char comma;

        // Parse each line assuming it contains exactly four comma-separated double values
        for (double& value : row) {
            if (!(lineStream >> value)) {
                std::cerr << "Error: Failed to parse a value in the line." << std::endl;
                break;
            }
            lineStream >> comma; // Skip the comma
        }
        data.push_back(row);
    }

    file.close();

    // Convert vector of arrays to Eigen matrix
    Eigen::MatrixXd matrix(data.size(), 4);
    for (size_t i = 0; i < data.size(); ++i) {
        matrix(i, 0) = data[i][0];
        matrix(i, 1) = data[i][1];
        matrix(i, 2) = data[i][2];
        matrix(i, 3) = data[i][3];
    }
    return matrix;
}

// Function to convert quaternions to angle around x-axis using Eigen
Eigen::VectorXd convertQuaternionsToXAngles(const Eigen::MatrixXd& quat_matrix) {
    // Calculate the angle around the x-axis in one call
    return 2 * (quat_matrix.col(0).array() / quat_matrix.col(3).array()).atan() *180/M_PI;
}

// Function to generate a random angle
int generateRandomAngle(const std::vector<int>& completedAngles, int tolerance, int angleRangeStart, int angleRangeEnd) {
    std::vector<int> allAngles;
    std::vector<int> uncompletedAngles;

    // Create a list of all angles within the specified range
    for (int angle = angleRangeStart; angle <= angleRangeEnd; ++angle) {
        allAngles.push_back(angle);
    }

    // Check for uncompleted angles
    for (int angle : allAngles) {
        bool isCompleted = false;
        for (int completed : completedAngles) {
            if (std::abs(angle - completed) <= tolerance) {
                isCompleted = true;
                break;
            }
        }
        if (!isCompleted) {
            uncompletedAngles.push_back(angle);
        }
    }

    // Check if there are uncompleted angles left
    if (!uncompletedAngles.empty()) {
        // Select a random angle from the uncompleted angles
        int randomIndex = rand() % uncompletedAngles.size();
        return uncompletedAngles[randomIndex];
    } else {
        return -1;  // No uncompleted angles available
    }
}

int main() {
    // Seed the random number generator
    std::srand(std::time(nullptr)); // initialize random seed
    std::cout << std::fixed << std::setprecision(3);
    // Get vector
    std::string filename = "/home/alexandergerard/Documents/quat_stop_tests.txt";
    // std::string filename = "build/data_output_knee/quat_stop_data_20241028_220631.txt";
    Eigen::MatrixXd quat_stop = readFileToMatrix(filename);

    // Convert quaternions to angles around the x-axis
    Eigen::VectorXd angleFlex = convertQuaternionsToXAngles(quat_stop);

    // Print out the calculated angles
    // for (int i = 0; i < angleFlex.size(); ++i) {
    //     std::cout << "Quaternion " << quat_stop.row(i) << " -> x_angle: " << angleFlex[i] << "°" << std::endl;
    // }
    // printf("\n");
    // Generate new angles
    int angleRangeStart = -36;
    int angleRangeEnd  = -0;
    int tolerance = 5;
    Eigen::VectorXd allAngles = Eigen::VectorXd::LinSpaced(angleRangeEnd - angleRangeStart + 1, angleRangeStart, angleRangeEnd);
    // std::cout << "All angles: \n" << allAngles << "\n";
    // Create a boolean mask for completed angles
    // Checking the size
    Eigen::MatrixXd result = angleFlex.array().replicate(1, allAngles.size());
    Eigen::MatrixXd zwischen = (result.rowwise() - allAngles.transpose());
    Eigen::Matrix<bool, Eigen::Dynamic, Eigen::Dynamic> completedMask = (result.rowwise() - allAngles.transpose()).cwiseAbs().array() < 1;
    Eigen::Vector<bool, Eigen::Dynamic> boolValidAngles = completedMask.colwise().any();
    int rows = result.rows();
    int cols = result.cols();

    // std::cout << "All Angles: " << allAngles.transpose() << "\n"; 
    std::cout << "angleFlex: " << angleFlex.transpose() << "\n"; 
    std::cout << "Shape: " << rows << " x " << cols << std::endl; 

    // Find uncompleted angles
    std::vector<double> uncompletedAngles;
    printf("Uncomp angles: \n");
    for (int i=0; i<completedMask.cols(); i++){
        if (boolValidAngles(i)==false) {
            uncompletedAngles.push_back(allAngles(i));
            std::cout << allAngles(i) << "\t"; 
        }
    }
    printf("\n");
    // Check if there are uncompleted angles left
    double newAngle; 
    if (!uncompletedAngles.empty()) {
        // Select a random angle from the uncompleted angles
        int randomIndex = std::rand() % uncompletedAngles.size();
        newAngle = uncompletedAngles[randomIndex];
    } else {
        printf("No uncompleted angles available");
    }
    std::cout << "New angle is: " << newAngle << std::endl; 
/*    std::vector<int> completedAngles = {10, 15, 25, 30};  // Example completed angles
    int tolerance = 5;  // Degrees of tolerance

    int newAngle = generateRandomAngle(completedAngles, tolerance, 0, 40);
    
    if (newAngle != -1) {
        std::cout << "New random angle: " << newAngle << std::endl;
    } else {
        std::cout << "No uncompleted angles available." << std::endl;
    }
*/
    return 0;
}
