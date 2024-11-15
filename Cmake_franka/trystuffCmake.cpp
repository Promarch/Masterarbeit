#include <iostream>
#include <iomanip>  // For set print decimal precision
#include <fstream>  // Read files
#include <sstream>
#include <vector>
#include <cstdlib>  // for rand() and srand()
#include <ctime>    // for time()
#include <utility>
#include <Eigen/Core>
#include <Eigen/Dense>


// Function to read in quaternion file
Eigen::MatrixXd readFileToMatrix(const std::string& filename) {
    std::vector<std::array<double, 4>> data;
    data.reserve(20);
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

std::vector<std::array<double, 4>> readFileToVector(const std::string& filename) {
    std::vector<std::array<double, 4>> data;
    data.reserve(20);
    std::ifstream file(filename);

    if (!file) {
        std::cerr << "Error: Unable to open file " << filename << std::endl;
        return data; // Return an empty matrix if the file cannot be opened
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

    return data;
}

/*
// Function to generate a random angle
std::pair<std::vector<std::array<int, 2>>,std::vector<std::array<int, 2>>> generateValidAngles_slow(const Eigen::MatrixXd &quat_stop, int angleRangeStart, int angleRangeEnd, int tolerance) {
    // Create evenly spaced possible angles
    Eigen::VectorXi allAngles = Eigen::VectorXi::LinSpaced(angleRangeEnd - angleRangeStart + 1, angleRangeStart, angleRangeEnd);
 
    // Calculate flexion and internal from quaternion
    Eigen::VectorXd angleFlex = 2 * (quat_stop.col(0).array() / quat_stop.col(3).array()).atan() *180/M_PI;
    Eigen::VectorXd angleIntern = 2 * (quat_stop.col(1).array() / quat_stop.col(3).array()).atan() *180/M_PI;

    for (size_t i=0; i<angleFlex.size(); i++) {

    }
}*/

// Function to generate a random angle
std::pair<std::vector<std::array<int, 2>>,std::vector<std::array<int, 2>>> generateValidAngles(const Eigen::MatrixXd &quat_stop, int angleRangeStart, int angleRangeEnd, int tolerance) {
    // Create evenly spaced possible angles
    Eigen::VectorXi allAngles = Eigen::VectorXi::LinSpaced(angleRangeEnd - angleRangeStart + 1, angleRangeStart, angleRangeEnd);
 
    // Create meshgrid of flexion angles and allAngles
    Eigen::VectorXd angleFlex = 2 * (quat_stop.col(0).array() / quat_stop.col(3).array()).atan() *180/M_PI;
    Eigen::VectorXd angleIntern = 2 * (quat_stop.col(1).array() / quat_stop.col(3).array()).atan() *180/M_PI;
    Eigen::MatrixXd result = angleFlex.array().replicate(1, allAngles.size());
    // Check which angles have been reached
    Eigen::MatrixXd zwischen = (result.rowwise() - allAngles.cast<double>().transpose());
    Eigen::Matrix<bool, Eigen::Dynamic, Eigen::Dynamic> completedMask = (result.rowwise() - allAngles.cast<double>().transpose()).cwiseAbs().array() < tolerance;
    // Multiply by the sign of the internal-external to know where the flexion was reached
    Eigen::MatrixXi boolMatrixFiltered = completedMask.cast<int>().array().colwise() * angleIntern.array().sign().cast<int>();
    // Get the index of the valid angles
    Eigen::Vector<bool, Eigen::Dynamic> boolValidAnglesIntern = !(boolMatrixFiltered.array()<0).colwise().any();
    Eigen::Vector<bool, Eigen::Dynamic> boolValidAnglesExtern = !(boolMatrixFiltered.array()>0).colwise().any();

    // This is only here for debugging
    int rows = result.rows();
    int cols = result.cols();

    std::cout << "Bool Matrix: \n" << boolMatrixFiltered << "\n"; 
    std::cout << "Valid int: " << boolValidAnglesIntern.transpose() << "\n"; 
    std::cout << "Valid ext: " << boolValidAnglesExtern.transpose() << "\n"; 
    // std::cout << "multiplication tests: \n" << completedMask.cast<double>().array().colwise() * angleIntern.array().sign() << "\n";
    std::cout << "angleFlex: " << angleFlex.transpose() << "\n"; 
    std::cout << "angleInt : " << angleIntern.transpose() << "\n"; 
    std::cout << "Shape: " << rows << " x " << cols << "\n\n"; 

    // Add the valid angles to a vector
    std::vector<std::array<int, 2>> validAnglesIntern, validAnglesExtern;
    for (int i=0; i<completedMask.cols(); i++){
        if (boolValidAnglesIntern(i)==true) {
            auto it = std::find(completedMask.col(i).array().begin(), completedMask.col(i).array().begin(), -1);
            // Calculate the index of the element pointed to by the iterator
            size_t index = std::distance(completedMask.col(i).array().begin(), it);
            int help = round(angleIntern(index));  // Access the corresponding element in angleIntern
            std::array<int, 2> validTemp = {help, allAngles[i]};
            validAnglesIntern.push_back(validTemp);
        }
        if (boolValidAnglesExtern(i)==true) {
            auto it = std::find(completedMask.col(i).array().begin(), completedMask.col(i).array().begin(), 1);
            // Calculate the index of the element pointed to by the iterator
            size_t index = std::distance(completedMask.col(i).array().begin(), it);
            int help = round(angleIntern(index));  // Access the corresponding element in angleIntern
            std::array<int, 2> validTemp = {help, allAngles[i]};
            validAnglesExtern.push_back(validTemp);
        }
    }

    return {validAnglesIntern, validAnglesExtern};
}

template <typename T>
void printVector(const std::vector<T>& vector) {
    for (int i=0; i<vector.size(); i++){
        std::cout << vector[i] << "\t";
    }
    printf("\n");
}


void printArray(const std::vector<std::array<int, 2>>& vec) {
    for (const auto& elem : vec) {
        std::cout << "(" << elem[0] << ", " << elem[1] << ")" << std::endl;
    }
    printf("\n");
}


int main() {
    // Seed the random number generator
    std::srand(std::time(nullptr)); // initialize random seed
    std::cout << std::fixed << std::setprecision(3);


    // Get vector
    std::string filename = "/home/alexandergerard/Documents/quat_stop_tests3.txt";
    // std::string filename = "build/data_output_knee/quat_stop_data_20241107_112356.txt";
    std::vector<std::array<double, 4>> quat_stop_data = readFileToVector(filename);
    Eigen::MatrixXd quat_stop = readFileToMatrix(filename);

    // Generate valid flexion angles for subsequent internal and external rotation
    int tolerance = 2;
    std::pair<std::vector<std::array<int, 2>>, std::vector<std::array<int, 2>>> result = generateValidAngles(quat_stop, -30, -10, 2);
    std::vector<std::array<int, 2>> validAnglesIntern = result.first;
    std::vector<std::array<int, 2>> validAnglesExtern = result.second;
    // Print these angles
    printf("ValidIntern: ");
    printArray(validAnglesIntern);
    printf("ValidExtern: ");
    printArray(validAnglesExtern);

    // Simulate choosing a new entry
    // std::array<double, 4> last_quat = quat_stop_data.back();
    std::array<double, 4> last_quat = {-0.189, -0.137, 0.0266, 0.9721};
    int lastFlexion = round(2*atan(last_quat[0]/last_quat[3]) *180/M_PI);
    bool isInternal = (2*atan(last_quat[1]/last_quat[3]) *180/M_PI)<0;
    std::cout << "Last flexion: " << lastFlexion << ", is internal? " << isInternal << "\n";

    return 0;
}
