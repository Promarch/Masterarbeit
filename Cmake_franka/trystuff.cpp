#include <iostream>
#include <fstream>
#include <sstream>
#include <vector>
#include <array>
#include <string>

std::vector<std::array<double, 4>> readFileToVector(const std::string& filename) {
    std::vector<std::array<double, 4>> data;
    std::ifstream file(filename);

    if (!file) {
        std::cerr << "Error: Unable to open file " << filename << std::endl;
        return data; // Return an empty vector if the file cannot be opened
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

int main() {
    std::string filename = "build/data_output_knee/quat_stop_data_20241028_220631.txt";
    std::vector<std::array<double, 4>> data = readFileToVector(filename);

    // Output the data to verify
    for (const auto& row : data) {
        for (const double value : row) {
            std::cout << value << " ";
        }
        std::cout << std::endl;
    }

    return 0;
}
