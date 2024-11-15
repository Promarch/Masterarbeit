#include <iostream>
#include <fstream>
#include <sstream>
#include <vector>
#include <array>
#include <string>
#include <utility> // for std::pair
#include <algorithm> // for remove if

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

template <typename T>
void printVector(const std::vector<T>& vector) {
    for (int i=0; i<vector.size(); i++){
        std::cout << vector[i] << "\t";
    }
    printf("\n");
}

std::pair<int, int> getTwoInts() {
    int a = 10;
    int b = 20;
    return std::make_pair(a, b);
}


int main() {
    auto [first, second] = getTwoInts();
    int test = first;
    std::cout << "First: " << test << ", Second: " << second << '\n';

    std::vector<int> vec = {0,1,2,3,4};
    std::vector<int> v_erase = {0,1,1,0,0};

    // Erase elements from `vec` where the corresponding value in `v_erase` is 1
    auto it = std::remove_if(vec.begin(), vec.end(),
        [&v_erase, i = 0](int) mutable { 
            return v_erase[i++] == 1; 
        });

    // Remove the elements (actually shrinking the vector)
    vec.erase(it, vec.end());
    printf("Chat GPT: "); printVector(vec);

    return 0;
}
