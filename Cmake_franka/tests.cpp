#include <iostream>
#include <iomanip>
#include <fstream>
#include <algorithm>
#include <array>
#include <vector>


// Helper function to linearly interpolate between two positions
std::array<double, 7> interpolate(const std::array<double, 7>& start, const std::array<double, 7>& end, double t) {
  std::array<double, 7> result;
  for (size_t i = 0; i < start.size(); ++i) {
    result[i] = t * (end[i] - start[i]); //start[i] + 
  }
  return result;
}

int main () {


    double testnumber = 5;
    std::array<double, 7> initial_position = {0.23, -0.75, 0.72, -2.63, 0, 1.86, 1.21};
    std::array<double, 7> pos1 = {-0.32, -0.23, -0.33, -2.68, 0.09, 2.27, 0.43};
    std::array<double, 7> diff_position;
    std::vector<std::array<double, 7>> interpolValues;


    for (size_t i=0; i<200; i++) {
        double t = i;
        diff_position = interpolate(initial_position, pos1, t/200);
        interpolValues.push_back(diff_position);
    }

    std::ofstream data_file("interpolValues.txt");
    if (data_file.is_open()) {
        data_file << std::fixed << std::setprecision(4);
        for (const auto& sample : interpolValues) {
        for (size_t i = 0; i < sample.size(); ++i) {
            data_file << sample[i];
            if (i < sample.size() - 1) {
            data_file << ", ";
            }
        }
        data_file << "\n";
        }
        data_file.close();
    } else {
        std::cerr << "Unable to open file for writing" << std::endl;
    }
    
//    std::transform(pos1.begin(), pos1.end(), initial_position.begin(), diff_position.begin(), std::minus<double>() );

//    for (double number : diff_position) {
//        std::cout << number << " ";
//    }
//    std::cout << "\n";

}