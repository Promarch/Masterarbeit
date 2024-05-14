#include <iostream>
#include <algorithm>
#include <array>

int main () {

    double testnumber = 5;
    std::array<double, 7> initial_position = {0.23, -0.75, 0.72, -2.63, 0, 1.86, 1.21};
    std::array<double, 7> modified_position = {-0.32, -0.23, -0.33, -2.68, 0.09, 2.27, 0.43};
    std::array<double, 7> diff_position;
    
    std::transform(modified_position.begin(), modified_position.end(), initial_position.begin(), diff_position.begin(), std::minus<double>() );

    for (double number : diff_position) {
        std::cout << number << " ";
    }
    std::cout << "\n";

}