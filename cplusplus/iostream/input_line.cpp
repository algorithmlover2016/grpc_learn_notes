#include <iostream>
#include <sstream>
#include <fstream>
#include <string>
#include <limits>
// #include <type_traits>
#include <cmath>
template <typename T>
typename std::enable_if<!std::numeric_limits<T>::is_integer, bool>::type
IsEqual(T x, T y, int ulp = 2) {
    return std::fabs(x - y) < std::numeric_limits<T>::epsilon() * std::abs(x + y) * ulp ||
            std::fabs(x - y) < std::numeric_limits<T>::min();
}


int main() {
    std::cout << "Double equal judge (3.2, 3.3): " << IsEqual<double>(3.2, 3.3) << "\n";
    std::cout << "Double equal judge (0.2, 0.2): " << IsEqual<double>(0.2, 0.2) << "\n";
    std::cout << "Double equal judge (0.2, -0.2): " << IsEqual<double>(0.2, -0.2) << "\n";

    std::string fileName("./input.txt");

    std::ifstream inputFile(fileName);
    if (!inputFile.is_open()) {
        std::cout << "open [" << fileName << "] encounter error" << "\n";
        return -1;
    }
    std::string inStrLine;
    while (std::getline(inputFile, inStrLine)) {
        if (inStrLine.empty()) {
            continue;
        }
        std::cout << "input [" << inStrLine << "]\n";
        std::stringstream ss(inStrLine);
        std::string str;
        while (ss >> str) {
            try {
                if (":" == str) {
                    std::cout << str;

                } else {
                    float num = std::stof(str);
                    std::cout << num << ",\t";
                }
            } catch (std::invalid_argument const & ex) {
                std::cout << "\nTo float [" << str << "] encounter " << ex.what() << "\n";
            } catch (std::out_of_range const & ex) {
                std::cout << "\nTo float [" << str << "] encounter " << ex.what() << "\n";
                continue;
            } catch (...) {
                std::exception_ptr ptr = std::current_exception();
            }
        }
        std::cout << "\n";
    }
    return 0;
}
// g++ -std=c++11 ./input_line.cpp -o bin_output