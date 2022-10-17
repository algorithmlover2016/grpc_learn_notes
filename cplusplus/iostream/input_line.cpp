#include <iostream>
#include <sstream>
#include <fstream>
#include <string>

int main() {
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
                    int num = std::stof(str);
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