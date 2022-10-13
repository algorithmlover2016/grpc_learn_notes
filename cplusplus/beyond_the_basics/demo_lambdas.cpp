#include <vector>
#include <string>
#include <algorithm>
#include <memory>
#include <iostream>
#include "resource.h"

int main() {
    auto isOdd = [] (int para) {return para % 2 != 0;};
    std::vector<int> nums{2, 3, 4, -6, 1, -1};
    int odds = std::count_if(std::begin(nums), std::end(nums), isOdd);
    int evens = std::count_if(std::begin(nums), std::end(nums), [] (int para) {return (para & 0x01) == 0;});
    std::cout << "odds: " << odds << "; evens: " << evens << "\n";
    for_each(std::begin(nums), std::end(nums), [] (int n) {
                                                std::cout << n << ": " << (n & 0x01) << ";";
                                                });
    std::cout << "\n";

    int x = 3, y = 7;
    std::string message = "elements between ";
    message += std::to_string(x) + " and " + std::to_string(y) + " inclusive: ";
    std::for_each(std::begin(nums), std::end(nums), [x, y, &message] (int n) {
        if (n >= x && n <= y) {
            message += " " + std::to_string(n);
        }
    });
    std::cout << message << "\n";

    x = y = 0;
    std::for_each(std::begin(nums), std::end(nums), [&, x] (int ele) mutable {
        x += ele;
        y += ele;
        std::cout << x << ",\t";
    });
    std::cout << "sum for nums is x: " << x << ", y: " << y << "\n";

    { // for sync call
        auto pResource = std::make_unique<Resource>(", ");
        std::for_each(std::begin(nums), std::end(nums), [=, &message, &pResource] (int n) {
            if (n >= x && n <= y) {
                message += pResource->GetName() + std::to_string(n);
            }
        });
        std::cout << message << "\n";
        ;
    }
    { // for async
        auto pResource = std::make_unique<Resource>(", ");
        std::for_each(std::begin(nums), std::end(nums), [=, &message, p = std::move(pResource)] (int n) {
            if (n >= x && n <= y) {
                message += p->GetName() + std::to_string(n);
            }
        });
        std::cout << message << "\n";
        ;
    }
    nums.erase(std::remove_if(std::begin(nums), std::end(nums), [](int n) {return (n == -6);}), std::end(nums));
    std::for_each(std::begin(nums), std::end(nums), [](int n) {std::cout << n << ", ";});
    return 0;
}

// g++ -std=c++14 ./demo_lambdas.cpp resource.cpp -I./ -o bin_out