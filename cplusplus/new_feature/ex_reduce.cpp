#if PARALLEL
#include <execution>
#define SEQ std::execution::seq,
#define PAR std::execution::par,
#else
#define SEQ
#define PAR
#endif
 
#include <chrono>
#include <iomanip>
#include <iostream>
#include <numeric>
#include <utility>
#include <vector>
 
int main()
{
    // std::cout.imbue(std::locale("en_US.UTF-8"));
    std::cout << std::fixed << std::setprecision(1);
    auto eval = [](auto fun) {
        const auto t1 = std::chrono::high_resolution_clock::now();
        const auto [name, result] = fun();
        const auto t2 = std::chrono::high_resolution_clock::now();
        const std::chrono::duration<double, std::milli> ms = t2 - t1;
        std::cout << std::setw(28) << std::left << name << "sum: "
                  << result << "\t time: " << ms.count() << " ms\n";
    };
    {
        const std::vector<double> v(100'000'007, 0.1);
 
        eval([&v]{ return std::pair{"std::accumulate (double)",
            std::accumulate(v.cbegin(), v.cend(), 0.0)}; } );
        eval([&v]{ return std::pair{"std::reduce (seq, double)",
            std::reduce(SEQ v.cbegin(), v.cend())}; } );
        eval([&v]{ return std::pair{"std::reduce (par, double)",
            std::reduce(PAR v.cbegin(), v.cend())}; } );
    }
    {
        const std::vector<long> v(100'000'007, 1);
 
        eval([&v]{ return std::pair{"std::accumulate (long)",
            std::accumulate(v.cbegin(), v.cend(), 0)}; } );
        eval([&v]{ return std::pair{"std::reduce (seq, long)",
            std::reduce(SEQ v.cbegin(), v.cend())}; } );
        eval([&v]{ return std::pair{"std::reduce (par, long)",
            std::reduce(PAR v.cbegin(), v.cend())}; } );
    }
}
// refer to https://en.cppreference.com/w/cpp/algorithm/reduce

// g++ -std=c++2a ./ex_reduce.cpp -O3 -o bin_output_SEQ; ./bin_output_SEQ
// The next command need dynamic lib of ltbb
// g++ -std=c++2a ./ex_reduce.cpp -ltbb -O3 -DPARALLEL -o bin_output_PAR; ./bin_output_PAR