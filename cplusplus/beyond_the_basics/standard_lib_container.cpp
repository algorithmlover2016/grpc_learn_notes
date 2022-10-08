#include<chrono>
#include<vector>
#include<list>
#include<algorithm>
#include<iostream>

using std::chrono::system_clock;
using std::chrono::steady_clock;
using std::chrono::duration_cast;
using std::chrono::seconds;
using std::chrono::milliseconds;



template<typename Func>
long long TimeFunc(Func f) {
    auto begin = steady_clock::now();
    f();
    auto end = steady_clock::now();
    return duration_cast<milliseconds>(end - begin).count();
}

void buildNonDescVector(int size, std::vector<int> & v) {
    for (int i = 0; i < size; i++) {
        int r = (int)rand();
        bool inserted = false;
        for (auto it = v.begin(); it != v.end(); it++) {
            if (*it > r) {
                v.insert(it, r);
                inserted = true;
                break;
            }
        }
        if (!inserted) {
            v.emplace_back(r);
        }
    }
}

void buildNonDescVectorByBS(int size, std::vector<int> & v) {
    for (int i = 0; i < size; i++) {
        int r = (int)rand();
        auto it = std::upper_bound(std::begin(v), std::end(v), r);
        v.insert(it, r);
    }
}

void buildNonDescList(int size, std::list<int> & v) {
    for (int i = 0; i < size; i++) {
        int r = (int)rand();
        bool inserted = false;
        for (auto it = v.begin(); it != v.end(); it++) {
            if (*it > r) {
                v.insert(it, r);
                inserted = true;
                break;
            }
        }
        if (!inserted) {
            v.emplace_back(r);
        }
    }
}

int main() {
    int const size = 100000;
    std::vector<int> v;
    v.push_back(0);
    std::vector<int> vBs;
    vBs.push_back(0);
    std::list<int> l;
    l.push_back(0);
    auto vMilliseconds = TimeFunc([&](){buildNonDescVector(size, v);});
    auto vBsMilliseconds = TimeFunc([&](){buildNonDescVectorByBS(size, vBs);});
    auto lMilliseconds = TimeFunc([&](){buildNonDescList(size, l);});
    std::cout << "vec Loop: " << vMilliseconds << ", vec Bs: " << vBsMilliseconds << ", list Loop: " << lMilliseconds << std::endl;

    return 0;
}
// g++ -std=c++11 -o bin_output ./standard_lib_container.cpp