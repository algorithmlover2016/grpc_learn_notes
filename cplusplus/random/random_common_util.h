#ifndef RANDOM_RANDOM_COMMON_UTIL_H
#define RANDOM_RANDOM_COMMON_UTIL_H
#include <random>
std::default_random_engine & global_urng();
void randomize(); 
int pick_a_number(int from, int thru); 
double pick_a_number(double from, double upto); 
#endif
