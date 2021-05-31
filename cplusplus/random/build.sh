# reference to https://www.jianshu.com/p/e5c6a255076b
rm -rf bin_output_random_number_generation
g++ -std=c++11 -c random_common_util.cpp -o random_common_util.o
g++ -std=c++11 random_number_generation.cpp -o bin_output_random_number_generation -I ./ random_common_util.o
rm -rf random_common_util.o
