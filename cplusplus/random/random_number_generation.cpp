#include <algorithm>
#include <array>
#include <iostream>
#include <numeric>

#include "random_common_util.h"
#define DEBUG

int main( ) {
    // Manufacture a deck of cards:
    using card = int;
    std::array<card, 52>  deck{};
    std::iota(deck.begin(), deck.end(), 0);
    // Shuffle the deck:
    randomize();
    std::shuffle(deck.begin(), deck.end(), global_urng());
#ifdef DEBUG
    for (card const & c : deck) {
        std::cout << c << ", ";
    }
    std::cout << "\n";
#endif
    // Display each card in the shuffled deck:
    auto suit = [](card c) { return "SHDC"[c / 13]; };
    auto rank = [](card c) { return "AKQJT98765432"[c % 13];};
    for( card const & c : deck ) {
        std::cout << ' ' << rank(c) << suit(c);
    }
    std::cout << std::endl;
    return 0;
}
