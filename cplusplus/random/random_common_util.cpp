#include <random>
#include "random_common_util.h"

std::default_random_engine & global_urng() {
    static std::default_random_engine u{};
    return u;
}

void randomize() {
    static std::random_device  rd{};
    global_urng().seed(rd());
}

int pick_a_number(int from, int thru) {
    static std::uniform_int_distribution<>  d{};
    using  parm_t = decltype(d)::param_type;
    return d(global_urng(), parm_t{from, thru});
}

double pick_a_number(double from, double upto) {
    static std::uniform_real_distribution<> d{};
    using  parm_t = decltype(d)::param_type;
    return d(global_urng(), parm_t{from, upto});
}
