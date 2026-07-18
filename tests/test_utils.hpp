#pragma once

#include <cmath>
#include <iostream>
#include <string_view>

inline int test_failure(std::string_view expression, std::string_view file, int line) {
    std::cerr << file << ':' << line << ": requirement failed: " << expression << '\n';
    return 1;
}

#define TEST_REQUIRE(expression)                                      \
    do {                                                              \
        if (!(expression)) {                                          \
            return test_failure(#expression, __FILE__, __LINE__);     \
        }                                                             \
    } while (false)

inline bool near(double actual, double expected, double tolerance = 1e-5) {
    return std::abs(actual - expected) <= tolerance;
}
