/**
 * @file common_types.cpp
 * @brief Implementation of common types
 */

#include "common_types.hpp"
#include <cmath>

namespace hybrid_astar {

bool State::operator==(const State& other) const {
    const double tolerance = 0.1;
    return (std::abs(x - other.x) < tolerance &&
            std::abs(y - other.y) < tolerance &&
            std::abs(yaw - other.yaw) < tolerance);
}

} // namespace hybrid_astar
