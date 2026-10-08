#ifndef PATHPLANNING_NATIVE_REEDS_SHEPP_HPP_
#define PATHPLANNING_NATIVE_REEDS_SHEPP_HPP_

#include <array>
#include <vector>

namespace pathplanning::kinodynamic {

struct ReedsSheppSegment {
    int steering;  // -1 right, 0 straight, +1 left
    int gear;      // -1 reverse, +1 forward
    double length;
};

bool shortest_reeds_shepp_path(const std::array<double, 3>& start,
                               const std::array<double, 3>& goal,
                               double turning_radius,
                               std::vector<ReedsSheppSegment>* output);

}  // namespace pathplanning::kinodynamic

#endif  // PATHPLANNING_NATIVE_REEDS_SHEPP_HPP_
