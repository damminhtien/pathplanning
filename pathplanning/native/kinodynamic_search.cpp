#include "kinodynamic_search.h"
#include "reeds_shepp.hpp"

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdlib>
#include <cstring>
#include <functional>
#include <limits>
#include <queue>
#include <stdexcept>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

namespace {

constexpr double kPi = 3.141592653589793238462643383279502884;
constexpr double kTwoPi = 2.0 * kPi;
constexpr double kInfinity = std::numeric_limits<double>::infinity();

struct Pose {
    double x;
    double y;
    double yaw;
};

struct Segment {
    int gear;
    double steering;
    double length;
};

struct Key {
    int64_t x;
    int64_t y;
    uint64_t heading;
    int gear;

    bool operator==(const Key& other) const noexcept {
        return x == other.x && y == other.y && heading == other.heading && gear == other.gear;
    }
};

struct KeyHash {
    size_t operator()(const Key& key) const noexcept {
        size_t seed = std::hash<int64_t>{}(key.x);
        seed ^= std::hash<int64_t>{}(key.y) + 0x9e3779b97f4a7c15ULL + (seed << 6) + (seed >> 2);
        seed ^= std::hash<uint64_t>{}(key.heading) + 0x9e3779b97f4a7c15ULL + (seed << 6) + (seed >> 2);
        seed ^= std::hash<int>{}(key.gear) + 0x9e3779b97f4a7c15ULL + (seed << 6) + (seed >> 2);
        return seed;
    }
};

struct Node {
    Pose pose{};
    double cost = kInfinity;
    size_t parent = std::numeric_limits<size_t>::max();
    int gear = 0;
    double steering = 0.0;
    double primitive_length = 0.0;
    bool analytic = false;
    std::vector<Segment> analytic_segments;
};

struct QueueItem {
    double priority;
    double heuristic;
    double cost;
    size_t node;
};

struct QueueGreater {
    bool operator()(const QueueItem& first, const QueueItem& second) const noexcept {
        if (first.priority != second.priority) return first.priority > second.priority;
        if (first.heuristic != second.heuristic) return first.heuristic > second.heuristic;
        return first.node > second.node;
    }
};

struct DubinsCandidate {
    std::array<int, 3> turns;
    std::array<double, 3> lengths;
};

struct TraceBuffer {
    explicit TraceBuffer(uint64_t max_bytes) {
        max_points = static_cast<size_t>(max_bytes / (sizeof(pp_trace_event) + 3 * sizeof(double)));
        const uint64_t point_bytes = static_cast<uint64_t>(max_points) * 3 * sizeof(double);
        max_events = static_cast<size_t>((max_bytes - point_bytes) / sizeof(pp_trace_event));
    }

    void add_point(const Pose& pose) {
        if (points.size() >= max_points) {
            truncated = true;
            return;
        }
        points.push_back(pose);
    }

    void add_event(uint64_t node, uint64_t parent, double value, uint32_t kind) {
        if (node >= points.size()) {
            truncated = true;
            return;
        }
        if (events.size() >= max_events) {
            truncated = true;
            return;
        }
        events.push_back({node, parent, value, kind, 0});
    }

    std::vector<Pose> points;
    std::vector<pp_trace_event> events;
    size_t max_points = 0;
    size_t max_events = 0;
    bool truncated = false;
};

double monotonic_seconds() {
    using Clock = std::chrono::steady_clock;
    return std::chrono::duration<double>(Clock::now().time_since_epoch()).count();
}

double wrap_yaw(double angle) {
    double value = std::fmod(angle + kPi, kTwoPi);
    if (value < 0.0) value += kTwoPi;
    return value - kPi;
}

double positive_mod2pi(double angle) {
    double value = std::fmod(angle, kTwoPi);
    if (value < 0.0) value += kTwoPi;
    return value;
}

double yaw_distance(double first, double second) {
    return std::abs(wrap_yaw(first - second));
}

Pose integrate(const Pose& initial, int gear, double steering, double distance,
               double wheelbase) {
    Pose result = initial;
    const double curvature = std::tan(steering) / wheelbase;
    const double yaw_delta = static_cast<double>(gear) * curvature * distance;
    if (std::abs(curvature) < 1e-12) {
        result.x += static_cast<double>(gear) * distance * std::cos(initial.yaw);
        result.y += static_cast<double>(gear) * distance * std::sin(initial.yaw);
    } else {
        const double next_yaw = initial.yaw + yaw_delta;
        result.x += (std::sin(next_yaw) - std::sin(initial.yaw)) / curvature;
        result.y += (std::cos(initial.yaw) - std::cos(next_yaw)) / curvature;
    }
    result.yaw = wrap_yaw(initial.yaw + yaw_delta);
    return result;
}

bool overlaps_occupied_cell(const Pose& pose, double half_length, double half_width,
                            double cell_x, double cell_y, double resolution) {
    const double cosine = std::cos(pose.yaw);
    const double sine = std::sin(pose.yaw);
    const double half_cell = 0.5 * resolution;
    const double dx = cell_x - pose.x;
    const double dy = cell_y - pose.y;
    const double cell_projection = half_cell * (std::abs(cosine) + std::abs(sine));
    if (std::abs(dx) > half_length * std::abs(cosine) + half_width * std::abs(sine) + half_cell ||
        std::abs(dy) > half_length * std::abs(sine) + half_width * std::abs(cosine) + half_cell) {
        return false;
    }
    const double along_length = std::abs(dx * cosine + dy * sine);
    const double along_width = std::abs(-dx * sine + dy * cosine);
    return along_length <= half_length + cell_projection &&
           along_width <= half_width + cell_projection;
}

bool grid_state_valid(const pp_hybrid_astar_space& space, const Pose& pose) {
    const double half_length = 0.5 * space.footprint_length;
    const double half_width = 0.5 * space.footprint_width;
    const double cosine = std::cos(pose.yaw);
    const double sine = std::sin(pose.yaw);
    const double extent_x = half_length * std::abs(cosine) + half_width * std::abs(sine);
    const double extent_y = half_length * std::abs(sine) + half_width * std::abs(cosine);
    const double world_max_x = space.origin_x + static_cast<double>(space.grid_width) * space.grid_resolution;
    const double world_max_y = space.origin_y + static_cast<double>(space.grid_height) * space.grid_resolution;
    if (pose.x - extent_x < space.origin_x || pose.y - extent_y < space.origin_y ||
        pose.x + extent_x > world_max_x || pose.y + extent_y > world_max_y) {
        return false;
    }
    const int64_t min_x = static_cast<int64_t>(std::floor((pose.x - extent_x - space.origin_x) / space.grid_resolution));
    const int64_t max_x = std::min(
        static_cast<int64_t>(space.grid_width) - 1,
        static_cast<int64_t>(std::floor((pose.x + extent_x - space.origin_x) / space.grid_resolution)));
    const int64_t min_y = static_cast<int64_t>(std::floor((pose.y - extent_y - space.origin_y) / space.grid_resolution));
    const int64_t max_y = std::min(
        static_cast<int64_t>(space.grid_height) - 1,
        static_cast<int64_t>(std::floor((pose.y + extent_y - space.origin_y) / space.grid_resolution)));
    for (int64_t y = min_y; y <= max_y; ++y) {
        for (int64_t x = min_x; x <= max_x; ++x) {
            const size_t index = static_cast<size_t>(y) * static_cast<size_t>(space.grid_width) +
                                 static_cast<size_t>(x);
            if (space.occupancy[index] == 0) continue;
            const double center_x = space.origin_x + (static_cast<double>(x) + 0.5) * space.grid_resolution;
            const double center_y = space.origin_y + (static_cast<double>(y) + 0.5) * space.grid_resolution;
            if (overlaps_occupied_cell(pose, half_length, half_width, center_x, center_y,
                                       space.grid_resolution)) {
                return false;
            }
        }
    }
    return true;
}

int state_is_valid(const pp_hybrid_astar_space& space, const Pose& pose,
                   uint64_t* motion_checks) {
    if (motion_checks != nullptr) ++*motion_checks;
    if (space.occupancy != nullptr) return grid_state_valid(space, pose) ? 1 : 0;
    if (space.state_valid == nullptr) return -1;
    const double values[3] = {pose.x, pose.y, pose.yaw};
    const int valid = space.state_valid(space.user_data, values, 3);
    return valid < 0 || valid > 1 ? -1 : valid;
}

bool motion_is_valid(const pp_hybrid_astar_space& space, const Pose& start, int gear,
                     double steering, double length, double collision_step,
                     uint64_t* checks, Pose* endpoint) {
    const size_t steps = std::max<size_t>(1, static_cast<size_t>(std::ceil(length / collision_step)));
    Pose current = start;
    for (size_t step = 1; step <= steps; ++step) {
        current = integrate(start, gear, steering, length * static_cast<double>(step) /
                            static_cast<double>(steps), space.wheelbase);
        const int valid = state_is_valid(space, current, checks);
        if (valid <= 0) return false;
    }
    if (endpoint != nullptr) *endpoint = current;
    return true;
}

void dubins_candidates(const Pose& start, const Pose& goal, double radius,
                       std::vector<DubinsCandidate>* output) {
    const double dx = goal.x - start.x;
    const double dy = goal.y - start.y;
    const double distance = std::hypot(dx, dy);
    const double direction = std::atan2(dy, dx);
    const double alpha = positive_mod2pi(start.yaw - direction);
    const double beta = positive_mod2pi(goal.yaw - direction);
    const double cosine_delta = std::cos(alpha - beta);
    const double sin_alpha = std::sin(alpha);
    const double sin_beta = std::sin(beta);
    const double cos_alpha = std::cos(alpha);
    const double cos_beta = std::cos(beta);
    const double d = distance / radius;
    double p_squared = 0.0;
    double temporary = 0.0;
    double t = 0.0;
    double p = 0.0;
    double q = 0.0;

    p_squared = 2.0 + d * d - 2.0 * cosine_delta + 2.0 * d * (sin_alpha - sin_beta);
    if (p_squared >= -1e-12) {
        p = std::sqrt(std::max(0.0, p_squared));
        temporary = std::atan2(cos_beta - cos_alpha, d + sin_alpha - sin_beta);
        t = positive_mod2pi(-alpha + temporary);
        q = positive_mod2pi(beta - temporary);
        output->push_back({{1, 0, 1}, {t * radius, p * radius, q * radius}});
    }

    p_squared = 2.0 + d * d - 2.0 * cosine_delta + 2.0 * d * (-sin_alpha + sin_beta);
    if (p_squared >= -1e-12) {
        p = std::sqrt(std::max(0.0, p_squared));
        temporary = std::atan2(cos_alpha - cos_beta, d - sin_alpha + sin_beta);
        t = positive_mod2pi(alpha - temporary);
        q = positive_mod2pi(-beta + temporary);
        output->push_back({{-1, 0, -1}, {t * radius, p * radius, q * radius}});
    }

    p_squared = -2.0 + d * d + 2.0 * cosine_delta + 2.0 * d * (sin_alpha + sin_beta);
    if (p_squared >= -1e-12) {
        p = std::sqrt(std::max(0.0, p_squared));
        temporary = std::atan2(-cos_alpha - cos_beta, d + sin_alpha + sin_beta) -
                    std::atan2(-2.0, p);
        t = positive_mod2pi(-alpha + temporary);
        q = positive_mod2pi(-beta + temporary);
        output->push_back({{1, 0, -1}, {t * radius, p * radius, q * radius}});
    }

    p_squared = -2.0 + d * d + 2.0 * cosine_delta - 2.0 * d * (sin_alpha + sin_beta);
    if (p_squared >= -1e-12) {
        p = std::sqrt(std::max(0.0, p_squared));
        temporary = std::atan2(cos_alpha + cos_beta, d - sin_alpha - sin_beta) -
                    std::atan2(2.0, p);
        t = positive_mod2pi(alpha - temporary);
        q = positive_mod2pi(beta - temporary);
        output->push_back({{-1, 0, 1}, {t * radius, p * radius, q * radius}});
    }

    temporary = (6.0 - d * d + 2.0 * cosine_delta + 2.0 * d * (sin_alpha - sin_beta)) / 8.0;
    if (std::abs(temporary) <= 1.0) {
        p = positive_mod2pi(kTwoPi - std::acos(std::clamp(temporary, -1.0, 1.0)));
        t = positive_mod2pi(alpha - std::atan2(cos_alpha - cos_beta, d - sin_alpha + sin_beta) +
                            0.5 * p);
        q = positive_mod2pi(alpha - beta - t + p);
        output->push_back({{1, -1, 1}, {t * radius, p * radius, q * radius}});
    }

    temporary = (6.0 - d * d + 2.0 * cosine_delta + 2.0 * d * (-sin_alpha + sin_beta)) / 8.0;
    if (std::abs(temporary) <= 1.0) {
        p = positive_mod2pi(kTwoPi - std::acos(std::clamp(temporary, -1.0, 1.0)));
        t = positive_mod2pi(-alpha - std::atan2(cos_alpha - cos_beta, d + sin_alpha - sin_beta) +
                            0.5 * p);
        q = positive_mod2pi(beta - alpha - t + p);
        output->push_back({{-1, 1, -1}, {t * radius, p * radius, q * radius}});
    }
}

std::vector<Segment> convert_candidate(const DubinsCandidate& candidate,
                                       const pp_hybrid_astar_space& space,
                                       int gear) {
    std::vector<Segment> segments;
    segments.reserve(3);
    for (size_t index = 0; index < 3; ++index) {
        const double virtual_steering = std::atan(
            static_cast<double>(candidate.turns[index]) * space.wheelbase /
            (space.wheelbase / std::tan(space.max_steering_angle))
        );
        const double steering = gear > 0 ? virtual_steering : -virtual_steering;
        if (candidate.lengths[index] > 1e-10) {
            segments.push_back({gear, steering, candidate.lengths[index]});
        }
    }
    return segments;
}

bool validate_connector(const pp_hybrid_astar_space& space, const Pose& start,
                        const Pose& goal, const std::vector<Segment>& segments,
                        const pp_hybrid_astar_options& options, uint64_t* motion_checks,
                        int initial_gear, double* path_cost) {
    Pose current = start;
    double cost = 0.0;
    int previous_gear = initial_gear;
    for (const Segment& segment : segments) {
        Pose endpoint{};
        if (!motion_is_valid(space, current, segment.gear, segment.steering,
                             segment.length, options.collision_step, motion_checks, &endpoint)) {
            return false;
        }
        const double steering_fraction = std::abs(segment.steering) /
                                         space.max_steering_angle;
        cost += segment.length * (segment.gear < 0 ? options.reverse_penalty : 1.0) *
                (1.0 + options.steering_penalty * steering_fraction);
        if (previous_gear != 0 && previous_gear != segment.gear) {
            cost += options.direction_switch_penalty;
        }
        current = endpoint;
        previous_gear = segment.gear;
    }
    if (std::hypot(current.x - goal.x, current.y - goal.y) > 1e-5 ||
        yaw_distance(current.yaw, goal.yaw) > 1e-5) {
        return false;
    }
    *path_cost = cost;
    return true;
}

std::vector<Segment> analytic_connector(const pp_hybrid_astar_space& space,
                                        const Pose& start, const Pose& goal,
                                        int initial_gear,
                                        const pp_hybrid_astar_options& options,
                                        uint64_t* motion_checks) {
    std::vector<Segment> best;
    double best_cost = kInfinity;
    if (options.allow_reverse != 0) {
        std::vector<pathplanning::kinodynamic::ReedsSheppSegment> reeds_shepp;
        if (pathplanning::kinodynamic::shortest_reeds_shepp_path(
                {start.x, start.y, start.yaw}, {goal.x, goal.y, goal.yaw},
                space.wheelbase / std::tan(space.max_steering_angle), &reeds_shepp)) {
            std::vector<Segment> segments;
            segments.reserve(reeds_shepp.size());
            for (const auto& segment : reeds_shepp) {
                segments.push_back({segment.gear,
                                    static_cast<double>(segment.steering) *
                                        space.max_steering_angle,
                                    segment.length});
            }
            if (validate_connector(space, start, goal, segments, options,
                                   motion_checks, initial_gear, &best_cost)) {
                best = std::move(segments);
            }
        }
    }
    const int last_gear = options.allow_reverse != 0 ? -1 : 1;
    for (int gear = 1; gear >= last_gear; gear -= 2) {
        Pose virtual_start = start;
        Pose virtual_goal = goal;
        if (gear < 0) {
            virtual_start.yaw = wrap_yaw(virtual_start.yaw + kPi);
            virtual_goal.yaw = wrap_yaw(virtual_goal.yaw + kPi);
        }
        std::vector<DubinsCandidate> candidates;
        const double radius = space.wheelbase / std::tan(space.max_steering_angle);
        dubins_candidates(virtual_start, virtual_goal, radius, &candidates);
        std::sort(candidates.begin(), candidates.end(), [](const auto& first, const auto& second) {
            const auto length = [](const DubinsCandidate& path) {
                return path.lengths[0] + path.lengths[1] + path.lengths[2];
            };
            return length(first) < length(second);
        });
        for (const DubinsCandidate& candidate : candidates) {
            auto segments = convert_candidate(candidate, space, gear);
            double cost = 0.0;
            if (validate_connector(space, start, goal, segments, options,
                                   motion_checks, initial_gear, &cost)) {
                if (cost < best_cost) {
                    best_cost = cost;
                    best = std::move(segments);
                }
            }
        }
    }
    return best;
}

Key make_key(const Pose& pose, int gear, double resolution, double origin_x,
             double origin_y, uint64_t heading_bins) {
    const double x = std::floor((pose.x - origin_x) / resolution);
    const double y = std::floor((pose.y - origin_y) / resolution);
    if (x < static_cast<double>(std::numeric_limits<int64_t>::min()) ||
        x > static_cast<double>(std::numeric_limits<int64_t>::max()) ||
        y < static_cast<double>(std::numeric_limits<int64_t>::min()) ||
        y > static_cast<double>(std::numeric_limits<int64_t>::max())) {
        throw std::invalid_argument("pose exceeds the supported discretization range");
    }
    const double angle = positive_mod2pi(pose.yaw + kPi);
    uint64_t heading = static_cast<uint64_t>(std::floor(angle / kTwoPi * heading_bins));
    if (heading >= heading_bins) heading = heading_bins - 1;
    return {static_cast<int64_t>(x), static_cast<int64_t>(y), heading, gear};
}

double heuristic(const Pose& pose, const Pose& goal, double radius) {
    const double euclidean = std::hypot(goal.x - pose.x, goal.y - pose.y);
    const double heading = radius * yaw_distance(goal.yaw, pose.yaw);
    return std::max(euclidean, heading);
}

void set_error(pp_hybrid_astar_result* result, const std::string& message) {
    if (result == nullptr || result->error_message != nullptr) return;
    result->error_message = static_cast<char*>(std::malloc(message.size() + 1));
    if (result->error_message != nullptr) {
        std::memcpy(result->error_message, message.c_str(), message.size() + 1);
    }
}

bool valid_input(const pp_hybrid_astar_space* space, const double* start,
                 const double* goal, const pp_hybrid_astar_options* options,
                 std::string* error) {
    if (space == nullptr || start == nullptr || goal == nullptr || options == nullptr) {
        *error = "space, start, goal, and options must not be null";
        return false;
    }
    if ((space->occupancy == nullptr && space->state_valid == nullptr) ||
        (space->occupancy != nullptr && (space->grid_width == 0 || space->grid_height == 0))) {
        *error = "provide either an occupancy grid or a state-validity callback";
        return false;
    }
    if (!std::isfinite(space->wheelbase) || space->wheelbase <= 0.0 ||
        !std::isfinite(space->max_steering_angle) || space->max_steering_angle <= 0.0 ||
        space->max_steering_angle >= 0.5 * kPi ||
        !std::isfinite(space->footprint_length) || space->footprint_length <= 0.0 ||
        !std::isfinite(space->footprint_width) || space->footprint_width <= 0.0) {
        *error = "vehicle geometry requires positive finite dimensions and steering limits";
        return false;
    }
    if (space->occupancy != nullptr &&
        (!std::isfinite(space->grid_resolution) || space->grid_resolution <= 0.0 ||
         !std::isfinite(space->origin_x) || !std::isfinite(space->origin_y) ||
         space->grid_width > std::numeric_limits<size_t>::max() / space->grid_height)) {
        *error = "occupancy grid dimensions and geometry are invalid";
        return false;
    }
    for (size_t axis = 0; axis < 3; ++axis) {
        if (!std::isfinite(start[axis]) || !std::isfinite(goal[axis])) {
            *error = "start and goal poses must contain finite values";
            return false;
        }
    }
    if (options->max_expansions == 0 || !std::isfinite(options->max_runtime_ms) ||
        options->max_runtime_ms < 0.0 || !std::isfinite(options->xy_resolution) ||
        options->xy_resolution <= 0.0 || options->heading_bins < 8 ||
        !std::isfinite(options->primitive_length) || options->primitive_length <= 0.0 ||
        !std::isfinite(options->collision_step) || options->collision_step <= 0.0 ||
        !std::isfinite(options->goal_xy_tolerance) || options->goal_xy_tolerance <= 0.0 ||
        !std::isfinite(options->goal_yaw_tolerance) || options->goal_yaw_tolerance <= 0.0 ||
        !std::isfinite(options->analytic_expansion_distance) || options->analytic_expansion_distance < 0.0 ||
        options->analytic_expansion_interval == 0 ||
        !std::isfinite(options->heuristic_weight) || options->heuristic_weight < 1.0 ||
        !std::isfinite(options->reverse_penalty) || options->reverse_penalty < 1.0 ||
        !std::isfinite(options->steering_penalty) || options->steering_penalty < 0.0 ||
        !std::isfinite(options->direction_switch_penalty) || options->direction_switch_penalty < 0.0 ||
        (options->allow_reverse != 0 && options->allow_reverse != 1)) {
        *error = "Hybrid A* options are invalid";
        return false;
    }
    return true;
}

int copy_trace(const TraceBuffer* source, pp_trace_result* target) {
    if (source == nullptr || target == nullptr) return 0;
    *target = pp_trace_result{};
    if (!source->events.empty()) {
        target->events = static_cast<pp_trace_event*>(
            std::malloc(source->events.size() * sizeof(pp_trace_event)));
        if (target->events == nullptr) return -1;
        std::memcpy(target->events, source->events.data(), source->events.size() * sizeof(pp_trace_event));
        target->event_count = source->events.size();
    }
    if (!source->points.empty()) {
        target->points = static_cast<double*>(
            std::malloc(source->points.size() * 3 * sizeof(double)));
        if (target->points == nullptr) {
            std::free(target->events);
            *target = pp_trace_result{};
            return -1;
        }
        for (size_t index = 0; index < source->points.size(); ++index) {
            target->points[index * 3] = source->points[index].x;
            target->points[index * 3 + 1] = source->points[index].y;
            target->points[index * 3 + 2] = source->points[index].yaw;
        }
        target->point_count = source->points.size();
        target->dimension = 3;
    }
    target->truncated = source->truncated ? 1 : 0;
    return 0;
}

int plan_impl(const pp_hybrid_astar_space* space, const double* start_values,
              const double* goal_values, const pp_hybrid_astar_options* options,
              pp_hybrid_astar_result* result, uint64_t trace_max_bytes,
              pp_trace_result* trace) {
    if (result == nullptr) return -1;
    *result = pp_hybrid_astar_result{};
    result->stop_reason = 2;
    const double started = monotonic_seconds();
    std::string error;
    if (!valid_input(space, start_values, goal_values, options, &error)) {
        set_error(result, error);
        return -1;
    }
    TraceBuffer trace_buffer(trace == nullptr ? 0 : trace_max_bytes);
    const Pose start{start_values[0], start_values[1], wrap_yaw(start_values[2])};
    const Pose goal{goal_values[0], goal_values[1], wrap_yaw(goal_values[2])};
    const double origin_x = space->occupancy == nullptr ? 0.0 : space->origin_x;
    const double origin_y = space->occupancy == nullptr ? 0.0 : space->origin_y;
    const double xy_resolution = options->xy_resolution;
    const double min_turning_radius = space->wheelbase / std::tan(space->max_steering_angle);
    const int start_valid = state_is_valid(*space, start, &result->motion_checks);
    if (start_valid < 0) {
        set_error(result, "state-validity callback failed for the start pose");
        return -1;
    }
    if (start_valid == 0) {
        set_error(result, "start pose is invalid or its footprint collides");
        return -1;
    }
    const int goal_valid = state_is_valid(*space, goal, &result->motion_checks);
    if (goal_valid < 0) {
        set_error(result, "state-validity callback failed for the goal pose");
        return -1;
    }
    if (goal_valid == 0) {
        set_error(result, "goal pose is invalid or its footprint collides");
        return -1;
    }

    std::vector<Node> nodes;
    nodes.reserve(static_cast<size_t>(std::min<uint64_t>(options->max_expansions, 65536)) + 1);
    std::unordered_map<Key, size_t, KeyHash> best_node;
    best_node.reserve(nodes.capacity());
    std::priority_queue<QueueItem, std::vector<QueueItem>, QueueGreater> open;
    nodes.push_back(Node{start, 0.0, std::numeric_limits<size_t>::max(), 0, 0.0, 0.0, false, {}});
    const Key start_key = make_key(start, 0, xy_resolution, origin_x, origin_y, options->heading_bins);
    best_node.emplace(start_key, 0);
    const double start_h = heuristic(start, goal, min_turning_radius);
    open.push({options->heuristic_weight * start_h, start_h, 0.0, 0});
    trace_buffer.add_point(start);
    trace_buffer.add_event(0, std::numeric_limits<uint64_t>::max(), 0.0, PP_TRACE_DISCOVER);
    size_t solution_id = std::numeric_limits<size_t>::max();

    while (!open.empty()) {
        if (result->iters >= options->max_expansions) {
            result->stop_reason = 1;
            break;
        }
        if (options->max_runtime_ms > 0.0 &&
            (monotonic_seconds() - started) * 1000.0 >= options->max_runtime_ms) {
            result->stop_reason = 4;
            break;
        }
        const QueueItem item = open.top();
        open.pop();
        if (item.node >= nodes.size()) continue;
        const Node current = nodes[item.node];
        if (item.cost > current.cost + 1e-10) continue;
        const Key current_key = make_key(current.pose, current.gear, xy_resolution,
                                         origin_x, origin_y, options->heading_bins);
        const auto current_best = best_node.find(current_key);
        if (current_best == best_node.end() || current_best->second != item.node) continue;
        ++result->iters;
        trace_buffer.add_event(item.node, current.parent, current.cost, PP_TRACE_EXPAND);

        const double goal_distance = std::hypot(current.pose.x - goal.x, current.pose.y - goal.y);
        if (goal_distance <= options->goal_xy_tolerance &&
            yaw_distance(current.pose.yaw, goal.yaw) <= options->goal_yaw_tolerance) {
            solution_id = item.node;
            result->first_solution_iter = result->iters;
            result->stop_reason = 0;
            trace_buffer.add_event(item.node, current.parent, current.cost, PP_TRACE_SOLUTION);
            break;
        }

        if (options->analytic_expansion_distance > 0.0 &&
            goal_distance <= options->analytic_expansion_distance &&
            (result->iters == 1 ||
             result->iters % options->analytic_expansion_interval == 0)) {
            ++result->analytic_expansions;
            auto connector = analytic_connector(*space, current.pose, goal, current.gear, *options,
                                                &result->motion_checks);
            if (!connector.empty()) {
                double analytic_cost = 0.0;
                int previous_gear = current.gear;
                for (const Segment& segment : connector) {
                    const double steering_fraction = std::abs(segment.steering) /
                                                     space->max_steering_angle;
                    analytic_cost += segment.length *
                                     (segment.gear < 0 ? options->reverse_penalty : 1.0) *
                                     (1.0 + options->steering_penalty * steering_fraction);
                    if (previous_gear != 0 && previous_gear != segment.gear) {
                        analytic_cost += options->direction_switch_penalty;
                    }
                    previous_gear = segment.gear;
                }
                Node goal_node;
                goal_node.pose = goal;
                goal_node.cost = current.cost + analytic_cost;
                goal_node.parent = item.node;
                goal_node.gear = connector.back().gear;
                goal_node.analytic = true;
                goal_node.analytic_segments = std::move(connector);
                solution_id = nodes.size();
                nodes.push_back(std::move(goal_node));
                trace_buffer.add_point(goal);
                trace_buffer.add_event(solution_id, item.node, nodes[solution_id].cost, PP_TRACE_DISCOVER);
                trace_buffer.add_event(solution_id, item.node, nodes[solution_id].cost, PP_TRACE_SOLUTION);
                result->first_solution_iter = result->iters;
                result->stop_reason = 0;
                break;
            }
        }

        const std::array<double, 3> steering_values = {
            -space->max_steering_angle, 0.0, space->max_steering_angle};
        const int first_gear = options->allow_reverse != 0 ? -1 : 1;
        for (int gear = first_gear; gear <= 1; gear += 2) {
            for (double steering : steering_values) {
                Pose next{};
                if (!motion_is_valid(*space, current.pose, gear, steering,
                                     options->primitive_length, options->collision_step,
                                     &result->motion_checks, &next)) {
                    continue;
                }
                const Key key = make_key(next, gear, xy_resolution, origin_x, origin_y,
                                         options->heading_bins);
                double edge_cost = options->primitive_length *
                                   (gear < 0 ? options->reverse_penalty : 1.0);
                const double steering_fraction = std::abs(steering) /
                                                 space->max_steering_angle;
                edge_cost *= 1.0 + options->steering_penalty * steering_fraction;
                if (current.gear != 0 && current.gear != gear) {
                    edge_cost += options->direction_switch_penalty;
                }
                const double tentative = current.cost + edge_cost;
                auto found = best_node.find(key);
                if (found != best_node.end() &&
                    tentative + 1e-10 >= nodes[found->second].cost) continue;
                const size_t next_id = nodes.size();
                nodes.push_back(Node{next, tentative, item.node, gear,
                                     steering, options->primitive_length, false, {}});
                if (found == best_node.end()) best_node.emplace(key, next_id);
                else found->second = next_id;
                trace_buffer.add_point(next);
                trace_buffer.add_event(next_id, item.node, tentative, PP_TRACE_DISCOVER);
                const double next_h = heuristic(next, goal, min_turning_radius);
                open.push({tentative + options->heuristic_weight * next_h, next_h,
                           tentative, next_id});
            }
        }
    }

    result->nodes = nodes.size();
    if (solution_id != std::numeric_limits<size_t>::max()) {
        std::vector<size_t> chain;
        for (size_t cursor = solution_id; cursor != std::numeric_limits<size_t>::max();
             cursor = nodes[cursor].parent) {
            chain.push_back(cursor);
        }
        std::reverse(chain.begin(), chain.end());
        std::vector<Pose> path;
        std::vector<int8_t> directions;
        path.push_back(nodes[chain.front()].pose);
        directions.push_back(0);
        for (size_t index = 1; index < chain.size(); ++index) {
            const Node& node = nodes[chain[index]];
            Pose pose = nodes[node.parent].pose;
            auto append_segment = [&](const Segment& segment) {
                const size_t steps = std::max<size_t>(1, static_cast<size_t>(
                    std::ceil(segment.length / options->collision_step)));
                const Pose segment_start = pose;
                for (size_t step = 1; step <= steps; ++step) {
                    pose = integrate(segment_start, segment.gear, segment.steering,
                                     segment.length * static_cast<double>(step) /
                                         static_cast<double>(steps),
                                     space->wheelbase);
                    path.push_back(pose);
                    directions.push_back(static_cast<int8_t>(segment.gear));
                }
            };
            if (node.analytic) {
                for (const Segment& segment : node.analytic_segments) append_segment(segment);
            } else {
                append_segment({node.gear, node.steering, node.primitive_length});
            }
        }
        if (!path.empty() && nodes[solution_id].analytic) path.back() = goal;
        result->path_length = path.size();
        result->poses = static_cast<double*>(std::malloc(path.size() * 3 * sizeof(double)));
        result->directions = static_cast<int8_t*>(std::malloc(path.size() * sizeof(int8_t)));
        if (result->poses == nullptr || result->directions == nullptr) {
            std::free(result->poses);
            std::free(result->directions);
            result->poses = nullptr;
            result->directions = nullptr;
            result->path_length = 0;
            set_error(result, "could not allocate Hybrid A* path buffers");
            return -1;
        }
        for (size_t index = 0; index < path.size(); ++index) {
            result->poses[index * 3] = path[index].x;
            result->poses[index * 3 + 1] = path[index].y;
            result->poses[index * 3 + 2] = path[index].yaw;
            result->directions[index] = directions[index];
        }
        result->success = 1;
        result->path_cost = nodes[solution_id].cost;
    }
    result->elapsed_s = monotonic_seconds() - started;
    if (trace != nullptr && copy_trace(&trace_buffer, trace) != 0) {
        set_error(result, "could not allocate Hybrid A* trace buffers");
        return -1;
    }
    return 0;
}

}  // namespace

extern "C" uint32_t pp_kinodynamic_abi_version(void) {
    return PP_KINODYNAMIC_ABI_VERSION;
}

extern "C" int pp_hybrid_astar_plan(const pp_hybrid_astar_space* space,
                                    const double* start,
                                    const double* goal,
                                    const pp_hybrid_astar_options* options,
                                    pp_hybrid_astar_result* result) {
    try {
        return plan_impl(space, start, goal, options, result, 0, nullptr);
    } catch (const std::bad_alloc&) {
        if (result != nullptr) set_error(result, "out of memory during Hybrid A* search");
        return -1;
    } catch (const std::exception& exception) {
        if (result != nullptr) set_error(result, exception.what());
        return -1;
    } catch (...) {
        if (result != nullptr) set_error(result, "unknown Hybrid A* failure");
        return -1;
    }
}

extern "C" void pp_hybrid_astar_free_result(pp_hybrid_astar_result* result) {
    if (result == nullptr) return;
    std::free(result->poses);
    std::free(result->directions);
    std::free(result->error_message);
    *result = pp_hybrid_astar_result{};
}

#if defined(PP_ENABLE_TRACE) && PP_ENABLE_TRACE
extern "C" uint32_t pp_kinodynamic_trace_abi_version(void) {
    return PP_KINODYNAMIC_TRACE_ABI_VERSION;
}

extern "C" int pp_hybrid_astar_plan_traced(const pp_hybrid_astar_space* space,
                                           const double* start,
                                           const double* goal,
                                           const pp_hybrid_astar_options* options,
                                           uint64_t trace_max_bytes,
                                           pp_hybrid_astar_result* result,
                                           pp_trace_result* trace) {
    if (trace == nullptr) {
        if (result != nullptr) set_error(result, "trace result must not be null");
        return -1;
    }
    try {
        return plan_impl(space, start, goal, options, result, trace_max_bytes, trace);
    } catch (const std::bad_alloc&) {
        if (result != nullptr) set_error(result, "out of memory during traced Hybrid A* search");
        return -1;
    } catch (const std::exception& exception) {
        if (result != nullptr) set_error(result, exception.what());
        return -1;
    } catch (...) {
        if (result != nullptr) set_error(result, "unknown traced Hybrid A* failure");
        return -1;
    }
}
#endif
