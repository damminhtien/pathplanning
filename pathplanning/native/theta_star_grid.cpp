#include "search_engine.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdlib>
#include <cstring>
#include <limits>
#include <new>
#include <queue>
#include <stdexcept>
#include <string>
#include <tuple>
#include <vector>

namespace {

struct ThetaPoint {
    int64_t x;
    int64_t y;

    bool operator==(const ThetaPoint& other) const {
        return x == other.x && y == other.y;
    }
};

struct ThetaDirection {
    int64_t x;
    int64_t y;
};

constexpr std::array<ThetaDirection, 8> kThetaDirections = {{
    {-1, 0}, {-1, 1}, {0, 1}, {1, 1},
    {1, 0}, {1, -1}, {0, -1}, {-1, -1},
}};
constexpr uint64_t kThetaNoParent = std::numeric_limits<uint64_t>::max();
constexpr double kThetaInfinity = std::numeric_limits<double>::infinity();

struct ThetaOpenEntry {
    double f;
    double g;
    uint64_t id;
};

struct ThetaOpenGreater {
    bool operator()(const ThetaOpenEntry& left, const ThetaOpenEntry& right) const {
        return std::tie(left.f, left.g, left.id) > std::tie(right.f, right.g, right.id);
    }
};

struct ThetaTraceEvent {
    uint64_t node;
    uint64_t parent;
    double value;
    uint32_t kind;
    uint32_t side;
};

class ThetaTraceBuffer {
public:
    explicit ThetaTraceBuffer(size_t max_events) : max_events_(max_events) {}

    void record(uint64_t node, uint64_t parent, double value, uint32_t kind) {
        if (events_.size() >= max_events_) {
            truncated_ = true;
            return;
        }
        events_.push_back({node, parent, value, kind, 0});
    }

    const std::vector<ThetaTraceEvent>& events() const { return events_; }
    bool truncated() const { return truncated_; }

private:
    size_t max_events_;
    bool truncated_ = false;
    std::vector<ThetaTraceEvent> events_;
};

struct ThetaMetrics {
    uint64_t line_of_sight_checks = 0;
    uint64_t cells_checked = 0;
    uint64_t expanded_nodes = 0;
};

class VisibilityGrid {
public:
    VisibilityGrid(
        uint64_t width,
        uint64_t height,
        const uint8_t* valid_nodes,
        const double* terrain_costs,
        double min_terrain_cost,
        ThetaMetrics* metrics
    ) : width_(width),
        height_(height),
        valid_(valid_nodes),
        terrain_costs_(terrain_costs),
        min_terrain_cost_(min_terrain_cost),
        metrics_(metrics) {}

    bool valid(ThetaPoint point) const {
        ++metrics_->cells_checked;
        return point.x >= 0 && point.y >= 0 &&
            static_cast<uint64_t>(point.x) < width_ &&
            static_cast<uint64_t>(point.y) < height_ &&
            valid_[id(point)] != 0;
    }

    uint64_t id(ThetaPoint point) const {
        return static_cast<uint64_t>(point.y) * width_ + static_cast<uint64_t>(point.x);
    }

    ThetaPoint point(uint64_t node_id) const {
        return {
            static_cast<int64_t>(node_id % width_),
            static_cast<int64_t>(node_id / width_),
        };
    }

    uint64_t node_count() const { return width_ * height_; }

    double heuristic(ThetaPoint first, ThetaPoint second) const {
        return min_terrain_cost_ * std::hypot(
            static_cast<double>(second.x - first.x),
            static_cast<double>(second.y - first.y)
        );
    }

    bool segment_cost(ThetaPoint from, ThetaPoint to, double* cost) const {
        ++metrics_->line_of_sight_checks;
        if (!valid(from) || !valid(to)) {
            return false;
        }
        const int64_t dx = to.x - from.x;
        const int64_t dy = to.y - from.y;
        const uint64_t abs_dx = static_cast<uint64_t>(std::abs(dx));
        const uint64_t abs_dy = static_cast<uint64_t>(std::abs(dy));
        const int64_t step_x = (dx > 0) - (dx < 0);
        const int64_t step_y = (dy > 0) - (dy < 0);
        const double length = std::hypot(static_cast<double>(dx), static_cast<double>(dy));
        if (length == 0.0) {
            *cost = 0.0;
            return true;
        }

        ThetaPoint current = from;
        uint64_t crossed_x = 0;
        uint64_t crossed_y = 0;
        double previous_t = 0.0;
        double total = 0.0;
        while (!(current == to)) {
            const uint64_t x_numerator = 2 * crossed_x + 1;
            const uint64_t y_numerator = 2 * crossed_y + 1;
            const bool can_cross_x = current.x != to.x;
            const bool can_cross_y = current.y != to.y;
            const uint64_t x_order = !can_cross_x
                ? std::numeric_limits<uint64_t>::max()
                : x_numerator * abs_dy;
            const uint64_t y_order = !can_cross_y
                ? std::numeric_limits<uint64_t>::max()
                : y_numerator * abs_dx;

            if (x_order < y_order) {
                const double next_t = static_cast<double>(x_numerator) /
                    (2.0 * static_cast<double>(abs_dx));
                total += (next_t - previous_t) * length * terrain(current);
                current.x += step_x;
                if (!valid(current)) {
                    return false;
                }
                ++crossed_x;
                previous_t = next_t;
            } else if (y_order < x_order) {
                const double next_t = static_cast<double>(y_numerator) /
                    (2.0 * static_cast<double>(abs_dy));
                total += (next_t - previous_t) * length * terrain(current);
                current.y += step_y;
                if (!valid(current)) {
                    return false;
                }
                ++crossed_y;
                previous_t = next_t;
            } else {
                const double next_t = static_cast<double>(x_numerator) /
                    (2.0 * static_cast<double>(abs_dx));
                total += (next_t - previous_t) * length * terrain(current);
                if (!valid({current.x + step_x, current.y}) ||
                    !valid({current.x, current.y + step_y})) {
                    return false;
                }
                current.x += step_x;
                current.y += step_y;
                if (!valid(current)) {
                    return false;
                }
                ++crossed_x;
                ++crossed_y;
                previous_t = next_t;
            }
        }
        total += (1.0 - previous_t) * length * terrain(current);
        *cost = total;
        return true;
    }

    double terrain(ThetaPoint point) const {
        return terrain_costs_[id(point)];
    }

private:
    uint64_t width_;
    uint64_t height_;
    const uint8_t* valid_;
    const double* terrain_costs_;
    double min_terrain_cost_;
    ThetaMetrics* metrics_;
};

int finish(
    const std::vector<uint64_t>& path,
    double cost,
    uint64_t expanded,
    uint64_t discovered,
    int reason,
    pp_search_result* result
) {
    result->success = reason == PP_SEARCH_STOP_SUCCESS ? 1 : 0;
    result->stop_reason = reason;
    result->iters = expanded;
    result->nodes = discovered;
    result->path_cost = cost;
    if (!path.empty()) {
        result->path_ids = static_cast<uint64_t*>(
            std::malloc(path.size() * sizeof(uint64_t))
        );
        if (result->path_ids == nullptr) {
            throw std::bad_alloc();
        }
        std::memcpy(result->path_ids, path.data(), path.size() * sizeof(uint64_t));
        result->path_length = path.size();
    }
    return 0;
}

class ThetaStarSearch {
public:
    ThetaStarSearch(const VisibilityGrid* grid, ThetaMetrics* metrics)
        : grid_(grid), metrics_(metrics) {}

    int plan(
        ThetaPoint start,
        ThetaPoint goal,
        bool has_max_expansions,
        uint64_t max_expansions,
        pp_search_result* result,
        ThetaTraceBuffer* trace
    ) const {
        const uint64_t start_id = grid_->id(start);
        const uint64_t goal_id = grid_->id(goal);
        const size_t node_count = static_cast<size_t>(grid_->node_count());
        std::vector<double> costs(node_count, kThetaInfinity);
        std::vector<uint64_t> parents(node_count, kThetaNoParent);
        std::vector<uint8_t> closed(node_count, 0);
        std::priority_queue<ThetaOpenEntry, std::vector<ThetaOpenEntry>, ThetaOpenGreater> queue;
        costs[static_cast<size_t>(start_id)] = 0.0;
        queue.push({grid_->heuristic(start, goal), 0.0, start_id});
        if (trace != nullptr) {
            trace->record(start_id, kThetaNoParent, 0.0, 1);
        }
        uint64_t expanded = 0;
        uint64_t discovered = 1;

        while (!queue.empty()) {
            const ThetaOpenEntry entry = queue.top();
            queue.pop();
            const size_t current_index = static_cast<size_t>(entry.id);
            if (closed[current_index] || entry.g != costs[current_index]) {
                continue;
            }
            if (entry.id == goal_id) {
                if (trace != nullptr) {
                    trace->record(entry.id, parents[current_index], entry.g, 6);
                }
                return finish(
                    reconstruct_path(start_id, goal_id, parents),
                    entry.g,
                    expanded,
                    discovered,
                    PP_SEARCH_STOP_SUCCESS,
                    result
                );
            }
            if (has_max_expansions && expanded >= max_expansions) {
                return finish({}, kThetaInfinity, expanded, discovered, PP_SEARCH_STOP_MAX_ITERS, result);
            }
            closed[current_index] = 1;
            ++expanded;
            ++metrics_->expanded_nodes;
            if (trace != nullptr) {
                trace->record(entry.id, parents[current_index], entry.g, 2);
            }
            const ThetaPoint current = grid_->point(entry.id);
            const uint64_t parent_id = parents[current_index];
            const bool has_parent = parent_id != kThetaNoParent;
            const ThetaPoint parent = has_parent ? grid_->point(parent_id) : current;

            for (const ThetaDirection direction : kThetaDirections) {
                const ThetaPoint successor{current.x + direction.x, current.y + direction.y};
                double step_cost = 0.0;
                if (!grid_->segment_cost(current, successor, &step_cost)) {
                    continue;
                }
                ThetaPoint candidate_parent = current;
                double candidate = entry.g + step_cost;
                if (has_parent) {
                    double shortcut_cost = 0.0;
                    if (grid_->segment_cost(parent, successor, &shortcut_cost)) {
                        candidate_parent = parent;
                        candidate = costs[static_cast<size_t>(parent_id)] + shortcut_cost;
                    }
                }
                const uint64_t successor_id = grid_->id(successor);
                const size_t successor_index = static_cast<size_t>(successor_id);
                if (closed[successor_index] || candidate >= costs[successor_index]) {
                    continue;
                }
                const bool first_discovery = !std::isfinite(costs[successor_index]);
                if (first_discovery) {
                    ++discovered;
                }
                costs[successor_index] = candidate;
                parents[successor_index] = grid_->id(candidate_parent);
                if (trace != nullptr) {
                    const uint64_t candidate_parent_id = grid_->id(candidate_parent);
                    if (first_discovery) {
                        trace->record(successor_id, candidate_parent_id, candidate, 1);
                    }
                    trace->record(successor_id, candidate_parent_id, candidate, 3);
                }
                queue.push({candidate + grid_->heuristic(successor, goal), candidate, successor_id});
            }
        }
        return finish({}, kThetaInfinity, expanded, discovered, PP_SEARCH_STOP_NO_PROGRESS, result);
    }

private:
    std::vector<uint64_t> reconstruct_path(
        uint64_t start_id,
        uint64_t goal_id,
        const std::vector<uint64_t>& parents
    ) const {
        std::vector<uint64_t> path;
        uint64_t current = goal_id;
        while (current != kThetaNoParent) {
            path.push_back(current);
            if (current == start_id) {
                break;
            }
            current = parents[static_cast<size_t>(current)];
        }
        if (path.empty() || path.back() != start_id) {
            throw std::runtime_error("Theta* parent chain did not reach the start");
        }
        std::reverse(path.begin(), path.end());
        return path;
    }

    const VisibilityGrid* grid_;
    ThetaMetrics* metrics_;
};

void reset_result(pp_search_result* result) {
    *result = pp_search_result{};
    result->stop_reason = PP_SEARCH_STOP_ERROR;
    result->path_cost = kThetaInfinity;
}

void set_error(pp_search_result* result, const std::string& message) {
    reset_result(result);
    result->error_message = static_cast<char*>(std::malloc(message.size() + 1));
    if (result->error_message != nullptr) {
        std::memcpy(result->error_message, message.c_str(), message.size() + 1);
    }
}

int run_theta_star_grid(
    uint64_t width,
    uint64_t height,
    const uint8_t* valid_nodes,
    const double* terrain_costs,
    uint64_t start_x,
    uint64_t start_y,
    uint64_t goal_x,
    uint64_t goal_y,
    int has_max_expansions,
    uint64_t max_expansions,
    pp_search_result* result,
    pp_theta_metrics* output_metrics,
    ThetaTraceBuffer* trace
) {
    if (output_metrics == nullptr || output_metrics->struct_size != sizeof(pp_theta_metrics)) {
        set_error(result, "Theta* metrics struct size mismatch");
        return 1;
    }
    output_metrics->line_of_sight_checks = 0;
    output_metrics->cells_checked = 0;
    output_metrics->expanded_nodes = 0;
    reset_result(result);
    try {
        if (width == 0 || height == 0 || width > static_cast<uint64_t>(INT32_MAX) ||
            height > static_cast<uint64_t>(INT32_MAX) ||
            width > std::numeric_limits<size_t>::max() / height) {
            throw std::invalid_argument("grid dimensions are invalid or too large");
        }
        if (valid_nodes == nullptr || terrain_costs == nullptr) {
            throw std::invalid_argument("valid_nodes and terrain_costs must not be null");
        }
        if (start_x >= width || start_y >= height || goal_x >= width || goal_y >= height) {
            throw std::invalid_argument("start and goal must be in grid bounds");
        }
        if (has_max_expansions != 0 && max_expansions == 0) {
            throw std::invalid_argument("max_expansions must be positive");
        }
        const size_t node_count = static_cast<size_t>(width * height);
        double min_terrain_cost = kThetaInfinity;
        for (size_t index = 0; index < node_count; ++index) {
            if (!std::isfinite(terrain_costs[index]) || terrain_costs[index] <= 0.0) {
                throw std::invalid_argument("terrain costs must be finite and strictly positive");
            }
            min_terrain_cost = std::min(min_terrain_cost, terrain_costs[index]);
        }
        ThetaMetrics metrics;
        const VisibilityGrid grid(
            width,
            height,
            valid_nodes,
            terrain_costs,
            min_terrain_cost,
            &metrics
        );
        const ThetaPoint start{static_cast<int64_t>(start_x), static_cast<int64_t>(start_y)};
        const ThetaPoint goal{static_cast<int64_t>(goal_x), static_cast<int64_t>(goal_y)};
        if (!grid.valid(start) || !grid.valid(goal)) {
            result->stop_reason = PP_SEARCH_STOP_NO_PROGRESS;
            output_metrics->cells_checked = metrics.cells_checked;
            return 0;
        }
        ThetaStarSearch search(&grid, &metrics);
        const int status = search.plan(
            start,
            goal,
            has_max_expansions != 0,
            max_expansions,
            result,
            trace
        );
        output_metrics->line_of_sight_checks = metrics.line_of_sight_checks;
        output_metrics->cells_checked = metrics.cells_checked;
        output_metrics->expanded_nodes = metrics.expanded_nodes;
        return status;
    } catch (const std::exception& error) {
        set_error(result, error.what());
        return 1;
    } catch (...) {
        set_error(result, "unknown Theta* error");
        return 1;
    }
}

}  // namespace

extern "C" int pp_native_theta_star_grid(
    uint64_t width,
    uint64_t height,
    const uint8_t* valid_nodes,
    const double* terrain_costs,
    uint64_t start_x,
    uint64_t start_y,
    uint64_t goal_x,
    uint64_t goal_y,
    int has_max_expansions,
    uint64_t max_expansions,
    pp_search_result* result,
    pp_theta_metrics* metrics
) {
    if (result == nullptr) {
        return 1;
    }
    return run_theta_star_grid(
        width,
        height,
        valid_nodes,
        terrain_costs,
        start_x,
        start_y,
        goal_x,
        goal_y,
        has_max_expansions,
        max_expansions,
        result,
        metrics,
        nullptr
    );
}

#if defined(PP_ENABLE_TRACE) && PP_ENABLE_TRACE
extern "C" int pp_native_theta_star_grid_traced(
    uint64_t width,
    uint64_t height,
    const uint8_t* valid_nodes,
    const double* terrain_costs,
    uint64_t start_x,
    uint64_t start_y,
    uint64_t goal_x,
    uint64_t goal_y,
    int has_max_expansions,
    uint64_t max_expansions,
    uint64_t trace_max_bytes,
    pp_search_result* result,
    pp_trace_result* trace,
    pp_theta_metrics* metrics
) {
    if (result == nullptr || trace == nullptr) {
        return 1;
    }
    *trace = pp_trace_result{};
    const size_t event_capacity = static_cast<size_t>(trace_max_bytes / sizeof(pp_trace_event));
    ThetaTraceBuffer captured(event_capacity);
    const int status = run_theta_star_grid(
        width,
        height,
        valid_nodes,
        terrain_costs,
        start_x,
        start_y,
        goal_x,
        goal_y,
        has_max_expansions,
        max_expansions,
        result,
        metrics,
        &captured
    );
    const auto& events = captured.events();
    if (!events.empty()) {
        trace->events = static_cast<pp_trace_event*>(
            std::malloc(events.size() * sizeof(pp_trace_event))
        );
        if (trace->events == nullptr) {
            std::free(result->path_ids);
            result->path_ids = nullptr;
            result->path_length = 0;
            set_error(result, "could not allocate Theta* trace events");
            return 1;
        }
        for (size_t index = 0; index < events.size(); ++index) {
            trace->events[index] = pp_trace_event{
                events[index].node,
                events[index].parent,
                events[index].value,
                events[index].kind,
                events[index].side,
            };
        }
        trace->event_count = events.size();
    }
    trace->truncated = captured.truncated() ? 1 : 0;
    return status;
}
#endif
