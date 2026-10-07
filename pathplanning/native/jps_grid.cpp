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
#include <utility>
#include <vector>

namespace {

struct Point {
    int64_t x;
    int64_t y;

    bool operator==(const Point& other) const {
        return x == other.x && y == other.y;
    }
};

struct Direction {
    int64_t x;
    int64_t y;
};

constexpr std::array<Direction, 8> kDirections = {{
    {-1, 0}, {-1, 1}, {0, 1}, {1, 1},
    {1, 0}, {1, -1}, {0, -1}, {-1, -1},
}};
constexpr uint64_t kNoParent = std::numeric_limits<uint64_t>::max();
constexpr double kInfinity = std::numeric_limits<double>::infinity();

struct OpenEntry {
    double f;
    double g;
    uint64_t id;
};

struct OpenEntryGreater {
    bool operator()(const OpenEntry& left, const OpenEntry& right) const {
        return std::tie(left.f, left.g, left.id) > std::tie(right.f, right.g, right.id);
    }
};

struct LocalEntry {
    double cost;
    Point point;
};

struct LocalEntryGreater {
    bool operator()(const LocalEntry& left, const LocalEntry& right) const {
        return std::tie(left.cost, left.point.y, left.point.x) >
            std::tie(right.cost, right.point.y, right.point.x);
    }
};

struct JpsTraceEvent {
    uint64_t node;
    uint64_t parent;
    double value;
    uint32_t kind;
    uint32_t side;
};

class JpsTraceBuffer {
public:
    explicit JpsTraceBuffer(size_t max_events) : max_events_(max_events) {}

    void record(uint64_t node, uint64_t parent, double value, uint32_t kind) {
        if (events_.size() >= max_events_) {
            truncated_ = true;
            return;
        }
        events_.push_back({node, parent, value, kind, 0});
    }

    const std::vector<JpsTraceEvent>& events() const { return events_; }
    bool truncated() const { return truncated_; }

private:
    size_t max_events_;
    bool truncated_ = false;
    std::vector<JpsTraceEvent> events_;
};

class GridJps {
public:
    GridJps(uint64_t width, uint64_t height, const uint8_t* valid)
        : width_(width), height_(height), valid_(valid) {}

    bool valid(Point point) const {
        return point.x >= 0 && point.y >= 0 &&
            static_cast<uint64_t>(point.x) < width_ &&
            static_cast<uint64_t>(point.y) < height_ &&
            valid_[id(point)] != 0;
    }

    bool legal_step(Point from, Point to) const {
        ++motion_checks_;
        if (!valid(to)) {
            return false;
        }
        const int64_t dx = to.x - from.x;
        const int64_t dy = to.y - from.y;
        if (dx != 0 && dy != 0) {
            return valid({from.x + dx, from.y}) && valid({from.x, from.y + dy});
        }
        return dx != 0 || dy != 0;
    }

    uint64_t id(Point point) const {
        return static_cast<uint64_t>(point.y) * width_ + static_cast<uint64_t>(point.x);
    }

    Point point(uint64_t id) const {
        return {
            static_cast<int64_t>(id % width_),
            static_cast<int64_t>(id / width_),
        };
    }

    static double distance(Point first, Point second) {
        return std::hypot(
            static_cast<double>(second.x - first.x),
            static_cast<double>(second.y - first.y)
        );
    }

    static double heuristic(Point first, Point second) {
        const double dx = std::abs(static_cast<double>(second.x - first.x));
        const double dy = std::abs(static_cast<double>(second.y - first.y));
        const double diagonal = std::min(dx, dy);
        return std::max(dx, dy) + (std::sqrt(2.0) - 1.0) * diagonal;
    }

    std::vector<Direction> forced_successors(Point parent, Point current) const {
        std::vector<Direction> forced;
        for (const Direction direction : kDirections) {
            const Point successor{current.x + direction.x, current.y + direction.y};
            if (successor == parent || !legal_step(current, successor)) {
                continue;
            }
            const double through_cost = distance(parent, current) + distance(current, successor);
            std::array<double, 25> costs;
            costs.fill(kInfinity);
            auto local_index = [current](Point point) -> size_t {
                return static_cast<size_t>(point.y - current.y + 2) * 5 +
                    static_cast<size_t>(point.x - current.x + 2);
            };
            std::priority_queue<LocalEntry, std::vector<LocalEntry>, LocalEntryGreater> queue;
            costs[local_index(parent)] = 0.0;
            queue.push({0.0, parent});
            double alternative_cost = kInfinity;
            while (!queue.empty()) {
                const LocalEntry entry = queue.top();
                queue.pop();
                if (entry.cost != costs[local_index(entry.point)] ||
                    entry.cost > through_cost + 1e-12) {
                    continue;
                }
                if (entry.point == successor) {
                    alternative_cost = entry.cost;
                    break;
                }
                for (const Direction step : kDirections) {
                    const Point next{entry.point.x + step.x, entry.point.y + step.y};
                    if (next == current ||
                        std::max(
                            std::abs(next.x - current.x),
                            std::abs(next.y - current.y)
                        ) > 2 ||
                        !legal_step(entry.point, next)) {
                        continue;
                    }
                    const double candidate = entry.cost + distance(entry.point, next);
                    const size_t next_index = local_index(next);
                    if (candidate <= through_cost + 1e-12 && candidate < costs[next_index]) {
                        costs[next_index] = candidate;
                        queue.push({candidate, next});
                    }
                }
            }
            if (alternative_cost > through_cost + 1e-12) {
                forced.push_back(direction);
            }
        }
        return forced;
    }

    bool straight_jump(Point origin, Direction direction, Point goal, Point* found) const {
        Point previous = origin;
        Point current{origin.x + direction.x, origin.y + direction.y};
        while (legal_step(previous, current)) {
            if (current == goal || !forced_successors(previous, current).empty()) {
                *found = current;
                return true;
            }
            previous = current;
            current = {current.x + direction.x, current.y + direction.y};
        }
        return false;
    }

    bool jump(Point origin, Direction direction, Point goal, Point* found) const {
        Point previous = origin;
        Point current{origin.x + direction.x, origin.y + direction.y};
        while (legal_step(previous, current)) {
            if (current == goal || !forced_successors(previous, current).empty()) {
                *found = current;
                return true;
            }
            if (direction.x != 0 && direction.y != 0) {
                Point ignored{};
                if (straight_jump(current, {direction.x, 0}, goal, &ignored) ||
                    straight_jump(current, {0, direction.y}, goal, &ignored)) {
                    *found = current;
                    return true;
                }
            }
            previous = current;
            current = {current.x + direction.x, current.y + direction.y};
        }
        return false;
    }

    std::vector<Direction> successors(Point parent, Point current) const {
        std::vector<Direction> output;
        const int64_t dx = (current.x > parent.x) - (current.x < parent.x);
        const int64_t dy = (current.y > parent.y) - (current.y < parent.y);
        output.push_back({dx, dy});
        if (dx != 0 && dy != 0) {
            output.push_back({dx, 0});
            output.push_back({0, dy});
        }
        for (const Direction forced : forced_successors(parent, current)) {
            bool already_present = false;
            for (const Direction existing : output) {
                if (existing.x == forced.x && existing.y == forced.y) {
                    already_present = true;
                    break;
                }
            }
            if (!already_present) {
                output.push_back(forced);
            }
        }
        return output;
    }

    uint64_t motion_checks() const { return motion_checks_; }

    int plan(
        Point start,
        Point goal,
        bool has_max_expansions,
        uint64_t max_expansions,
        pp_search_result* result,
        JpsTraceBuffer* trace = nullptr
    ) const {
        const size_t node_count = static_cast<size_t>(width_ * height_);
        const uint64_t start_id = id(start);
        const uint64_t goal_id = id(goal);
        if (start_id == goal_id) {
            if (trace != nullptr) {
                trace->record(start_id, kNoParent, 0.0, 1);
                trace->record(start_id, kNoParent, 0.0, 6);
            }
            return finish({start_id}, 0.0, 0, 1, PP_SEARCH_STOP_SUCCESS, result);
        }

        std::vector<double> costs(node_count, kInfinity);
        std::vector<uint64_t> parents(node_count, kNoParent);
        std::vector<uint8_t> closed(node_count, 0);
        std::priority_queue<OpenEntry, std::vector<OpenEntry>, OpenEntryGreater> queue;
        costs[static_cast<size_t>(start_id)] = 0.0;
        queue.push({heuristic(start, goal), 0.0, start_id});
        if (trace != nullptr) {
            trace->record(start_id, kNoParent, 0.0, 1);
        }
        uint64_t expanded = 0;
        uint64_t discovered = 1;

        while (!queue.empty()) {
            const OpenEntry entry = queue.top();
            queue.pop();
            if (closed[static_cast<size_t>(entry.id)] ||
                entry.g != costs[static_cast<size_t>(entry.id)]) {
                continue;
            }
            if (entry.id == goal_id) {
                if (trace != nullptr) {
                    trace->record(entry.id, parents[static_cast<size_t>(entry.id)], entry.g, 6);
                }
                std::vector<uint64_t> jump_path;
                uint64_t current = goal_id;
                while (current != kNoParent) {
                    jump_path.push_back(current);
                    if (current == start_id) {
                        break;
                    }
                    current = parents[static_cast<size_t>(current)];
                }
                if (jump_path.empty() || jump_path.back() != start_id) {
                    throw std::runtime_error("JPS parent chain did not reach the start");
                }
                std::reverse(jump_path.begin(), jump_path.end());
                std::vector<uint64_t> path{start_id};
                for (size_t index = 1; index < jump_path.size(); ++index) {
                    Point from = point(jump_path[index - 1]);
                    const Point to = point(jump_path[index]);
                    const Direction step{
                        (to.x > from.x) - (to.x < from.x),
                        (to.y > from.y) - (to.y < from.y),
                    };
                    while (!(from == to)) {
                        const Point next{from.x + step.x, from.y + step.y};
                        if (!legal_step(from, next)) {
                            throw std::runtime_error("JPS produced an invalid grid segment");
                        }
                        path.push_back(id(next));
                        from = next;
                    }
                }
                return finish(
                    path,
                    entry.g,
                    expanded,
                    discovered,
                    PP_SEARCH_STOP_SUCCESS,
                    result
                );
            }
            if (has_max_expansions && expanded >= max_expansions) {
                return finish({}, kInfinity, expanded, discovered, PP_SEARCH_STOP_MAX_ITERS, result);
            }
            closed[static_cast<size_t>(entry.id)] = 1;
            ++expanded;
            if (trace != nullptr) {
                trace->record(
                    entry.id,
                    parents[static_cast<size_t>(entry.id)],
                    entry.g,
                    2
                );
            }
            const Point current = point(entry.id);
            const uint64_t parent_id = parents[static_cast<size_t>(entry.id)];
            const std::vector<Direction> directions = parent_id == kNoParent
                ? std::vector<Direction>(kDirections.begin(), kDirections.end())
                : successors(point(parent_id), current);
            for (const Direction direction : directions) {
                Point successor{};
                if (!jump(current, direction, goal, &successor)) {
                    continue;
                }
                const uint64_t successor_id = id(successor);
                if (closed[static_cast<size_t>(successor_id)]) {
                    continue;
                }
                const double candidate = entry.g + distance(current, successor);
                const size_t successor_index = static_cast<size_t>(successor_id);
                if (candidate < costs[successor_index]) {
                    const bool first_discovery = !std::isfinite(costs[successor_index]);
                    if (first_discovery) {
                        ++discovered;
                    }
                    costs[successor_index] = candidate;
                    parents[successor_index] = entry.id;
                    if (trace != nullptr) {
                        if (first_discovery) {
                            trace->record(successor_id, entry.id, candidate, 1);
                        }
                        trace->record(successor_id, entry.id, candidate, 3);
                    }
                    queue.push({candidate + heuristic(successor, goal), candidate, successor_id});
                }
            }
        }
        return finish({}, kInfinity, expanded, discovered, PP_SEARCH_STOP_NO_PROGRESS, result);
    }

private:
    static int finish(
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

    uint64_t width_;
    uint64_t height_;
    const uint8_t* valid_;
    mutable uint64_t motion_checks_ = 0;
};

void reset_result(pp_search_result* result) {
    *result = pp_search_result{};
    result->stop_reason = PP_SEARCH_STOP_ERROR;
    result->path_cost = kInfinity;
}

void set_error(pp_search_result* result, const std::string& message) {
    reset_result(result);
    result->error_message = static_cast<char*>(std::malloc(message.size() + 1));
    if (result->error_message != nullptr) {
        std::memcpy(result->error_message, message.c_str(), message.size() + 1);
    }
}

int run_jps_grid(
    uint64_t width,
    uint64_t height,
    const uint8_t* valid_nodes,
    uint64_t start_x,
    uint64_t start_y,
    uint64_t goal_x,
    uint64_t goal_y,
    int has_max_expansions,
    uint64_t max_expansions,
    pp_search_result* result,
    pp_jps_metrics* metrics,
    JpsTraceBuffer* trace
) {
    if (metrics == nullptr || metrics->struct_size != sizeof(pp_jps_metrics)) {
        set_error(result, "JPS metrics struct size mismatch");
        return 1;
    }
    metrics->motion_checks = 0;
    metrics->jump_points_expanded = 0;
    reset_result(result);
    try {
        if (width == 0 || height == 0 || width > static_cast<uint64_t>(INT32_MAX) ||
            height > static_cast<uint64_t>(INT32_MAX) ||
            width > std::numeric_limits<size_t>::max() / height) {
            throw std::invalid_argument("grid dimensions are invalid or too large");
        }
        if (valid_nodes == nullptr) {
            throw std::invalid_argument("valid_nodes must not be null");
        }
        if (start_x >= width || start_y >= height || goal_x >= width || goal_y >= height) {
            throw std::invalid_argument("start and goal must be in grid bounds");
        }
        if (has_max_expansions != 0 && max_expansions == 0) {
            throw std::invalid_argument("max_expansions must be positive");
        }
        const GridJps planner(width, height, valid_nodes);
        const Point start{static_cast<int64_t>(start_x), static_cast<int64_t>(start_y)};
        const Point goal{static_cast<int64_t>(goal_x), static_cast<int64_t>(goal_y)};
        if (!planner.valid(start) || !planner.valid(goal)) {
            result->stop_reason = PP_SEARCH_STOP_NO_PROGRESS;
            return 0;
        }
        int status = 0;
        try {
            status = planner.plan(
                start,
                goal,
                has_max_expansions != 0,
                max_expansions,
                result,
                trace
            );
        } catch (...) {
            metrics->motion_checks = planner.motion_checks();
            throw;
        }
        metrics->motion_checks = planner.motion_checks();
        metrics->jump_points_expanded = result->iters;
        return status;
    } catch (const std::exception& error) {
        set_error(result, error.what());
        return 1;
    } catch (...) {
        set_error(result, "unknown native JPS error");
        return 1;
    }
}

}  // namespace

extern "C" int pp_native_jps_grid(
    uint64_t width,
    uint64_t height,
    const uint8_t* valid_nodes,
    uint64_t start_x,
    uint64_t start_y,
    uint64_t goal_x,
    uint64_t goal_y,
    int has_max_expansions,
    uint64_t max_expansions,
    pp_search_result* result,
    pp_jps_metrics* metrics
) {
    if (result == nullptr) {
        return 1;
    }
    return run_jps_grid(
        width,
        height,
        valid_nodes,
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
extern "C" int pp_native_jps_grid_traced(
    uint64_t width,
    uint64_t height,
    const uint8_t* valid_nodes,
    uint64_t start_x,
    uint64_t start_y,
    uint64_t goal_x,
    uint64_t goal_y,
    int has_max_expansions,
    uint64_t max_expansions,
    uint64_t trace_max_bytes,
    pp_search_result* result,
    pp_trace_result* trace,
    pp_jps_metrics* metrics
) {
    if (result == nullptr || trace == nullptr) {
        return 1;
    }
    *trace = pp_trace_result{};
    const size_t event_capacity = static_cast<size_t>(trace_max_bytes / sizeof(pp_trace_event));
    JpsTraceBuffer captured(event_capacity);
    const int status = run_jps_grid(
        width,
        height,
        valid_nodes,
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
            set_error(result, "could not allocate JPS trace events");
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
