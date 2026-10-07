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

struct JpswPoint {
    int64_t x;
    int64_t y;

    bool operator==(const JpswPoint& other) const {
        return x == other.x && y == other.y;
    }
};

struct JpswDirection {
    int64_t x;
    int64_t y;
};

constexpr std::array<JpswDirection, 8> kJpswDirections = {{
    {-1, 0}, {-1, 1}, {0, 1}, {1, 1},
    {1, 0}, {1, -1}, {0, -1}, {-1, -1},
}};
constexpr uint64_t kJpswNoParent = std::numeric_limits<uint64_t>::max();
constexpr double kJpswInfinity = std::numeric_limits<double>::infinity();

struct JpswOpenEntry {
    double f;
    double g;
    uint64_t id;
};

struct JpswOpenGreater {
    bool operator()(const JpswOpenEntry& left, const JpswOpenEntry& right) const {
        return std::tie(left.f, left.g, left.id) > std::tie(right.f, right.g, right.id);
    }
};

struct JpswLocalEntry {
    double cost;
    double last_move_length;
    JpswPoint point;
};

struct JpswLocalGreater {
    bool operator()(const JpswLocalEntry& left, const JpswLocalEntry& right) const {
        return std::tie(left.cost, left.last_move_length, left.point.y, left.point.x) >
            std::tie(right.cost, right.last_move_length, right.point.y, right.point.x);
    }
};

struct JpswTraceEvent {
    uint64_t node;
    uint64_t parent;
    double value;
    uint32_t kind;
    uint32_t side;
};

class JpswTraceBuffer {
public:
    explicit JpswTraceBuffer(size_t max_events) : max_events_(max_events) {}

    void record(uint64_t node, uint64_t parent, double value, uint32_t kind) {
        if (events_.size() >= max_events_) {
            truncated_ = true;
            return;
        }
        events_.push_back({node, parent, value, kind, 0});
    }

    const std::vector<JpswTraceEvent>& events() const { return events_; }
    bool truncated() const { return truncated_; }

private:
    size_t max_events_;
    bool truncated_ = false;
    std::vector<JpswTraceEvent> events_;
};

struct JpswMetrics {
    uint64_t motion_checks = 0;
    uint64_t neighborhood_checks = 0;
    uint64_t prospective_prunes = 0;
    uint64_t jump_points_expanded = 0;
};

struct JpswJump {
    bool found = false;
    JpswPoint point{};
    double cost = kJpswInfinity;
};

class WeightedGridJps {
public:
    WeightedGridJps(
        uint64_t width,
        uint64_t height,
        const uint8_t* valid_nodes,
        const double* terrain_costs,
        double min_terrain_cost,
        JpswMetrics* metrics
    ) : width_(width),
        height_(height),
        valid_(valid_nodes),
        terrain_costs_(terrain_costs),
        min_terrain_cost_(min_terrain_cost),
        metrics_(metrics) {}

    bool valid(JpswPoint point) const {
        return point.x >= 0 && point.y >= 0 &&
            static_cast<uint64_t>(point.x) < width_ &&
            static_cast<uint64_t>(point.y) < height_ &&
            valid_[id(point)] != 0;
    }

    uint64_t id(JpswPoint point) const {
        return static_cast<uint64_t>(point.y) * width_ + static_cast<uint64_t>(point.x);
    }

    JpswPoint point(uint64_t node_id) const {
        return {
            static_cast<int64_t>(node_id % width_),
            static_cast<int64_t>(node_id / width_),
        };
    }

    bool legal_step(JpswPoint from, JpswPoint to) const {
        ++metrics_->motion_checks;
        if (!valid(to)) {
            return false;
        }
        const int64_t dx = to.x - from.x;
        const int64_t dy = to.y - from.y;
        if (std::max(std::abs(dx), std::abs(dy)) != 1) {
            return false;
        }
        if (dx != 0 && dy != 0) {
            return valid({from.x + dx, from.y}) && valid({from.x, from.y + dy});
        }
        return true;
    }

    double move_cost(JpswPoint from, JpswPoint to) const {
        const double from_cost = terrain_cost(from);
        const double to_cost = terrain_cost(to);
        const int64_t dx = to.x - from.x;
        const int64_t dy = to.y - from.y;
        if (dx != 0 && dy != 0) {
            return std::sqrt(2.0) * 0.25 * (
                from_cost + to_cost +
                terrain_cost({from.x + dx, from.y}) +
                terrain_cost({from.x, from.y + dy})
            );
        }
        return 0.5 * (from_cost + to_cost);
    }

    double heuristic(JpswPoint first, JpswPoint second) const {
        const double dx = std::abs(static_cast<double>(second.x - first.x));
        const double dy = std::abs(static_cast<double>(second.y - first.y));
        return min_terrain_cost_ * (
            std::max(dx, dy) + (std::sqrt(2.0) - 1.0) * std::min(dx, dy)
        );
    }

    std::vector<JpswDirection> neighborhood_successors(
        bool has_parent,
        JpswPoint parent,
        JpswPoint current
    ) const {
        ++metrics_->neighborhood_checks;
        std::vector<JpswDirection> successors;
        for (const JpswDirection direction : kJpswDirections) {
            const JpswPoint candidate{current.x + direction.x, current.y + direction.y};
            if (!legal_step(current, candidate)) {
                continue;
            }
            if (!has_parent) {
                successors.push_back(direction);
                continue;
            }
            if (candidate == parent) {
                continue;
            }
            const double via_cost = move_cost(parent, current) + move_cost(current, candidate);
            const double via_last = std::hypot(
                static_cast<double>(direction.x),
                static_cast<double>(direction.y)
            );
            const auto alternative = local_alternative(parent, current, candidate);
            if (!alternative.first ||
                alternative.second.first > via_cost ||
                (alternative.second.first == via_cost &&
                 alternative.second.second >= via_last)) {
                successors.push_back(direction);
            }
        }
        return successors;
    }

    bool has_terrain_boundary(JpswPoint center) const {
        ++metrics_->neighborhood_checks;
        const double center_cost = terrain_cost(center);
        for (int64_t dy = -1; dy <= 1; ++dy) {
            for (int64_t dx = -1; dx <= 1; ++dx) {
                const JpswPoint neighbor{center.x + dx, center.y + dy};
                if (!valid(neighbor) || terrain_cost(neighbor) != center_cost) {
                    return true;
                }
            }
        }
        return false;
    }

    JpswJump jump(
        JpswPoint origin,
        JpswDirection direction,
        JpswPoint goal,
        double origin_cost,
        const std::vector<double>& prospective_g,
        const std::vector<uint8_t>& prospective_orthogonal
    ) const {
        JpswPoint previous = origin;
        JpswPoint current{origin.x + direction.x, origin.y + direction.y};
        double traveled = 0.0;
        while (legal_step(previous, current)) {
            const double edge_cost = move_cost(previous, current);
            traveled += edge_cost;
            if (is_prospectively_pruned(
                    current,
                    direction,
                    origin_cost + traveled,
                    prospective_g,
                    prospective_orthogonal
                )) {
                ++metrics_->prospective_prunes;
                return {};
            }
            if (current == goal || has_terrain_boundary(current)) {
                return {true, current, traveled};
            }
            if (direction.x != 0 && direction.y != 0) {
                const auto successors = neighborhood_successors(true, previous, current);
                if (contains(successors, {direction.x, 0})) {
                    const JpswJump horizontal = straight_jump(
                        current,
                        {direction.x, 0},
                        goal,
                        origin_cost + traveled,
                        prospective_g,
                        prospective_orthogonal
                    );
                    if (horizontal.found) {
                        return {true, current, traveled};
                    }
                }
                if (contains(successors, {0, direction.y})) {
                    const JpswJump vertical = straight_jump(
                        current,
                        {0, direction.y},
                        goal,
                        origin_cost + traveled,
                        prospective_g,
                        prospective_orthogonal
                    );
                    if (vertical.found) {
                        return {true, current, traveled};
                    }
                }
            }
            previous = current;
            current = {current.x + direction.x, current.y + direction.y};
        }
        return {};
    }

    int plan(
        JpswPoint start,
        JpswPoint goal,
        bool has_max_expansions,
        uint64_t max_expansions,
        pp_search_result* result,
        JpswTraceBuffer* trace
    ) const {
        const size_t node_count = static_cast<size_t>(width_ * height_);
        const uint64_t start_id = id(start);
        const uint64_t goal_id = id(goal);
        std::vector<double> costs(node_count, kJpswInfinity);
        std::vector<double> prospective_g(node_count, kJpswInfinity);
        std::vector<uint8_t> prospective_orthogonal(node_count, 0);
        std::vector<uint64_t> parents(node_count, kJpswNoParent);
        std::vector<uint8_t> closed(node_count, 0);
        std::priority_queue<JpswOpenEntry, std::vector<JpswOpenEntry>, JpswOpenGreater> queue;
        costs[static_cast<size_t>(start_id)] = 0.0;
        queue.push({heuristic(start, goal), 0.0, start_id});
        if (trace != nullptr) {
            trace->record(start_id, kJpswNoParent, 0.0, 1);
        }
        update_prospective(
            false,
            {},
            start,
            0.0,
            prospective_g,
            prospective_orthogonal
        );
        uint64_t expanded = 0;
        uint64_t discovered = 1;

        while (!queue.empty()) {
            const JpswOpenEntry entry = queue.top();
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
                return finish({}, kJpswInfinity, expanded, discovered, PP_SEARCH_STOP_MAX_ITERS, result);
            }
            closed[current_index] = 1;
            ++expanded;
            ++metrics_->jump_points_expanded;
            if (trace != nullptr) {
                trace->record(
                    entry.id,
                    parents[current_index],
                    entry.g,
                    2
                );
            }
            const JpswPoint current = point(entry.id);
            const uint64_t parent_id = parents[current_index];
            const bool has_parent = parent_id != kJpswNoParent;
            JpswPoint local_parent{};
            if (has_parent) {
                const JpswPoint search_parent = point(parent_id);
                // Jump-point parents can be distant; pruning needs the preceding grid cell.
                const JpswDirection incoming{
                    (current.x > search_parent.x) - (current.x < search_parent.x),
                    (current.y > search_parent.y) - (current.y < search_parent.y),
                };
                local_parent = {current.x - incoming.x, current.y - incoming.y};
            }
            const auto directions = neighborhood_successors(has_parent, local_parent, current);
            for (const JpswDirection direction : directions) {
                const JpswJump found = jump(
                    current,
                    direction,
                    goal,
                    entry.g,
                    prospective_g,
                    prospective_orthogonal
                );
                if (!found.found) {
                    continue;
                }
                const uint64_t successor_id = id(found.point);
                const size_t successor_index = static_cast<size_t>(successor_id);
                if (closed[successor_index]) {
                    continue;
                }
                const double candidate = entry.g + found.cost;
                if (candidate < costs[successor_index]) {
                    const bool first_discovery = !std::isfinite(costs[successor_index]);
                    if (first_discovery) {
                        ++discovered;
                    }
                    costs[successor_index] = candidate;
                    parents[successor_index] = entry.id;
                    const JpswPoint predecessor{
                        found.point.x - direction.x,
                        found.point.y - direction.y,
                    };
                    update_prospective(
                        true,
                        predecessor,
                        found.point,
                        candidate,
                        prospective_g,
                        prospective_orthogonal
                    );
                    if (trace != nullptr) {
                        if (first_discovery) {
                            trace->record(successor_id, entry.id, candidate, 1);
                        }
                        trace->record(successor_id, entry.id, candidate, 3);
                    }
                    queue.push({candidate + heuristic(found.point, goal), candidate, successor_id});
                }
            }
        }
        return finish({}, kJpswInfinity, expanded, discovered, PP_SEARCH_STOP_NO_PROGRESS, result);
    }

private:
    double terrain_cost(JpswPoint point) const {
        return terrain_costs_[id(point)];
    }

    std::pair<bool, std::pair<double, double>> local_alternative(
        JpswPoint parent,
        JpswPoint center,
        JpswPoint target
    ) const {
        if (std::max(std::abs(parent.x - center.x), std::abs(parent.y - center.y)) != 1 ||
            std::max(std::abs(target.x - center.x), std::abs(target.y - center.y)) != 1) {
            throw std::logic_error("JPSW neighborhood candidates must be adjacent to center");
        }
        std::array<double, 9> costs;
        std::array<double, 9> last_lengths;
        costs.fill(kJpswInfinity);
        last_lengths.fill(kJpswInfinity);
        auto local_index = [center](JpswPoint point) -> size_t {
            return static_cast<size_t>(point.y - center.y + 1) * 3 +
                static_cast<size_t>(point.x - center.x + 1);
        };
        std::priority_queue<JpswLocalEntry, std::vector<JpswLocalEntry>, JpswLocalGreater> queue;
        const size_t parent_index = local_index(parent);
        costs[parent_index] = 0.0;
        last_lengths[parent_index] = 0.0;
        queue.push({0.0, 0.0, parent});
        while (!queue.empty()) {
            const JpswLocalEntry entry = queue.top();
            queue.pop();
            const size_t index = local_index(entry.point);
            if (entry.cost != costs[index] || entry.last_move_length != last_lengths[index]) {
                continue;
            }
            if (entry.point == target) {
                return {true, {entry.cost, entry.last_move_length}};
            }
            for (const JpswDirection direction : kJpswDirections) {
                const JpswPoint next{entry.point.x + direction.x, entry.point.y + direction.y};
                if (next == center ||
                    std::max(std::abs(next.x - center.x), std::abs(next.y - center.y)) > 1 ||
                    !legal_step(entry.point, next)) {
                    continue;
                }
                const double candidate = entry.cost + move_cost(entry.point, next);
                const double last_move = std::hypot(
                    static_cast<double>(direction.x),
                    static_cast<double>(direction.y)
                );
                const size_t next_index = local_index(next);
                if (candidate < costs[next_index] ||
                    (candidate == costs[next_index] && last_move < last_lengths[next_index])) {
                    costs[next_index] = candidate;
                    last_lengths[next_index] = last_move;
                    queue.push({candidate, last_move, next});
                }
            }
        }
        return {false, {kJpswInfinity, kJpswInfinity}};
    }

    bool is_prospectively_pruned(
        JpswPoint target,
        JpswDirection direction,
        double candidate_cost,
        const std::vector<double>& prospective_g,
        const std::vector<uint8_t>& prospective_orthogonal
    ) const {
        const size_t target_index = static_cast<size_t>(id(target));
        const double best = prospective_g[target_index];
        if (best < candidate_cost) {
            return true;
        }
        const bool diagonal = direction.x != 0 && direction.y != 0;
        return diagonal && prospective_orthogonal[target_index] != 0 && best == candidate_cost;
    }

    void update_prospective(
        bool has_parent,
        JpswPoint parent,
        JpswPoint current,
        double current_cost,
        std::vector<double>& prospective_g,
        std::vector<uint8_t>& prospective_orthogonal
    ) const {
        const auto directions = neighborhood_successors(has_parent, parent, current);
        for (const JpswDirection direction : directions) {
            const JpswPoint next{current.x + direction.x, current.y + direction.y};
            const size_t next_index = static_cast<size_t>(id(next));
            const double candidate = current_cost + move_cost(current, next);
            const bool orthogonal = direction.x == 0 || direction.y == 0;
            if (candidate < prospective_g[next_index]) {
                prospective_g[next_index] = candidate;
                prospective_orthogonal[next_index] = orthogonal ? 1 : 0;
            } else if (candidate == prospective_g[next_index] && orthogonal) {
                prospective_orthogonal[next_index] = 1;
            }
        }
    }

    JpswJump straight_jump(
        JpswPoint origin,
        JpswDirection direction,
        JpswPoint goal,
        double origin_cost,
        const std::vector<double>& prospective_g,
        const std::vector<uint8_t>& prospective_orthogonal
    ) const {
        JpswPoint previous = origin;
        JpswPoint current{origin.x + direction.x, origin.y + direction.y};
        double traveled = 0.0;
        while (legal_step(previous, current)) {
            traveled += move_cost(previous, current);
            if (is_prospectively_pruned(
                    current,
                    direction,
                    origin_cost + traveled,
                    prospective_g,
                    prospective_orthogonal
                )) {
                ++metrics_->prospective_prunes;
                return {};
            }
            if (current == goal || has_terrain_boundary(current)) {
                return {true, current, traveled};
            }
            previous = current;
            current = {current.x + direction.x, current.y + direction.y};
        }
        return {};
    }

    static bool contains(const std::vector<JpswDirection>& directions, JpswDirection target) {
        return std::any_of(
            directions.begin(),
            directions.end(),
            [target](JpswDirection direction) {
                return direction.x == target.x && direction.y == target.y;
            }
        );
    }

    std::vector<uint64_t> reconstruct_path(
        uint64_t start_id,
        uint64_t goal_id,
        const std::vector<uint64_t>& parents
    ) const {
        std::vector<uint64_t> jump_path;
        uint64_t current = goal_id;
        while (current != kJpswNoParent) {
            jump_path.push_back(current);
            if (current == start_id) {
                break;
            }
            current = parents[static_cast<size_t>(current)];
        }
        if (jump_path.empty() || jump_path.back() != start_id) {
            throw std::runtime_error("JPSW parent chain did not reach the start");
        }
        std::reverse(jump_path.begin(), jump_path.end());
        std::vector<uint64_t> path{start_id};
        for (size_t index = 1; index < jump_path.size(); ++index) {
            JpswPoint from = point(jump_path[index - 1]);
            const JpswPoint to = point(jump_path[index]);
            const JpswDirection step{
                (to.x > from.x) - (to.x < from.x),
                (to.y > from.y) - (to.y < from.y),
            };
            while (!(from == to)) {
                const JpswPoint next{from.x + step.x, from.y + step.y};
                if (!legal_step(from, next)) {
                    throw std::runtime_error("JPSW produced an invalid grid segment");
                }
                path.push_back(id(next));
                from = next;
            }
        }
        return path;
    }

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
    const double* terrain_costs_;
    double min_terrain_cost_;
    JpswMetrics* metrics_;
};

void reset_result(pp_search_result* result) {
    *result = pp_search_result{};
    result->stop_reason = PP_SEARCH_STOP_ERROR;
    result->path_cost = kJpswInfinity;
}

void set_error(pp_search_result* result, const std::string& message) {
    reset_result(result);
    result->error_message = static_cast<char*>(std::malloc(message.size() + 1));
    if (result->error_message != nullptr) {
        std::memcpy(result->error_message, message.c_str(), message.size() + 1);
    }
}

int run_jpsw_grid(
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
    pp_jpsw_metrics* output_metrics,
    JpswTraceBuffer* trace
) {
    if (output_metrics == nullptr || output_metrics->struct_size != sizeof(pp_jpsw_metrics)) {
        set_error(result, "JPSW metrics struct size mismatch");
        return 1;
    }
    output_metrics->motion_checks = 0;
    output_metrics->neighborhood_checks = 0;
    output_metrics->prospective_prunes = 0;
    output_metrics->jump_points_expanded = 0;
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
        double min_terrain_cost = kJpswInfinity;
        for (size_t index = 0; index < node_count; ++index) {
            if (!std::isfinite(terrain_costs[index]) || terrain_costs[index] <= 0.0) {
                throw std::invalid_argument("terrain costs must be finite and strictly positive");
            }
            min_terrain_cost = std::min(min_terrain_cost, terrain_costs[index]);
        }
        JpswMetrics metrics;
        const WeightedGridJps search(
            width,
            height,
            valid_nodes,
            terrain_costs,
            min_terrain_cost,
            &metrics
        );
        const JpswPoint start{static_cast<int64_t>(start_x), static_cast<int64_t>(start_y)};
        const JpswPoint goal{static_cast<int64_t>(goal_x), static_cast<int64_t>(goal_y)};
        if (!search.valid(start) || !search.valid(goal)) {
            result->stop_reason = PP_SEARCH_STOP_NO_PROGRESS;
            return 0;
        }
        const int status = search.plan(
            start,
            goal,
            has_max_expansions != 0,
            max_expansions,
            result,
            trace
        );
        output_metrics->motion_checks = metrics.motion_checks;
        output_metrics->neighborhood_checks = metrics.neighborhood_checks;
        output_metrics->prospective_prunes = metrics.prospective_prunes;
        output_metrics->jump_points_expanded = metrics.jump_points_expanded;
        return status;
    } catch (const std::exception& error) {
        set_error(result, error.what());
        return 1;
    } catch (...) {
        set_error(result, "unknown native JPSW error");
        return 1;
    }
}

}  // namespace

extern "C" int pp_native_jpsw_grid(
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
    pp_jpsw_metrics* metrics
) {
    if (result == nullptr) {
        return 1;
    }
    return run_jpsw_grid(
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
extern "C" int pp_native_jpsw_grid_traced(
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
    pp_jpsw_metrics* metrics
) {
    if (result == nullptr || trace == nullptr) {
        return 1;
    }
    *trace = pp_trace_result{};
    const size_t event_capacity = static_cast<size_t>(trace_max_bytes / sizeof(pp_trace_event));
    JpswTraceBuffer captured(event_capacity);
    const int status = run_jpsw_grid(
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
            set_error(result, "could not allocate JPSW trace events");
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
