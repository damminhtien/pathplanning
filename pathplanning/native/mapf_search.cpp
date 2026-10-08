#include "search_engine.h"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdlib>
#include <cstring>
#include <limits>
#include <map>
#include <set>
#include <sstream>
#include <stdexcept>
#include <string>
#include <tuple>
#include <unordered_map>
#include <unordered_set>
#include <utility>
#include <vector>

namespace {

constexpr uint64_t kInfinity = std::numeric_limits<uint64_t>::max();
using Clock = std::chrono::steady_clock;

struct TraceBuffer {
    explicit TraceBuffer(uint64_t max_bytes) : max_events_(max_bytes / sizeof(pp_trace_event)) {}

    void record(uint32_t kind, uint64_t node, uint64_t parent, double value) {
        if (events_.size() >= max_events_) {
            truncated_ = true;
            return;
        }
        events_.push_back({node, parent, value, kind, 0});
    }

    int copy(pp_trace_result* trace) const {
        if (trace == nullptr) {
            return 1;
        }
        *trace = pp_trace_result{};
        if (!events_.empty()) {
            trace->events = static_cast<pp_trace_event*>(
                std::malloc(events_.size() * sizeof(pp_trace_event))
            );
            if (trace->events == nullptr) {
                return 1;
            }
            std::memcpy(trace->events, events_.data(), events_.size() * sizeof(pp_trace_event));
        }
        trace->event_count = events_.size();
        trace->truncated = truncated_ ? 1 : 0;
        return 0;
    }

private:
    size_t max_events_;
    bool truncated_ = false;
    std::vector<pp_trace_event> events_;
};

struct SearchBudget {
    SearchBudget(int has_limit, uint64_t limit, double runtime_ms)
        : has_expansion_limit(has_limit != 0), max_expansions(limit),
          max_runtime_ms(runtime_ms), started(Clock::now()) {}

    bool can_expand() {
        if (has_expansion_limit && expansions >= max_expansions) {
            expansion_limited = true;
            return false;
        }
        if (max_runtime_ms > 0.0 && std::chrono::duration<double, std::milli>(
                Clock::now() - started).count() >= max_runtime_ms) {
            time_limited = true;
            return false;
        }
        return true;
    }

    void expanded(bool high_level) {
        ++expansions;
        if (high_level) {
            ++high_level_expanded;
        } else {
            ++low_level_expanded;
        }
    }

    void discovered(uint64_t count = 1) { nodes += count; }

    bool has_expansion_limit;
    uint64_t max_expansions;
    double max_runtime_ms;
    Clock::time_point started;
    uint64_t expansions = 0;
    uint64_t nodes = 0;
    uint64_t high_level_expanded = 0;
    uint64_t low_level_expanded = 0;
    bool expansion_limited = false;
    bool time_limited = false;
};

struct EdgeConstraint {
    uint64_t from;
    uint64_t to;
    uint64_t time;

    bool operator<(const EdgeConstraint& other) const {
        return std::tie(from, to, time) < std::tie(other.from, other.to, other.time);
    }
};

struct AgentConstraints {
    std::set<std::pair<uint64_t, uint64_t>> vertices;
    std::set<EdgeConstraint> edges;
};

struct LowLevelState {
    uint64_t node;
    uint64_t time;
    uint64_t parent;
    uint64_t conflicts;
    uint64_t f;
    bool active;
};

struct LowLevelResult {
    bool found = false;
    bool interrupted = false;
    uint64_t f_min = 0;
    std::vector<uint64_t> path;
};

struct Conflict {
    bool edge_swap;
    size_t first_agent;
    size_t second_agent;
    uint64_t first_node;
    uint64_t second_node;
    uint64_t time;
};

struct ConstraintTreeNode {
    uint64_t id;
    uint64_t parent;
    std::vector<AgentConstraints> constraints;
    std::vector<std::vector<uint64_t>> paths;
    std::vector<uint64_t> path_lower_bounds;
    uint64_t cost;
    uint64_t lower_bound;
    uint64_t conflicts;
    double estimated_cost;
    bool active;
};

uint64_t saturating_add(uint64_t left, uint64_t right) {
    if (left > kInfinity - right) {
        return kInfinity;
    }
    return left + right;
}

uint64_t path_cost(const std::vector<uint64_t>& path) {
    return path.empty() ? 0 : static_cast<uint64_t>(path.size() - 1);
}

uint64_t path_node(const std::vector<uint64_t>& path, uint64_t time) {
    return path[std::min<size_t>(static_cast<size_t>(time), path.size() - 1)];
}

std::vector<std::vector<uint64_t>> shortest_distances(
    const std::vector<std::vector<uint64_t>>& adjacency,
    const std::vector<uint64_t>& goals
) {
    std::vector<std::vector<uint64_t>> distances;
    distances.reserve(goals.size());
    for (uint64_t goal : goals) {
        std::vector<uint64_t> distance(adjacency.size(), kInfinity);
        std::vector<uint64_t> queue;
        distance[static_cast<size_t>(goal)] = 0;
        queue.push_back(goal);
        for (size_t cursor = 0; cursor < queue.size(); ++cursor) {
            const uint64_t node = queue[cursor];
            for (uint64_t neighbor : adjacency[static_cast<size_t>(node)]) {
                uint64_t& candidate = distance[static_cast<size_t>(neighbor)];
                if (candidate == kInfinity) {
                    candidate = distance[static_cast<size_t>(node)] + 1;
                    queue.push_back(neighbor);
                }
            }
        }
        distances.push_back(std::move(distance));
    }
    return distances;
}

uint64_t other_path_conflicts(
    uint64_t node,
    uint64_t parent_node,
    uint64_t time,
    size_t agent,
    const std::vector<std::vector<uint64_t>>& paths
) {
    uint64_t conflicts = 0;
    for (size_t other = 0; other < paths.size(); ++other) {
        if (other == agent || paths[other].empty()) {
            continue;
        }
        if (path_node(paths[other], time) == node) {
            ++conflicts;
        }
        if (time > 0 && parent_node != node &&
            path_node(paths[other], time - 1) == node &&
            path_node(paths[other], time) == parent_node) {
            ++conflicts;
        }
    }
    return conflicts;
}

bool has_future_goal_constraint(const AgentConstraints& constraints, uint64_t goal, uint64_t time) {
    auto it = constraints.vertices.lower_bound({goal, time});
    return it != constraints.vertices.end() && it->first == goal;
}

LowLevelResult low_level_focal_search(
    size_t agent,
    uint64_t start,
    uint64_t goal,
    const AgentConstraints& constraints,
    const std::vector<std::vector<uint64_t>>& adjacency,
    const std::vector<uint64_t>& heuristic,
    const std::vector<std::vector<uint64_t>>& other_paths,
    double weight,
    SearchBudget* budget,
    TraceBuffer* trace
) {
    LowLevelResult result;
    if (constraints.vertices.count({start, 0}) != 0) {
        return result;
    }
    if (heuristic[static_cast<size_t>(start)] == kInfinity) {
        return result;
    }

    uint64_t latest_constraint = 0;
    for (const auto& [node, time] : constraints.vertices) {
        (void)node;
        latest_constraint = std::max(latest_constraint, time);
    }
    for (const EdgeConstraint& edge : constraints.edges) {
        latest_constraint = std::max(latest_constraint, edge.time);
    }
    const uint64_t horizon = saturating_add(latest_constraint, adjacency.size() + 1);

    std::vector<LowLevelState> states;
    std::map<std::pair<uint64_t, uint64_t>, uint64_t> state_ids;
    std::vector<uint64_t> open;
    auto add_state = [&](uint64_t node, uint64_t time, uint64_t parent, uint64_t conflicts) {
        const uint64_t f = saturating_add(time, heuristic[static_cast<size_t>(node)]);
        const auto key = std::make_pair(node, time);
        const auto existing = state_ids.find(key);
        if (existing != state_ids.end()) {
            LowLevelState& state = states[static_cast<size_t>(existing->second)];
            if (conflicts < state.conflicts) {
                state.conflicts = conflicts;
                state.parent = parent;
                if (!state.active) {
                    state.active = true;
                    open.push_back(existing->second);
                }
            }
            return existing->second;
        }
        const uint64_t id = static_cast<uint64_t>(states.size());
        states.push_back({node, time, parent, conflicts, f, true});
        state_ids.emplace(key, id);
        open.push_back(id);
        budget->discovered();
        return id;
    };

    add_state(start, 0, kInfinity, 0);
    while (!open.empty()) {
        if (!budget->can_expand()) {
            result.interrupted = true;
            return result;
        }
        uint64_t min_f = kInfinity;
        for (uint64_t id : open) {
            const LowLevelState& state = states[static_cast<size_t>(id)];
            if (state.active) {
                min_f = std::min(min_f, state.f);
            }
        }
        if (min_f == kInfinity) {
            break;
        }
        const double focal_bound = weight * static_cast<double>(min_f);
        uint64_t selected = kInfinity;
        for (uint64_t id : open) {
            const LowLevelState& state = states[static_cast<size_t>(id)];
            if (!state.active || static_cast<double>(state.f) > focal_bound) {
                continue;
            }
            if (selected == kInfinity) {
                selected = id;
                continue;
            }
            const LowLevelState& best = states[static_cast<size_t>(selected)];
            if (std::tie(state.conflicts, state.f, state.time, state.node) <
                std::tie(best.conflicts, best.f, best.time, best.node)) {
                selected = id;
            }
        }
        if (selected == kInfinity) {
            throw std::runtime_error("EECBS focal list is unexpectedly empty");
        }
        LowLevelState& state = states[static_cast<size_t>(selected)];
        state.active = false;
        budget->expanded(false);
        if (trace != nullptr) {
            const uint64_t parent_node = state.parent == kInfinity ? kInfinity :
                states[static_cast<size_t>(state.parent)].node;
            trace->record(PP_TRACE_EXPAND, state.node, parent_node, static_cast<double>(state.f));
        }
        if (state.node == goal && !has_future_goal_constraint(constraints, goal, state.time)) {
            result.found = true;
            result.f_min = min_f;
            uint64_t cursor = selected;
            while (cursor != kInfinity) {
                const LowLevelState& path_state = states[static_cast<size_t>(cursor)];
                result.path.push_back(path_state.node);
                cursor = path_state.parent;
            }
            std::reverse(result.path.begin(), result.path.end());
            return result;
        }
        const uint64_t current_node = state.node;
        const uint64_t current_time = state.time;
        const uint64_t parent_id = selected;
        const uint64_t prior_conflicts = state.conflicts;
        if (current_time >= horizon) {
            continue;
        }
        auto relax = [&](uint64_t target, bool wait_action) {
            const uint64_t next_time = current_time + 1;
            if (constraints.vertices.count({target, next_time}) != 0) {
                return;
            }
            if (!wait_action && constraints.edges.count({current_node, target, current_time}) != 0) {
                return;
            }
            const auto key = std::make_pair(target, next_time);
            const uint64_t conflicts = saturating_add(
                prior_conflicts,
                other_path_conflicts(target, current_node, next_time, agent, other_paths)
            );
            if (state_ids.count(key) == 0 ||
                conflicts < states[static_cast<size_t>(state_ids.at(key))].conflicts) {
                add_state(target, next_time, parent_id, conflicts);
            }
        };

        relax(current_node, true);
        for (uint64_t neighbor : adjacency[static_cast<size_t>(current_node)]) {
            relax(neighbor, false);
        }
    }
    return result;
}

bool find_conflict(
    const std::vector<std::vector<uint64_t>>& paths,
    Conflict* conflict,
    uint64_t* total_conflicts
) {
    size_t max_path_length = 0;
    for (const auto& path : paths) {
        max_path_length = std::max(max_path_length, path.size());
    }
    uint64_t count = 0;
    for (uint64_t time = 0; time < max_path_length; ++time) {
        for (size_t first = 0; first < paths.size(); ++first) {
            for (size_t second = first + 1; second < paths.size(); ++second) {
                const uint64_t first_node = path_node(paths[first], time);
                const uint64_t second_node = path_node(paths[second], time);
                if (first_node == second_node) {
                    ++count;
                    if (conflict != nullptr && *total_conflicts == 0) {
                        *conflict = {false, first, second, first_node, second_node, time};
                    }
                }
                if (time > 0) {
                    const uint64_t first_previous = path_node(paths[first], time - 1);
                    const uint64_t second_previous = path_node(paths[second], time - 1);
                    if (first_previous != first_node && first_previous == second_node &&
                        second_previous == first_node) {
                        ++count;
                        if (conflict != nullptr && *total_conflicts == 0) {
                            *conflict = {
                                true, first, second, first_previous, first_node, time - 1
                            };
                        }
                    }
                }
            }
        }
    }
    *total_conflicts = count;
    return count != 0;
}

uint64_t sum_path_costs(const std::vector<std::vector<uint64_t>>& paths) {
    uint64_t total = 0;
    for (const auto& path : paths) {
        total = saturating_add(total, path_cost(path));
    }
    return total;
}

uint64_t paths_makespan(const std::vector<std::vector<uint64_t>>& paths) {
    uint64_t maximum = 0;
    for (const auto& path : paths) {
        maximum = std::max(maximum, path_cost(path));
    }
    return maximum;
}

uint64_t lower_bound_sum(const std::vector<uint64_t>& lower_bounds) {
    uint64_t total = 0;
    for (uint64_t value : lower_bounds) {
        total = saturating_add(total, value);
    }
    return total;
}

std::string constraints_key(const std::vector<AgentConstraints>& constraints) {
    std::ostringstream key;
    for (size_t agent = 0; agent < constraints.size(); ++agent) {
        key << agent << ':';
        for (const auto& [node, time] : constraints[agent].vertices) {
            key << 'v' << node << ',' << time << ';';
        }
        for (const EdgeConstraint& edge : constraints[agent].edges) {
            key << 'e' << edge.from << ',' << edge.to << ',' << edge.time << ';';
        }
        key << '|';
    }
    return key.str();
}

void set_result_error(pp_mapf_result* result, const std::string& message) {
    if (result == nullptr) {
        return;
    }
    std::free(result->error_message);
    result->error_message = static_cast<char*>(std::malloc(message.size() + 1));
    if (result->error_message != nullptr) {
        std::memcpy(result->error_message, message.c_str(), message.size() + 1);
    }
}

int validate_inputs(
    uint64_t node_count,
    uint64_t edge_count,
    const uint64_t* offsets,
    const uint64_t* neighbor_ids,
    const uint64_t* starts,
    const uint64_t* goals,
    size_t agent_count,
    double weight,
    int has_max_expansions,
    uint64_t max_expansions,
    double max_runtime_ms
) {
    if (node_count == 0 || offsets == nullptr || starts == nullptr || goals == nullptr ||
        agent_count == 0 || offsets[0] != 0 || offsets[node_count] != edge_count ||
        (edge_count > 0 && neighbor_ids == nullptr)) {
        throw std::invalid_argument("EECBS graph and agent arrays are invalid");
    }
    if (!std::isfinite(weight) || weight < 1.0) {
        throw std::invalid_argument("EECBS suboptimality weight must be finite and at least 1");
    }
    if ((has_max_expansions != 0 && has_max_expansions != 1) ||
        (has_max_expansions != 0 && max_expansions == 0)) {
        throw std::invalid_argument("max_expansions must be positive when enabled");
    }
    if (!std::isfinite(max_runtime_ms) || max_runtime_ms < 0.0) {
        throw std::invalid_argument("max_runtime_ms must be finite and non-negative");
    }
    for (uint64_t node = 0; node < node_count; ++node) {
        if (offsets[node] > offsets[node + 1]) {
            throw std::invalid_argument("EECBS CSR offsets must be monotone");
        }
    }
    for (uint64_t edge = 0; edge < edge_count; ++edge) {
        if (neighbor_ids[edge] >= node_count) {
            throw std::invalid_argument("EECBS edge endpoint is outside the graph");
        }
    }
    for (size_t agent = 0; agent < agent_count; ++agent) {
        if (starts[agent] >= node_count || goals[agent] >= node_count) {
            throw std::invalid_argument("EECBS start or goal is outside the graph");
        }
    }
    std::set<uint64_t> unique_starts(starts, starts + agent_count);
    std::set<uint64_t> unique_goals(goals, goals + agent_count);
    if (unique_starts.size() != agent_count || unique_goals.size() != agent_count) {
        throw std::invalid_argument("EECBS starts and goals must be unique");
    }
    return 0;
}

void copy_solution(
    const std::vector<std::vector<uint64_t>>& paths,
    pp_mapf_result* result
) {
    size_t total_nodes = 0;
    for (const auto& path : paths) {
        total_nodes += path.size();
    }
    result->path_offsets = static_cast<uint64_t*>(
        std::malloc((paths.size() + 1) * sizeof(uint64_t))
    );
    result->path_nodes = static_cast<uint64_t*>(std::malloc(total_nodes * sizeof(uint64_t)));
    if (result->path_offsets == nullptr || (total_nodes > 0 && result->path_nodes == nullptr)) {
        std::free(result->path_offsets);
        std::free(result->path_nodes);
        result->path_offsets = nullptr;
        result->path_nodes = nullptr;
        throw std::bad_alloc();
    }
    size_t offset = 0;
    result->path_offsets[0] = 0;
    for (size_t agent = 0; agent < paths.size(); ++agent) {
        std::copy(paths[agent].begin(), paths[agent].end(), result->path_nodes + offset);
        offset += paths[agent].size();
        result->path_offsets[agent + 1] = static_cast<uint64_t>(offset);
    }
    result->agent_count = paths.size();
    result->path_node_count = total_nodes;
    result->sum_of_costs = sum_path_costs(paths);
    result->makespan = paths_makespan(paths);
    result->success = 1;
}

int run_eecbs(
    uint64_t node_count,
    uint64_t edge_count,
    const uint64_t* offsets,
    const uint64_t* neighbor_ids,
    const uint64_t* starts,
    const uint64_t* goals,
    size_t agent_count,
    double weight,
    int has_max_expansions,
    uint64_t max_expansions,
    double max_runtime_ms,
    pp_mapf_result* result,
    TraceBuffer* trace
) {
    if (result == nullptr) {
        return 1;
    }
    *result = pp_mapf_result{};
    try {
        validate_inputs(
            node_count, edge_count, offsets, neighbor_ids, starts, goals, agent_count,
            weight, has_max_expansions, max_expansions, max_runtime_ms
        );
        SearchBudget budget(has_max_expansions, max_expansions, max_runtime_ms);
        std::vector<std::vector<uint64_t>> adjacency(static_cast<size_t>(node_count));
        for (uint64_t node = 0; node < node_count; ++node) {
            for (uint64_t edge = offsets[node]; edge < offsets[node + 1]; ++edge) {
                const uint64_t neighbor = neighbor_ids[edge];
                if (node == neighbor) {
                    throw std::invalid_argument("MAPF graph must not contain self-loops");
                }
                adjacency[static_cast<size_t>(node)].push_back(neighbor);
            }
        }
        for (uint64_t node = 0; node < node_count; ++node) {
            auto& row = adjacency[static_cast<size_t>(node)];
            std::sort(row.begin(), row.end());
            if (std::adjacent_find(row.begin(), row.end()) != row.end()) {
                throw std::invalid_argument("MAPF graph must not contain duplicate edges");
            }
            for (uint64_t neighbor : row) {
                const auto& reverse = adjacency[static_cast<size_t>(neighbor)];
                if (std::find(reverse.begin(), reverse.end(), node) == reverse.end()) {
                    throw std::invalid_argument("MAPF graph must be undirected");
                }
            }
        }
        const std::vector<uint64_t> goal_vector(goals, goals + agent_count);
        const auto heuristics = shortest_distances(adjacency, goal_vector);
        std::vector<ConstraintTreeNode> nodes;
        std::vector<uint64_t> open;
        std::unordered_set<std::string> explored;
        std::vector<std::vector<uint64_t>> incumbent;
        uint64_t incumbent_cost = kInfinity;
        double average_distance_error = 0.0;
        double average_cost_error = 0.0;
        uint64_t error_samples = 0;
        std::vector<std::vector<uint64_t>> root_paths(agent_count);
        std::vector<uint64_t> root_lower_bounds(agent_count);
        std::vector<AgentConstraints> empty_constraints(agent_count);
        for (size_t agent = 0; agent < agent_count; ++agent) {
            LowLevelResult low = low_level_focal_search(
                agent, starts[agent], goals[agent], empty_constraints[agent], adjacency,
                heuristics[agent], root_paths, weight, &budget, trace
            );
            if (low.interrupted) {
                result->stop_reason = budget.time_limited ? PP_SEARCH_STOP_TIME_BUDGET :
                    PP_SEARCH_STOP_MAX_ITERS;
                result->iters = budget.expansions;
                result->nodes = budget.nodes;
                result->high_level_expanded = budget.high_level_expanded;
                result->low_level_expanded = budget.low_level_expanded;
                return 0;
            }
            if (!low.found) {
                result->stop_reason = PP_SEARCH_STOP_NO_PROGRESS;
                result->iters = budget.expansions;
                result->nodes = budget.nodes;
                result->high_level_expanded = budget.high_level_expanded;
                result->low_level_expanded = budget.low_level_expanded;
                return 0;
            }
            root_paths[agent] = std::move(low.path);
            root_lower_bounds[agent] = low.f_min;
        }
        ConstraintTreeNode root{
            0, kInfinity, std::move(empty_constraints), std::move(root_paths),
            std::move(root_lower_bounds), 0, 0, 0, 0.0, true
        };
        root.cost = sum_path_costs(root.paths);
        root.lower_bound = lower_bound_sum(root.path_lower_bounds);
        find_conflict(root.paths, nullptr, &root.conflicts);
        root.estimated_cost = static_cast<double>(root.cost);
        if (root.conflicts == 0) {
            incumbent = root.paths;
            incumbent_cost = root.cost;
        }
        nodes.push_back(std::move(root));
        open.push_back(0);
        explored.insert(constraints_key(nodes[0].constraints));
        budget.discovered();
        bool exhausted = false;

        while (!open.empty()) {
            if (!budget.can_expand()) {
                break;
            }
            size_t cleanup_index = open.size();
            size_t estimate_index = open.size();
            for (size_t index = 0; index < open.size(); ++index) {
                const ConstraintTreeNode& node = nodes[static_cast<size_t>(open[index])];
                if (!node.active) {
                    continue;
                }
                if (cleanup_index == open.size() ||
                    std::tie(node.lower_bound, node.cost, node.conflicts, node.id) <
                    std::tie(
                        nodes[static_cast<size_t>(open[cleanup_index])].lower_bound,
                        nodes[static_cast<size_t>(open[cleanup_index])].cost,
                        nodes[static_cast<size_t>(open[cleanup_index])].conflicts,
                        nodes[static_cast<size_t>(open[cleanup_index])].id
                    )) {
                    cleanup_index = index;
                }
                if (estimate_index == open.size() ||
                    std::tie(node.estimated_cost, node.lower_bound, node.id) <
                    std::tie(
                        nodes[static_cast<size_t>(open[estimate_index])].estimated_cost,
                        nodes[static_cast<size_t>(open[estimate_index])].lower_bound,
                        nodes[static_cast<size_t>(open[estimate_index])].id
                    )) {
                    estimate_index = index;
                }
            }
            if (cleanup_index == open.size()) {
                exhausted = true;
                break;
            }
            const ConstraintTreeNode& best_lower = nodes[static_cast<size_t>(open[cleanup_index])];
            const ConstraintTreeNode& best_estimate = nodes[static_cast<size_t>(open[estimate_index])];
            size_t focal_index = open.size();
            const double focal_estimate_bound = weight * best_estimate.estimated_cost;
            for (size_t index = 0; index < open.size(); ++index) {
                const ConstraintTreeNode& node = nodes[static_cast<size_t>(open[index])];
                if (!node.active || node.estimated_cost > focal_estimate_bound) {
                    continue;
                }
                if (focal_index == open.size() ||
                    std::tie(node.conflicts, node.cost, node.id) <
                    std::tie(
                        nodes[static_cast<size_t>(open[focal_index])].conflicts,
                        nodes[static_cast<size_t>(open[focal_index])].cost,
                        nodes[static_cast<size_t>(open[focal_index])].id
                    )) {
                    focal_index = index;
                }
            }
            const double bound = weight * static_cast<double>(best_lower.lower_bound);
            size_t selected_index = cleanup_index;
            if (focal_index < open.size() &&
                static_cast<double>(nodes[static_cast<size_t>(open[focal_index])].cost) <= bound) {
                selected_index = focal_index;
            } else if (static_cast<double>(best_estimate.cost) <= bound) {
                selected_index = estimate_index;
            }
            const uint64_t selected_id = open[selected_index];
            nodes[static_cast<size_t>(selected_id)].active = false;
            const ConstraintTreeNode selected_node = nodes[static_cast<size_t>(selected_id)];
            budget.expanded(true);
            if (selected_node.conflicts == 0) {
                incumbent = selected_node.paths;
                incumbent_cost = selected_node.cost;
                result->stop_reason = PP_SEARCH_STOP_SUCCESS;
                exhausted = true;
                break;
            }

            Conflict conflict{};
            uint64_t conflict_total = 0;
            find_conflict(selected_node.paths, &conflict, &conflict_total);
            std::vector<uint64_t> children;
            for (size_t side = 0; side < 2; ++side) {
                const size_t agent = side == 0 ? conflict.first_agent : conflict.second_agent;
                auto child_constraints = selected_node.constraints;
                bool inserted = false;
                if (conflict.edge_swap) {
                    const uint64_t from = side == 0 ? conflict.first_node : conflict.second_node;
                    const uint64_t to = side == 0 ? conflict.second_node : conflict.first_node;
                    inserted = child_constraints[agent].edges.insert(
                        {from, to, conflict.time}
                    ).second;
                } else {
                    inserted = child_constraints[agent].vertices.insert(
                        {conflict.first_node, conflict.time}
                    ).second;
                }
                if (!inserted || !explored.insert(constraints_key(child_constraints)).second) {
                    continue;
                }
                auto child_paths = selected_node.paths;
                auto child_lower_bounds = selected_node.path_lower_bounds;
                LowLevelResult low = low_level_focal_search(
                    agent, starts[agent], goals[agent], child_constraints[agent], adjacency,
                    heuristics[agent], child_paths, weight, &budget, trace
                );
                if (low.interrupted) {
                    budget.expansion_limited = budget.expansion_limited ||
                        (!budget.time_limited && has_max_expansions != 0);
                    break;
                }
                if (!low.found) {
                    continue;
                }
                child_paths[agent] = std::move(low.path);
                child_lower_bounds[agent] = low.f_min;
                ConstraintTreeNode child{
                    static_cast<uint64_t>(nodes.size()), selected_id,
                    std::move(child_constraints), std::move(child_paths),
                    std::move(child_lower_bounds), 0, 0, 0, 0.0, true
                };
                child.cost = sum_path_costs(child.paths);
                child.lower_bound = lower_bound_sum(child.path_lower_bounds);
                find_conflict(child.paths, nullptr, &child.conflicts);
                const double estimated_remaining = std::max(
                    0.0,
                    static_cast<double>(child.conflicts) * average_cost_error /
                        std::max(0.1, 1.0 - average_distance_error)
                );
                child.estimated_cost = static_cast<double>(child.cost) + estimated_remaining;
                if (child.conflicts == 0 && child.cost < incumbent_cost) {
                    incumbent = child.paths;
                    incumbent_cost = child.cost;
                }
                children.push_back(child.id);
                nodes.push_back(std::move(child));
                open.push_back(nodes.back().id);
                budget.discovered();
            }
            if (!children.empty()) {
                uint64_t best_child_id = children[0];
                for (uint64_t child_id : children) {
                    const ConstraintTreeNode& child = nodes[static_cast<size_t>(child_id)];
                    const ConstraintTreeNode& best = nodes[static_cast<size_t>(best_child_id)];
                    if (std::tie(child.estimated_cost, child.conflicts) <
                        std::tie(best.estimated_cost, best.conflicts)) {
                        best_child_id = child_id;
                    }
                }
                const ConstraintTreeNode& best_child_node = nodes[static_cast<size_t>(best_child_id)];
                const double distance_error = static_cast<double>(best_child_node.conflicts) -
                    (static_cast<double>(selected_node.conflicts) - 1.0);
                const double cost_error = static_cast<double>(best_child_node.cost) -
                    static_cast<double>(selected_node.cost);
                ++error_samples;
                average_distance_error += (distance_error - average_distance_error) /
                    static_cast<double>(error_samples);
                average_cost_error += (cost_error - average_cost_error) /
                    static_cast<double>(error_samples);
            }
        }

        if (open.empty() && !budget.time_limited && !budget.expansion_limited) {
            exhausted = true;
        }
        if (!incumbent.empty()) {
            copy_solution(incumbent, result);
        }
        result->iters = budget.expansions;
        result->nodes = budget.nodes;
        result->high_level_expanded = budget.high_level_expanded;
        result->low_level_expanded = budget.low_level_expanded;
        if (result->success && !exhausted) {
            result->stop_reason = budget.time_limited ? PP_SEARCH_STOP_TIME_BUDGET :
                PP_SEARCH_STOP_MAX_ITERS;
        } else if (!result->success && (budget.time_limited || budget.expansion_limited)) {
            result->stop_reason = budget.time_limited ? PP_SEARCH_STOP_TIME_BUDGET :
                PP_SEARCH_STOP_MAX_ITERS;
        } else if (!result->success) {
            result->stop_reason = PP_SEARCH_STOP_NO_PROGRESS;
        }
        return 0;
    } catch (const std::exception& error) {
        std::free(result->path_offsets);
        std::free(result->path_nodes);
        result->path_offsets = nullptr;
        result->path_nodes = nullptr;
        result->success = 0;
        set_result_error(result, error.what());
        return 1;
    } catch (...) {
        std::free(result->path_offsets);
        std::free(result->path_nodes);
        result->path_offsets = nullptr;
        result->path_nodes = nullptr;
        result->success = 0;
        set_result_error(result, "unknown EECBS search error");
        return 1;
    }
}

}  // namespace

extern "C" int pp_eecbs_plan(
    uint64_t node_count,
    uint64_t edge_count,
    const uint64_t* offsets,
    const uint64_t* neighbor_ids,
    const uint64_t* starts,
    const uint64_t* goals,
    size_t agent_count,
    double suboptimality_weight,
    int has_max_expansions,
    uint64_t max_expansions,
    double max_runtime_ms,
    pp_mapf_result* result
) {
    return run_eecbs(
        node_count, edge_count, offsets, neighbor_ids, starts, goals, agent_count,
        suboptimality_weight, has_max_expansions, max_expansions, max_runtime_ms,
        result, nullptr
    );
}

extern "C" void pp_mapf_free_result(pp_mapf_result* result) {
    if (result == nullptr) {
        return;
    }
    std::free(result->path_offsets);
    std::free(result->path_nodes);
    std::free(result->error_message);
    *result = pp_mapf_result{};
}

#if defined(PP_ENABLE_TRACE) && PP_ENABLE_TRACE
extern "C" int pp_eecbs_plan_traced(
    uint64_t node_count,
    uint64_t edge_count,
    const uint64_t* offsets,
    const uint64_t* neighbor_ids,
    const uint64_t* starts,
    const uint64_t* goals,
    size_t agent_count,
    double suboptimality_weight,
    int has_max_expansions,
    uint64_t max_expansions,
    double max_runtime_ms,
    uint64_t trace_max_bytes,
    pp_mapf_result* result,
    pp_trace_result* trace
) {
    if (trace == nullptr) {
        set_result_error(result, "trace result must not be null");
        return 1;
    }
    TraceBuffer capture(trace_max_bytes);
    const int status = run_eecbs(
        node_count, edge_count, offsets, neighbor_ids, starts, goals, agent_count,
        suboptimality_weight, has_max_expansions, max_expansions, max_runtime_ms,
        result, &capture
    );
    if (capture.copy(trace) != 0) {
        pp_mapf_free_result(result);
        set_result_error(result, "could not allocate EECBS trace events");
        return 1;
    }
    return status;
}
#endif
