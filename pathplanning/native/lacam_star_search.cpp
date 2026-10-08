#include "search_engine.h"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdlib>
#include <cstring>
#include <deque>
#include <functional>
#include <limits>
#include <map>
#include <new>
#include <queue>
#include <random>
#include <set>
#include <stdexcept>
#include <string>
#include <tuple>
#include <utility>
#include <vector>

namespace {

constexpr uint64_t kInfinity = std::numeric_limits<uint64_t>::max();
constexpr uint64_t kNoNode = std::numeric_limits<uint64_t>::max();
using Clock = std::chrono::steady_clock;

struct TraceBuffer {
    explicit TraceBuffer(uint64_t max_bytes) : max_events(max_bytes / sizeof(pp_trace_event)) {}

    void record(uint64_t node, uint64_t parent, uint64_t value) {
        if (events.size() >= max_events) {
            truncated = true;
            return;
        }
        events.push_back({node, parent, static_cast<double>(value), PP_TRACE_EXPAND, 0});
    }

    int copy(pp_trace_result* result) const {
        if (result == nullptr) {
            return 1;
        }
        *result = pp_trace_result{};
        if (!events.empty()) {
            result->events = static_cast<pp_trace_event*>(
                std::malloc(events.size() * sizeof(pp_trace_event))
            );
            if (result->events == nullptr) {
                return 1;
            }
            std::memcpy(result->events, events.data(), events.size() * sizeof(pp_trace_event));
        }
        result->event_count = events.size();
        result->truncated = truncated ? 1 : 0;
        return 0;
    }

    size_t max_events;
    bool truncated = false;
    std::vector<pp_trace_event> events;
};

struct Budget {
    Budget(int enabled, uint64_t max_expansions, double max_runtime_ms)
        : limited(enabled != 0), max_expansions(max_expansions),
          max_runtime_ms(max_runtime_ms), started(Clock::now()) {}

    bool can_expand() {
        if (limited && expansions >= max_expansions) {
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

    bool limited;
    uint64_t max_expansions;
    double max_runtime_ms;
    Clock::time_point started;
    uint64_t expansions = 0;
    uint64_t discovered = 0;
    uint64_t high_level_expanded = 0;
    uint64_t low_level_expanded = 0;
    bool expansion_limited = false;
    bool time_limited = false;
};

struct ConstraintNode {
    uint64_t parent;
    uint64_t agent;
    uint64_t target;
    uint64_t depth;
};

struct ConfigurationNode {
    std::vector<uint64_t> positions;
    uint64_t parent = kNoNode;
    uint64_t g = kInfinity;
    uint64_t h = kInfinity;
    std::vector<uint64_t> successors;
    std::vector<ConstraintNode> constraints;
    std::deque<uint64_t> pending_constraints;
    bool queued = false;
};

uint64_t add_cost(uint64_t left, uint64_t right) {
    if (left > kInfinity - right) {
        return kInfinity;
    }
    return left + right;
}

void set_error(pp_mapf_result* result, const std::string& message) {
    if (result == nullptr) {
        return;
    }
    std::free(result->error_message);
    result->error_message = static_cast<char*>(std::malloc(message.size() + 1));
    if (result->error_message != nullptr) {
        std::memcpy(result->error_message, message.c_str(), message.size() + 1);
    }
}

void validate_inputs(
    uint64_t node_count,
    uint64_t edge_count,
    const uint64_t* offsets,
    const uint64_t* neighbors,
    const uint64_t* starts,
    const uint64_t* goals,
    size_t agent_count,
    int has_max_expansions,
    uint64_t max_expansions,
    double max_runtime_ms
) {
    if (node_count == 0 || offsets == nullptr || starts == nullptr || goals == nullptr ||
        agent_count == 0 || offsets[0] != 0 || offsets[node_count] != edge_count ||
        (edge_count > 0 && neighbors == nullptr)) {
        throw std::invalid_argument("LaCAM* graph and agent arrays are invalid");
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
            throw std::invalid_argument("LaCAM* CSR offsets must be monotone");
        }
    }
    for (uint64_t edge = 0; edge < edge_count; ++edge) {
        if (neighbors[edge] >= node_count) {
            throw std::invalid_argument("LaCAM* edge endpoint is outside the graph");
        }
    }
    for (size_t agent = 0; agent < agent_count; ++agent) {
        if (starts[agent] >= node_count || goals[agent] >= node_count) {
            throw std::invalid_argument("LaCAM* start or goal is outside the graph");
        }
    }
    const std::set<uint64_t> unique_starts(starts, starts + agent_count);
    const std::set<uint64_t> unique_goals(goals, goals + agent_count);
    if (unique_starts.size() != agent_count || unique_goals.size() != agent_count) {
        throw std::invalid_argument("LaCAM* starts and goals must be unique");
    }
}

std::vector<std::vector<uint64_t>> make_heuristics(
    const std::vector<std::vector<uint64_t>>& adjacency,
    const std::vector<uint64_t>& goals
) {
    std::vector<std::vector<uint64_t>> result;
    result.reserve(goals.size());
    for (uint64_t goal : goals) {
        std::vector<uint64_t> distances(adjacency.size(), kInfinity);
        std::vector<uint64_t> queue{goal};
        distances[static_cast<size_t>(goal)] = 0;
        for (size_t cursor = 0; cursor < queue.size(); ++cursor) {
            const uint64_t node = queue[cursor];
            for (uint64_t neighbor : adjacency[static_cast<size_t>(node)]) {
                uint64_t& distance = distances[static_cast<size_t>(neighbor)];
                if (distance == kInfinity) {
                    distance = distances[static_cast<size_t>(node)] + 1;
                    queue.push_back(neighbor);
                }
            }
        }
        result.push_back(std::move(distances));
    }
    return result;
}

std::vector<uint64_t> reconstruct_prefix(
    const ConfigurationNode& node,
    uint64_t constraint_id
) {
    const ConstraintNode& leaf = node.constraints[static_cast<size_t>(constraint_id)];
    std::vector<uint64_t> assigned(static_cast<size_t>(leaf.depth));
    uint64_t cursor = constraint_id;
    while (cursor != 0) {
        const ConstraintNode& constraint = node.constraints[static_cast<size_t>(cursor)];
        assigned[static_cast<size_t>(constraint.agent)] = constraint.target;
        cursor = constraint.parent;
    }
    return assigned;
}

bool valid_constraint(
    const ConfigurationNode& node,
    uint64_t agent,
    uint64_t target,
    const std::vector<uint64_t>& assigned
) {
    for (size_t other = 0; other < assigned.size(); ++other) {
        if (assigned[other] == target) {
            return false;
        }
        if (node.positions[static_cast<size_t>(agent)] != target &&
            node.positions[other] != assigned[other] &&
            node.positions[static_cast<size_t>(agent)] == assigned[other] &&
            node.positions[other] == target) {
            return false;
        }
    }
    return true;
}

std::vector<uint64_t> next_positions(
    const ConfigurationNode& node,
    size_t agent,
    const std::vector<uint64_t>& goals,
    const std::vector<std::vector<uint64_t>>& adjacency,
    const std::vector<std::vector<uint64_t>>& heuristics
) {
    const uint64_t current = node.positions[agent];
    if (current == goals[agent]) {
        return {current};
    }
    std::vector<uint64_t> candidates = adjacency[static_cast<size_t>(current)];
    candidates.push_back(current);
    std::sort(candidates.begin(), candidates.end(), [&](uint64_t left, uint64_t right) {
        return std::tie(heuristics[agent][static_cast<size_t>(left)], left) <
            std::tie(heuristics[agent][static_cast<size_t>(right)], right);
    });
    return candidates;
}

uint64_t transition_cost(const ConfigurationNode& node, const std::vector<uint64_t>& goals) {
    uint64_t cost = 0;
    for (size_t agent = 0; agent < goals.size(); ++agent) {
        if (node.positions[agent] != goals[agent]) {
            ++cost;
        }
    }
    return cost;
}

std::vector<std::vector<uint64_t>> decode_solution(
    const std::vector<ConfigurationNode>& nodes,
    uint64_t goal_id,
    const std::vector<uint64_t>& goals
) {
    std::vector<uint64_t> configurations;
    uint64_t cursor = goal_id;
    while (cursor != kNoNode) {
        configurations.push_back(cursor);
        cursor = nodes[static_cast<size_t>(cursor)].parent;
    }
    std::reverse(configurations.begin(), configurations.end());
    std::vector<std::vector<uint64_t>> paths(goals.size());
    for (uint64_t id : configurations) {
        const auto& positions = nodes[static_cast<size_t>(id)].positions;
        for (size_t agent = 0; agent < goals.size(); ++agent) {
            paths[agent].push_back(positions[agent]);
        }
    }
    for (size_t agent = 0; agent < goals.size(); ++agent) {
        while (paths[agent].size() > 1 && paths[agent].back() == goals[agent] &&
               paths[agent][paths[agent].size() - 2] == goals[agent]) {
            paths[agent].pop_back();
        }
    }
    return paths;
}

void copy_solution(const std::vector<std::vector<uint64_t>>& paths, pp_mapf_result* result) {
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
    result->sum_of_costs = 0;
    result->makespan = 0;
    for (const auto& path : paths) {
        const uint64_t cost = path.empty() ? 0 : static_cast<uint64_t>(path.size() - 1);
        result->sum_of_costs = add_cost(result->sum_of_costs, cost);
        result->makespan = std::max(result->makespan, cost);
    }
    result->success = 1;
}

int run_lacam_star(
    uint64_t node_count,
    uint64_t edge_count,
    const uint64_t* offsets,
    const uint64_t* neighbors,
    const uint64_t* starts,
    const uint64_t* goals,
    size_t agent_count,
    uint64_t seed,
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
            node_count, edge_count, offsets, neighbors, starts, goals, agent_count,
            has_max_expansions, max_expansions, max_runtime_ms
        );
        Budget budget(has_max_expansions, max_expansions, max_runtime_ms);
        std::vector<std::vector<uint64_t>> adjacency(static_cast<size_t>(node_count));
        for (uint64_t node = 0; node < node_count; ++node) {
            auto& row = adjacency[static_cast<size_t>(node)];
            for (uint64_t edge = offsets[node]; edge < offsets[node + 1]; ++edge) {
                const uint64_t neighbor = neighbors[edge];
                if (neighbor == node) {
                    throw std::invalid_argument("MAPF graph must not contain self-loops");
                }
                row.push_back(neighbor);
            }
            std::sort(row.begin(), row.end());
            if (std::adjacent_find(row.begin(), row.end()) != row.end()) {
                throw std::invalid_argument("MAPF graph must not contain duplicate edges");
            }
        }
        for (uint64_t node = 0; node < node_count; ++node) {
            for (uint64_t neighbor : adjacency[static_cast<size_t>(node)]) {
                const auto& reverse = adjacency[static_cast<size_t>(neighbor)];
                if (!std::binary_search(reverse.begin(), reverse.end(), node)) {
                    throw std::invalid_argument("MAPF graph must be undirected");
                }
            }
        }

        const std::vector<uint64_t> goal_vector(goals, goals + agent_count);
        const auto heuristics = make_heuristics(adjacency, goal_vector);
        ConfigurationNode root;
        root.positions.assign(starts, starts + agent_count);
        root.g = 0;
        root.h = 0;
        root.constraints.push_back({kNoNode, 0, 0, 0});
        root.pending_constraints.push_back(0);
        for (size_t agent = 0; agent < agent_count; ++agent) {
            const uint64_t h = heuristics[agent][static_cast<size_t>(starts[agent])];
            root.h = add_cost(root.h, h);
        }
        if (root.h == kInfinity) {
            result->stop_reason = PP_SEARCH_STOP_NO_PROGRESS;
            return 0;
        }

        std::vector<ConfigurationNode> nodes;
        nodes.push_back(std::move(root));
        std::map<std::vector<uint64_t>, uint64_t> node_ids;
        node_ids.emplace(nodes[0].positions, 0);
        std::vector<uint64_t> open{0};
        nodes[0].queued = true;
        budget.discovered = 1;
        uint64_t goal_id = nodes[0].positions == goal_vector ? 0 : kNoNode;
        uint64_t incumbent_cost = goal_id == kNoNode ? kInfinity : 0;
        std::mt19937_64 random(seed);
        bool exhausted = false;

        auto enqueue = [&](uint64_t id) {
            ConfigurationNode& node = nodes[static_cast<size_t>(id)];
            if (!node.queued && !node.pending_constraints.empty()) {
                node.queued = true;
                open.push_back(id);
            }
        };
        auto update_goal = [&]() {
            if (goal_id != kNoNode) {
                const uint64_t cost = nodes[static_cast<size_t>(goal_id)].g;
                incumbent_cost = std::min(incumbent_cost, cost);
            }
        };
        auto dijkstra_update = [&](uint64_t source, uint64_t target) {
            using Entry = std::pair<uint64_t, uint64_t>;
            std::priority_queue<Entry, std::vector<Entry>, std::greater<Entry>> queue;
            const uint64_t candidate = add_cost(
                nodes[static_cast<size_t>(source)].g,
                transition_cost(nodes[static_cast<size_t>(source)], goal_vector)
            );
            if (candidate < nodes[static_cast<size_t>(target)].g) {
                nodes[static_cast<size_t>(target)].g = candidate;
                nodes[static_cast<size_t>(target)].parent = source;
                queue.emplace(candidate, target);
            }
            while (!queue.empty()) {
                const auto [cost, current] = queue.top();
                queue.pop();
                if (cost != nodes[static_cast<size_t>(current)].g) {
                    continue;
                }
                for (uint64_t next : nodes[static_cast<size_t>(current)].successors) {
                    const uint64_t next_cost = add_cost(
                        cost, transition_cost(nodes[static_cast<size_t>(current)], goal_vector)
                    );
                    ConfigurationNode& next_node = nodes[static_cast<size_t>(next)];
                    if (next_cost < next_node.g) {
                        next_node.g = next_cost;
                        next_node.parent = current;
                        queue.emplace(next_cost, next);
                        enqueue(next);
                    }
                }
            }
            update_goal();
        };

        while (!open.empty()) {
            if (!budget.can_expand()) {
                break;
            }
            size_t selected = open.size() - 1;
            if (goal_id != kNoNode) {
                selected = static_cast<size_t>(random() % open.size());
            }
            const uint64_t current_id = open[selected];
            open[selected] = open.back();
            open.pop_back();
            ConfigurationNode& current = nodes[static_cast<size_t>(current_id)];
            current.queued = false;
            budget.expanded(true);
            if (trace != nullptr) {
                const uint64_t parent_position = current.parent == kNoNode ? kNoNode :
                    nodes[static_cast<size_t>(current.parent)].positions[0];
                trace->record(
                    current.positions[0], parent_position,
                    add_cost(current.g, current.h)
                );
            }

            const uint64_t estimate = add_cost(current.g, current.h);
            if (estimate >= incumbent_cost || current.pending_constraints.empty()) {
                continue;
            }
            if (!budget.can_expand()) {
                enqueue(current_id);
                break;
            }
            budget.expanded(false);
            const uint64_t constraint_id = current.pending_constraints.front();
            current.pending_constraints.pop_front();
            const ConstraintNode constraint = current.constraints[static_cast<size_t>(constraint_id)];
            std::vector<uint64_t> successor;
            if (constraint.depth == agent_count) {
                successor = reconstruct_prefix(current, constraint_id);
            } else {
                const size_t agent = static_cast<size_t>(constraint.depth);
                const auto assigned = reconstruct_prefix(current, constraint_id);
                const auto candidates = next_positions(
                    current, agent, goal_vector, adjacency, heuristics
                );
                for (uint64_t target : candidates) {
                    if (!valid_constraint(current, agent, target, assigned)) {
                        continue;
                    }
                    const uint64_t child_id = static_cast<uint64_t>(current.constraints.size());
                    current.constraints.push_back({
                        constraint_id, static_cast<uint64_t>(agent), target, constraint.depth + 1
                    });
                    current.pending_constraints.push_back(child_id);
                }
            }
            if (!current.pending_constraints.empty()) {
                enqueue(current_id);
            }
            if (successor.empty()) {
                continue;
            }

            auto found = node_ids.find(successor);
            uint64_t next_id;
            if (found == node_ids.end()) {
                ConfigurationNode next;
                next.positions = successor;
                next.g = add_cost(current.g, transition_cost(current, goal_vector));
                next.h = 0;
                next.parent = current_id;
                next.constraints.push_back({kNoNode, 0, 0, 0});
                next.pending_constraints.push_back(0);
                for (size_t agent = 0; agent < agent_count; ++agent) {
                    next.h = add_cost(
                        next.h,
                        heuristics[agent][static_cast<size_t>(successor[agent])]
                    );
                }
                next_id = static_cast<uint64_t>(nodes.size());
                nodes.push_back(std::move(next));
                node_ids.emplace(successor, next_id);
                nodes[static_cast<size_t>(current_id)].successors.push_back(next_id);
                enqueue(next_id);
                ++budget.discovered;
                if (successor == goal_vector) {
                    goal_id = next_id;
                    update_goal();
                }
            } else {
                next_id = found->second;
                auto& successors = nodes[static_cast<size_t>(current_id)].successors;
                if (std::find(successors.begin(), successors.end(), next_id) == successors.end()) {
                    successors.push_back(next_id);
                    dijkstra_update(current_id, next_id);
                }
                enqueue(next_id);
            }
        }

        exhausted = open.empty() && !budget.time_limited && !budget.expansion_limited;
        if (goal_id != kNoNode && incumbent_cost != kInfinity) {
            const auto paths = decode_solution(nodes, goal_id, goal_vector);
            copy_solution(paths, result);
        }
        result->iters = budget.expansions;
        result->nodes = budget.discovered;
        result->high_level_expanded = budget.high_level_expanded;
        result->low_level_expanded = budget.low_level_expanded;
        if (result->success && !exhausted) {
            result->stop_reason = budget.time_limited ? PP_SEARCH_STOP_TIME_BUDGET :
                PP_SEARCH_STOP_MAX_ITERS;
        } else if (result->success) {
            result->stop_reason = PP_SEARCH_STOP_SUCCESS;
        } else if (budget.time_limited || budget.expansion_limited) {
            result->stop_reason = budget.time_limited ? PP_SEARCH_STOP_TIME_BUDGET :
                PP_SEARCH_STOP_MAX_ITERS;
        } else {
            result->stop_reason = PP_SEARCH_STOP_NO_PROGRESS;
        }
        return 0;
    } catch (const std::exception& error) {
        std::free(result->path_offsets);
        std::free(result->path_nodes);
        result->path_offsets = nullptr;
        result->path_nodes = nullptr;
        result->success = 0;
        set_error(result, error.what());
        return 1;
    } catch (...) {
        std::free(result->path_offsets);
        std::free(result->path_nodes);
        result->path_offsets = nullptr;
        result->path_nodes = nullptr;
        result->success = 0;
        set_error(result, "unknown LaCAM* search error");
        return 1;
    }
}

}  // namespace

extern "C" int pp_lacam_star_plan(
    uint64_t node_count,
    uint64_t edge_count,
    const uint64_t* offsets,
    const uint64_t* neighbors,
    const uint64_t* starts,
    const uint64_t* goals,
    size_t agent_count,
    uint64_t seed,
    int has_max_expansions,
    uint64_t max_expansions,
    double max_runtime_ms,
    pp_mapf_result* result
) {
    return run_lacam_star(
        node_count, edge_count, offsets, neighbors, starts, goals, agent_count, seed,
        has_max_expansions, max_expansions, max_runtime_ms, result, nullptr
    );
}

#if defined(PP_ENABLE_TRACE) && PP_ENABLE_TRACE
extern "C" int pp_lacam_star_plan_traced(
    uint64_t node_count,
    uint64_t edge_count,
    const uint64_t* offsets,
    const uint64_t* neighbors,
    const uint64_t* starts,
    const uint64_t* goals,
    size_t agent_count,
    uint64_t seed,
    int has_max_expansions,
    uint64_t max_expansions,
    double max_runtime_ms,
    uint64_t trace_max_bytes,
    pp_mapf_result* result,
    pp_trace_result* trace
) {
    if (trace == nullptr) {
        set_error(result, "trace result must not be null");
        return 1;
    }
    TraceBuffer capture(trace_max_bytes);
    const int status = run_lacam_star(
        node_count, edge_count, offsets, neighbors, starts, goals, agent_count, seed,
        has_max_expansions, max_expansions, max_runtime_ms, result, &capture
    );
    if (capture.copy(trace) != 0) {
        pp_mapf_free_result(result);
        set_error(result, "could not allocate LaCAM* trace events");
        return 1;
    }
    return status;
}
#endif
