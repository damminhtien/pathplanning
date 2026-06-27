#include "search_engine.h"

#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <cstring>
#include <deque>
#include <limits>
#include <new>
#include <queue>
#include <stdexcept>
#include <string>
#include <unordered_map>
#include <vector>

namespace {

constexpr const char* kVersion = "0.2.0";
constexpr double kInfinity = std::numeric_limits<double>::infinity();

struct NodeState {
    double g_cost = kInfinity;
    uint64_t parent = 0;
    bool has_parent = false;
    bool closed = false;
};

struct QueueEntry {
    double f_score = 0.0;
    double h_score = 0.0;
    uint64_t order = 0;
    uint64_t node_id = 0;
};

struct QueueEntryGreater {
    bool operator()(const QueueEntry& lhs, const QueueEntry& rhs) const {
        if (lhs.f_score != rhs.f_score) {
            return lhs.f_score > rhs.f_score;
        }
        if (lhs.h_score != rhs.h_score) {
            return lhs.h_score > rhs.h_score;
        }
        return lhs.order > rhs.order;
    }
};

struct NeighborBatch {
    const uint64_t* ids = nullptr;
    const double* costs = nullptr;
    size_t count = 0;
};

struct BidirectionalExpansion {
    bool expanded = false;
    double best_cost = kInfinity;
    uint64_t meet_id = 0;
    bool has_meet = false;
};

void reset_result(pp_search_result* result) {
    if (result == nullptr) {
        return;
    }
    result->success = 0;
    result->stop_reason = PP_SEARCH_STOP_ERROR;
    result->iters = 0;
    result->nodes = 0;
    result->path_cost = kInfinity;
    result->path_ids = nullptr;
    result->path_length = 0;
    result->error_message = nullptr;
}

char* copy_message(const std::string& message) {
    char* buffer = static_cast<char*>(std::malloc(message.size() + 1));
    if (buffer == nullptr) {
        return nullptr;
    }
    std::memcpy(buffer, message.c_str(), message.size() + 1);
    return buffer;
}

void set_error(pp_search_result* result, const std::string& message) {
    reset_result(result);
    if (result != nullptr) {
        result->error_message = copy_message(message);
    }
}

void validate_graph_and_result(
    const pp_graph_callbacks* graph,
    const pp_search_result* result
) {
    if (graph == nullptr) {
        throw std::invalid_argument("graph callbacks must not be null");
    }
    if (graph->is_goal == nullptr) {
        throw std::invalid_argument("graph.is_goal callback must not be null");
    }
    if (graph->heuristic == nullptr) {
        throw std::invalid_argument("graph.heuristic callback must not be null");
    }
    if (graph->neighbors == nullptr) {
        throw std::invalid_argument("graph.neighbors callback must not be null");
    }
    if (result == nullptr) {
        throw std::invalid_argument("result must not be null");
    }
}

void validate_search_options(const pp_search_options* options) {
    if (options == nullptr) {
        throw std::invalid_argument("search options must not be null");
    }
    switch (options->algorithm) {
        case PP_SEARCH_BFS:
        case PP_SEARCH_DFS:
        case PP_SEARCH_GREEDY_BEST_FIRST:
        case PP_SEARCH_ASTAR:
        case PP_SEARCH_DIJKSTRA:
        case PP_SEARCH_WEIGHTED_ASTAR:
        case PP_SEARCH_BIDIRECTIONAL_ASTAR:
        case PP_SEARCH_ANYTIME_ASTAR:
            break;
        default:
            throw std::invalid_argument("unknown search algorithm");
    }
    if (!std::isfinite(options->heuristic_weight) || options->heuristic_weight < 0.0) {
        throw std::invalid_argument("heuristic_weight must be a finite non-negative value");
    }
    if (options->has_max_expansions && options->max_expansions == 0) {
        throw std::invalid_argument("max_expansions must be > 0 when enabled");
    }
    if (options->algorithm == PP_SEARCH_BIDIRECTIONAL_ASTAR && !options->has_goal_id) {
        throw std::invalid_argument("bidirectional search requires an exact goal id");
    }
    if (options->algorithm == PP_SEARCH_ANYTIME_ASTAR) {
        if (options->anytime_weights == nullptr || options->anytime_weight_count == 0) {
            throw std::invalid_argument("anytime search requires at least one weight");
        }
        for (size_t index = 0; index < options->anytime_weight_count; ++index) {
            const double weight = options->anytime_weights[index];
            if (!std::isfinite(weight) || weight < 1.0) {
                throw std::invalid_argument("anytime weights must be finite values >= 1.0");
            }
        }
    }
}

void validate_inputs(
    const pp_graph_callbacks* graph,
    const pp_search_options* options,
    const pp_search_result* result
) {
    validate_graph_and_result(graph, result);
    validate_search_options(options);
}

double compute_heuristic(const pp_graph_callbacks* graph, uint64_t node_id, double weight) {
    double value = 0.0;
    if (graph->heuristic(graph->user_data, node_id, &value) != 0) {
        throw std::runtime_error("heuristic callback failed");
    }
    if (!std::isfinite(value)) {
        value = 0.0;
    }
    return weight * value;
}

bool is_goal(const pp_graph_callbacks* graph, uint64_t node_id) {
    int value = 0;
    if (graph->is_goal(graph->user_data, node_id, &value) != 0) {
        throw std::runtime_error("is_goal callback failed");
    }
    return value != 0;
}

NeighborBatch load_neighbors(const pp_graph_callbacks* graph, uint64_t node_id) {
    NeighborBatch batch;
    if (graph->neighbors(
            graph->user_data,
            node_id,
            &batch.ids,
            &batch.costs,
            &batch.count
        ) != 0) {
        throw std::runtime_error("neighbors callback failed");
    }
    if (batch.count > 0 && (batch.ids == nullptr || batch.costs == nullptr)) {
        throw std::runtime_error("neighbors callback returned null edge arrays");
    }
    return batch;
}

bool over_expansion_budget(const pp_search_options* options, uint64_t expanded) {
    return options->has_max_expansions && expanded >= options->max_expansions;
}

void reserve_states(
    std::unordered_map<uint64_t, NodeState>* states,
    const pp_search_options* options
) {
    if (options->reserve_nodes > 0) {
        states->reserve(static_cast<size_t>(options->reserve_nodes));
    }
}

std::vector<uint64_t> reconstruct_path(
    const std::unordered_map<uint64_t, NodeState>& states,
    uint64_t start_id,
    uint64_t reached_id
) {
    std::vector<uint64_t> reversed;
    uint64_t current = reached_id;
    reversed.push_back(current);

    while (current != start_id) {
        auto found = states.find(current);
        if (found == states.end() || !found->second.has_parent) {
            throw std::runtime_error("missing parent while reconstructing path");
        }
        current = found->second.parent;
        reversed.push_back(current);
    }

    std::reverse(reversed.begin(), reversed.end());
    return reversed;
}

std::vector<uint64_t> reconstruct_bidirectional_path(
    const std::unordered_map<uint64_t, NodeState>& forward_states,
    const std::unordered_map<uint64_t, NodeState>& backward_states,
    uint64_t start_id,
    uint64_t goal_id,
    uint64_t meet_id
) {
    std::vector<uint64_t> path = reconstruct_path(forward_states, start_id, meet_id);

    uint64_t current = meet_id;
    while (current != goal_id) {
        auto found = backward_states.find(current);
        if (found == backward_states.end() || !found->second.has_parent) {
            throw std::runtime_error("missing backward parent while reconstructing path");
        }
        current = found->second.parent;
        path.push_back(current);
    }
    return path;
}

void store_path(pp_search_result* result, const std::vector<uint64_t>& path) {
    if (path.empty()) {
        result->path_ids = nullptr;
        result->path_length = 0;
        return;
    }

    const size_t bytes = path.size() * sizeof(uint64_t);
    uint64_t* ids = static_cast<uint64_t*>(std::malloc(bytes));
    if (ids == nullptr) {
        throw std::bad_alloc();
    }
    std::memcpy(ids, path.data(), bytes);
    result->path_ids = ids;
    result->path_length = path.size();
}

void set_failure_result(
    pp_search_result* result,
    pp_search_stop_reason reason,
    uint64_t expanded,
    uint64_t nodes
) {
    reset_result(result);
    result->success = 0;
    result->stop_reason = reason;
    result->iters = expanded;
    result->nodes = nodes;
    result->path_cost = kInfinity;
}

void set_success_result(
    pp_search_result* result,
    uint64_t expanded,
    uint64_t nodes,
    double path_cost,
    const std::vector<uint64_t>& path
) {
    result->success = 1;
    result->stop_reason = PP_SEARCH_STOP_SUCCESS;
    result->iters = expanded;
    result->nodes = nodes;
    result->path_cost = path_cost;
    store_path(result, path);
}

int run_linear_search(
    const pp_graph_callbacks* graph,
    uint64_t start_id,
    const pp_search_options* options,
    pp_search_result* result
) {
    std::unordered_map<uint64_t, NodeState> states;
    reserve_states(&states, options);

    states[start_id].g_cost = 0.0;

    std::deque<uint64_t> frontier;
    frontier.push_back(start_id);
    uint64_t expanded = 0;

    while (!frontier.empty()) {
        uint64_t node_id = frontier.front();
        if (options->algorithm == PP_SEARCH_DFS) {
            node_id = frontier.back();
            frontier.pop_back();
        } else {
            frontier.pop_front();
        }

        auto state_iter = states.find(node_id);
        if (state_iter == states.end()) {
            continue;
        }

        ++expanded;
        if (is_goal(graph, node_id)) {
            set_success_result(
                result,
                expanded,
                static_cast<uint64_t>(states.size()),
                state_iter->second.g_cost,
                reconstruct_path(states, start_id, node_id)
            );
            return 0;
        }

        if (over_expansion_budget(options, expanded)) {
            set_failure_result(
                result,
                PP_SEARCH_STOP_MAX_ITERS,
                expanded,
                static_cast<uint64_t>(states.size())
            );
            return 0;
        }

        const double current_g = state_iter->second.g_cost;
        const NeighborBatch neighbors = load_neighbors(graph, node_id);

        auto add_neighbor = [&](size_t index) {
            const double edge_cost = neighbors.costs[index];
            if (!std::isfinite(edge_cost)) {
                return;
            }
            const uint64_t neighbor_id = neighbors.ids[index];
            if (states.find(neighbor_id) != states.end()) {
                return;
            }

            const double tentative = current_g + edge_cost;
            if (!std::isfinite(tentative)) {
                return;
            }

            NodeState& neighbor_state = states[neighbor_id];
            neighbor_state.g_cost = tentative;
            neighbor_state.parent = node_id;
            neighbor_state.has_parent = true;
            frontier.push_back(neighbor_id);
        };

        if (options->algorithm == PP_SEARCH_DFS) {
            for (size_t offset = 0; offset < neighbors.count; ++offset) {
                add_neighbor(neighbors.count - 1 - offset);
            }
        } else {
            for (size_t index = 0; index < neighbors.count; ++index) {
                add_neighbor(index);
            }
        }
    }

    set_failure_result(
        result,
        PP_SEARCH_STOP_NO_PROGRESS,
        expanded,
        static_cast<uint64_t>(states.size())
    );
    return 0;
}

int run_best_first_search(
    const pp_graph_callbacks* graph,
    uint64_t start_id,
    const pp_search_options* options,
    pp_search_result* result
) {
    std::unordered_map<uint64_t, NodeState> states;
    reserve_states(&states, options);

    std::priority_queue<
        QueueEntry,
        std::vector<QueueEntry>,
        QueueEntryGreater
    > open;

    states[start_id].g_cost = 0.0;

    const bool greedy = options->algorithm == PP_SEARCH_GREEDY_BEST_FIRST;
    const bool use_heuristic =
        options->algorithm == PP_SEARCH_ASTAR ||
        options->algorithm == PP_SEARCH_WEIGHTED_ASTAR ||
        options->algorithm == PP_SEARCH_GREEDY_BEST_FIRST;
    const double heuristic_weight = use_heuristic ? options->heuristic_weight : 0.0;

    uint64_t order = 0;
    const double start_h = compute_heuristic(graph, start_id, heuristic_weight);
    open.push(QueueEntry{start_h, start_h, order++, start_id});

    uint64_t expanded = 0;

    while (!open.empty()) {
        QueueEntry entry = open.top();
        open.pop();

        auto state_iter = states.find(entry.node_id);
        if (state_iter == states.end() || state_iter->second.closed) {
            continue;
        }

        NodeState& state = state_iter->second;
        state.closed = true;
        ++expanded;

        if (is_goal(graph, entry.node_id)) {
            set_success_result(
                result,
                expanded,
                static_cast<uint64_t>(states.size()),
                state.g_cost,
                reconstruct_path(states, start_id, entry.node_id)
            );
            return 0;
        }

        if (over_expansion_budget(options, expanded)) {
            set_failure_result(
                result,
                PP_SEARCH_STOP_MAX_ITERS,
                expanded,
                static_cast<uint64_t>(states.size())
            );
            return 0;
        }

        const double current_g = state.g_cost;
        const NeighborBatch neighbors = load_neighbors(graph, entry.node_id);
        for (size_t index = 0; index < neighbors.count; ++index) {
            const double edge_cost = neighbors.costs[index];
            if (!std::isfinite(edge_cost)) {
                continue;
            }

            const uint64_t neighbor_id = neighbors.ids[index];
            const double tentative = current_g + edge_cost;
            if (!std::isfinite(tentative)) {
                continue;
            }

            NodeState& neighbor_state = states[neighbor_id];
            if (neighbor_state.closed) {
                continue;
            }

            if (greedy) {
                if (neighbor_state.g_cost != kInfinity) {
                    continue;
                }
            } else if (tentative >= neighbor_state.g_cost) {
                continue;
            }

            neighbor_state.g_cost = tentative;
            neighbor_state.parent = entry.node_id;
            neighbor_state.has_parent = true;

            const double h_score = compute_heuristic(graph, neighbor_id, heuristic_weight);
            const double f_score = greedy ? h_score : tentative + h_score;
            open.push(QueueEntry{f_score, h_score, order++, neighbor_id});
        }
    }

    set_failure_result(
        result,
        PP_SEARCH_STOP_NO_PROGRESS,
        expanded,
        static_cast<uint64_t>(states.size())
    );
    return 0;
}

BidirectionalExpansion expand_bidirectional_frontier(
    const pp_graph_callbacks* graph,
    std::priority_queue<QueueEntry, std::vector<QueueEntry>, QueueEntryGreater>* heap,
    std::unordered_map<uint64_t, NodeState>* states_this,
    const std::unordered_map<uint64_t, NodeState>& states_other,
    uint64_t* tie_breaker,
    uint64_t* expanded
) {
    while (!heap->empty()) {
        QueueEntry entry = heap->top();
        heap->pop();

        auto state_iter = states_this->find(entry.node_id);
        if (state_iter == states_this->end() || state_iter->second.closed) {
            continue;
        }

        NodeState& state = state_iter->second;
        state.closed = true;
        ++(*expanded);

        BidirectionalExpansion outcome;
        outcome.expanded = true;
        auto other_iter = states_other.find(entry.node_id);
        if (other_iter != states_other.end() && std::isfinite(other_iter->second.g_cost)) {
            outcome.best_cost = state.g_cost + other_iter->second.g_cost;
            outcome.meet_id = entry.node_id;
            outcome.has_meet = true;
        }

        const double current_g = state.g_cost;
        const NeighborBatch neighbors = load_neighbors(graph, entry.node_id);
        for (size_t index = 0; index < neighbors.count; ++index) {
            const double edge_cost = neighbors.costs[index];
            if (!std::isfinite(edge_cost)) {
                continue;
            }
            const uint64_t neighbor_id = neighbors.ids[index];
            const double tentative = current_g + edge_cost;
            if (!std::isfinite(tentative)) {
                continue;
            }

            NodeState& neighbor_state = (*states_this)[neighbor_id];
            if (neighbor_state.closed || tentative >= neighbor_state.g_cost) {
                continue;
            }

            neighbor_state.g_cost = tentative;
            neighbor_state.parent = entry.node_id;
            neighbor_state.has_parent = true;
            heap->push(QueueEntry{tentative, 0.0, (*tie_breaker)++, neighbor_id});

            auto other_neighbor = states_other.find(neighbor_id);
            if (other_neighbor != states_other.end() && std::isfinite(other_neighbor->second.g_cost)) {
                const double candidate = tentative + other_neighbor->second.g_cost;
                if (candidate < outcome.best_cost) {
                    outcome.best_cost = candidate;
                    outcome.meet_id = neighbor_id;
                    outcome.has_meet = true;
                }
            }
        }

        return outcome;
    }

    return BidirectionalExpansion{};
}

int run_bidirectional_search(
    const pp_graph_callbacks* graph,
    uint64_t start_id,
    const pp_search_options* options,
    pp_search_result* result
) {
    const uint64_t goal_id = options->goal_id;
    if (start_id == goal_id) {
        set_success_result(result, 0, 1, 0.0, std::vector<uint64_t>{start_id});
        return 0;
    }

    std::unordered_map<uint64_t, NodeState> forward_states;
    std::unordered_map<uint64_t, NodeState> backward_states;
    reserve_states(&forward_states, options);
    reserve_states(&backward_states, options);

    forward_states[start_id].g_cost = 0.0;
    backward_states[goal_id].g_cost = 0.0;

    std::priority_queue<QueueEntry, std::vector<QueueEntry>, QueueEntryGreater> forward_heap;
    std::priority_queue<QueueEntry, std::vector<QueueEntry>, QueueEntryGreater> backward_heap;
    uint64_t tie_breaker = 0;
    forward_heap.push(QueueEntry{0.0, 0.0, tie_breaker++, start_id});
    backward_heap.push(QueueEntry{0.0, 0.0, tie_breaker++, goal_id});

    double best_cost = kInfinity;
    uint64_t meet_id = 0;
    bool has_meet = false;
    uint64_t expanded = 0;

    while (!forward_heap.empty() && !backward_heap.empty()) {
        if (over_expansion_budget(options, expanded)) {
            break;
        }

        BidirectionalExpansion local;
        if (forward_heap.top().f_score <= backward_heap.top().f_score) {
            local = expand_bidirectional_frontier(
                graph,
                &forward_heap,
                &forward_states,
                backward_states,
                &tie_breaker,
                &expanded
            );
        } else {
            local = expand_bidirectional_frontier(
                graph,
                &backward_heap,
                &backward_states,
                forward_states,
                &tie_breaker,
                &expanded
            );
        }

        if (!local.expanded) {
            break;
        }
        if (local.has_meet && local.best_cost < best_cost) {
            best_cost = local.best_cost;
            meet_id = local.meet_id;
            has_meet = true;
        }

        const double forward_bound = forward_heap.empty() ? kInfinity : forward_heap.top().f_score;
        const double backward_bound = backward_heap.empty() ? kInfinity : backward_heap.top().f_score;
        if (has_meet && (forward_bound + backward_bound) >= best_cost) {
            break;
        }
    }

    const uint64_t node_count =
        static_cast<uint64_t>(forward_states.size() + backward_states.size());
    if (!has_meet) {
        set_failure_result(
            result,
            options->has_max_expansions ? PP_SEARCH_STOP_MAX_ITERS : PP_SEARCH_STOP_NO_PROGRESS,
            expanded,
            node_count
        );
        return 0;
    }

    set_success_result(
        result,
        expanded,
        node_count,
        best_cost,
        reconstruct_bidirectional_path(
            forward_states,
            backward_states,
            start_id,
            goal_id,
            meet_id
        )
    );
    return 0;
}

int run_anytime_astar_search(
    const pp_graph_callbacks* graph,
    uint64_t start_id,
    const pp_search_options* options,
    pp_search_result* result
) {
    uint64_t total_iters = 0;
    uint64_t total_nodes = 0;
    double best_cost = kInfinity;
    std::vector<uint64_t> best_path;

    for (size_t index = 0; index < options->anytime_weight_count; ++index) {
        pp_search_options weighted_options = *options;
        weighted_options.algorithm = PP_SEARCH_WEIGHTED_ASTAR;
        weighted_options.heuristic_weight = options->anytime_weights[index];

        pp_search_result candidate;
        reset_result(&candidate);
        const int status = run_best_first_search(graph, start_id, &weighted_options, &candidate);
        if (status != 0) {
            pp_search_free_result(&candidate);
            return status;
        }

        total_iters += candidate.iters;
        total_nodes += candidate.nodes;

        if (candidate.success && candidate.path_ids != nullptr && candidate.path_cost < best_cost) {
            best_cost = candidate.path_cost;
            best_path.assign(candidate.path_ids, candidate.path_ids + candidate.path_length);
        }

        pp_search_free_result(&candidate);
        if (best_path.size() > 0 && options->anytime_weights[index] <= 1.0) {
            break;
        }
    }

    if (best_path.empty()) {
        set_failure_result(result, PP_SEARCH_STOP_NO_PROGRESS, total_iters, total_nodes);
        return 0;
    }

    set_success_result(result, total_iters, total_nodes, best_cost, best_path);
    return 0;
}

}  // namespace

extern "C" int pp_search_plan(
    const pp_graph_callbacks* graph,
    uint64_t start_id,
    const pp_search_options* options,
    pp_search_result* result
) {
    try {
        validate_inputs(graph, options, result);
        reset_result(result);

        switch (options->algorithm) {
            case PP_SEARCH_BFS:
            case PP_SEARCH_DFS:
                return run_linear_search(graph, start_id, options, result);
            case PP_SEARCH_GREEDY_BEST_FIRST:
            case PP_SEARCH_ASTAR:
            case PP_SEARCH_DIJKSTRA:
            case PP_SEARCH_WEIGHTED_ASTAR:
                return run_best_first_search(graph, start_id, options, result);
            case PP_SEARCH_BIDIRECTIONAL_ASTAR:
                return run_bidirectional_search(graph, start_id, options, result);
            case PP_SEARCH_ANYTIME_ASTAR:
                return run_anytime_astar_search(graph, start_id, options, result);
            default:
                throw std::invalid_argument("unknown search algorithm");
        }
    } catch (const std::exception& exc) {
        set_error(result, exc.what());
        return 1;
    } catch (...) {
        set_error(result, "unknown native search error");
        return 1;
    }
}

extern "C" int pp_astar_plan(
    const pp_graph_callbacks* graph,
    uint64_t start_id,
    const pp_astar_options* options,
    pp_search_result* result
) {
    if (options == nullptr) {
        set_error(result, "A* options must not be null");
        return 1;
    }

    pp_search_options search_options;
    search_options.algorithm = PP_SEARCH_ASTAR;
    search_options.has_max_expansions = options->has_max_expansions;
    search_options.max_expansions = options->max_expansions;
    search_options.heuristic_weight = options->heuristic_weight;
    search_options.reserve_nodes = options->reserve_nodes;
    search_options.has_goal_id = 0;
    search_options.goal_id = 0;
    search_options.anytime_weights = nullptr;
    search_options.anytime_weight_count = 0;
    return pp_search_plan(graph, start_id, &search_options, result);
}

extern "C" void pp_search_free_result(pp_search_result* result) {
    if (result == nullptr) {
        return;
    }
    std::free(result->path_ids);
    std::free(result->error_message);
    reset_result(result);
}

extern "C" const char* pp_search_engine_version(void) {
    return kVersion;
}
