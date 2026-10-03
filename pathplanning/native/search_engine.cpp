#include "search_engine.h"

#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <cstring>
#include <deque>
#include <limits>
#include <memory>
#include <mutex>
#include <new>
#include <queue>
#include <stdexcept>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

struct pp_native_graph {
    uint64_t node_count = 0;
    uint64_t grid_width = 0;
    uint64_t grid_height = 0;
    uint64_t grid_depth = 0;
    bool euclidean_grid_heuristic = false;
    std::vector<uint64_t> offsets;
    std::vector<uint64_t> neighbor_ids;
    std::vector<double> edge_costs;
    mutable std::vector<uint64_t> reverse_offsets;
    mutable std::vector<uint64_t> reverse_neighbor_ids;
    mutable std::vector<double> reverse_edge_costs;
    mutable std::once_flag reverse_edges_once;
};

namespace {

constexpr const char* kVersion = "0.4.0";
constexpr double kInfinity = std::numeric_limits<double>::infinity();
constexpr uint64_t kNoGoalId = std::numeric_limits<uint64_t>::max();

#if defined(PP_ENABLE_TRACE) && PP_ENABLE_TRACE
struct TraceRecorder {
    pp_trace_result* result;
    size_t capacity = 0;
    size_t max_events;

    TraceRecorder(pp_trace_result* output, uint64_t max_bytes) : result(output), max_events(0) {
        const uint64_t allowed = max_bytes / sizeof(pp_trace_event);
        max_events = allowed > std::numeric_limits<size_t>::max()
            ? std::numeric_limits<size_t>::max()
            : static_cast<size_t>(allowed);
        *result = pp_trace_result{};
    }

    void append(uint32_t kind, uint64_t node, uint64_t parent, double value, uint32_t side) noexcept {
        if (result->truncated) {
            return;
        }
        if (result->event_count == capacity) {
            if (capacity == max_events) {
                result->truncated = 1;
                return;
            }
            const size_t next = capacity == 0 ? std::min<size_t>(256, max_events)
                : capacity <= max_events / 2 ? capacity * 2 : max_events;
            void* grown = std::realloc(result->events, next * sizeof(pp_trace_event));
            if (grown == nullptr) {
                result->truncated = 1;
                return;
            }
            result->events = static_cast<pp_trace_event*>(grown);
            capacity = next;
        }
        result->events[result->event_count++] = {node, parent, value, kind, side};
    }
};

thread_local TraceRecorder* active_trace = nullptr;

struct TraceScope {
    explicit TraceScope(TraceRecorder* recorder) : previous(active_trace) {
        active_trace = recorder;
    }
    ~TraceScope() { active_trace = previous; }
    TraceRecorder* previous;
};

inline void trace_record(uint32_t kind, uint64_t node, uint64_t parent, double value, uint32_t side) noexcept {
    if (active_trace != nullptr) {
        active_trace->append(kind, node, parent, value, side);
    }
}

#define PP_TRACE(KIND, NODE, PARENT, VALUE, SIDE) \
    trace_record((KIND), (NODE), (PARENT), (VALUE), (SIDE))
#else
#define PP_TRACE(KIND, NODE, PARENT, VALUE, SIDE) ((void)0)
#endif

// Keep costs separate from parent/flag data. Graphs up to UINT32_MAX nodes use
// 32-bit parents; larger graphs retain 64-bit parents.
template <typename ParentId>
struct SearchStates {
    explicit SearchStates(size_t node_count)
        : g_costs(node_count, kInfinity),
          parents(node_count, std::numeric_limits<ParentId>::max()),
          flags(node_count, 0) {}

    double& g_cost(uint64_t node_id) { return g_costs[static_cast<size_t>(node_id)]; }
    double g_cost(uint64_t node_id) const { return g_costs[static_cast<size_t>(node_id)]; }

    uint64_t parent(uint64_t node_id) const {
        return parents[static_cast<size_t>(node_id)];
    }

    bool has_parent(uint64_t node_id) const {
        return parents[static_cast<size_t>(node_id)] != std::numeric_limits<ParentId>::max();
    }

    bool is_closed(uint64_t node_id) const {
        return (flags[static_cast<size_t>(node_id)] & kClosed) != 0;
    }

    void set_parent(uint64_t node_id, uint64_t parent_id) {
        parents[static_cast<size_t>(node_id)] = static_cast<ParentId>(parent_id);
    }

    void mark_closed(uint64_t node_id) { flags[static_cast<size_t>(node_id)] |= kClosed; }

private:
    static constexpr uint8_t kClosed = 1;

    std::vector<double> g_costs;
    std::vector<ParentId> parents;
    std::vector<uint8_t> flags;
};

struct ReverseCsr {
    std::vector<uint64_t> offsets;
    std::vector<uint64_t> neighbor_ids;
    std::vector<double> edge_costs;
};

ReverseCsr build_reverse_edges(const pp_native_graph* graph);
void ensure_reverse_edges(const pp_native_graph* graph);

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

struct AdjacencyView {
    const std::vector<uint64_t>* offsets;
    const std::vector<uint64_t>* neighbor_ids;
    const std::vector<double>* edge_costs;
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

void write_error(char* buffer, size_t capacity, const std::string& message) {
    if (buffer == nullptr || capacity == 0) {
        return;
    }
    const size_t length = std::min(capacity - 1, message.size());
    std::memcpy(buffer, message.data(), length);
    buffer[length] = '\0';
}

void validate_graph_and_result(
    const pp_native_graph* graph,
    const uint8_t* goal_flags,
    uint64_t start_id,
    const pp_search_options* options,
    const pp_search_result* result
) {
    if (graph == nullptr) {
        throw std::invalid_argument("native graph must not be null");
    }
    if (graph->node_count == 0) {
        throw std::invalid_argument("native graph must contain at least one node");
    }
    if (start_id >= graph->node_count) {
        throw std::invalid_argument("start node id is outside the graph");
    }
    if (options == nullptr) {
        throw std::invalid_argument("search options must not be null");
    }
    if (!options->has_goal_id && goal_flags == nullptr) {
        throw std::invalid_argument("goal flags must not be null");
    }
    if (result == nullptr) {
        throw std::invalid_argument("result must not be null");
    }
}

void validate_search_options(const pp_search_options* options, uint64_t node_count) {
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
        case PP_SEARCH_BIDIRECTIONAL_DIJKSTRA:
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
    if (options->algorithm == PP_SEARCH_BIDIRECTIONAL_DIJKSTRA ||
        options->algorithm == PP_SEARCH_BIDIRECTIONAL_ASTAR) {
        if (!options->has_goal_id) {
            throw std::invalid_argument("bidirectional search requires an exact goal id");
        }
        if (options->goal_id != kNoGoalId && options->goal_id >= node_count) {
            throw std::invalid_argument("goal node id is outside the graph");
        }
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

AdjacencyView adjacency(const pp_native_graph* graph, bool reverse) {
    if (reverse) {
        if (graph->reverse_offsets.empty()) {
            throw std::invalid_argument("reverse adjacency is required for backward search");
        }
        return {
            &graph->reverse_offsets,
            &graph->reverse_neighbor_ids,
            &graph->reverse_edge_costs,
        };
    }
    return {&graph->offsets, &graph->neighbor_ids, &graph->edge_costs};
}

bool is_goal(const uint8_t* goal_flags, const pp_search_options* options, uint64_t node_id) {
    return options->has_goal_id ? node_id == options->goal_id : goal_flags[node_id] != 0;
}

bool over_expansion_budget(const pp_search_options* options, uint64_t expanded) {
    return options->has_max_expansions && expanded >= options->max_expansions;
}

double compute_heuristic(
    const pp_native_graph* graph,
    const double* heuristic_values,
    uint64_t node_id,
    const pp_search_options* options,
    double weight
) {
    if (weight == 0.0) {
        return 0.0;
    }
    if (heuristic_values != nullptr) {
        const double value = heuristic_values[node_id];
        return weight * (std::isfinite(value) ? value : 0.0);
    }
    if (!graph->euclidean_grid_heuristic || !options->has_goal_id ||
        options->goal_id >= graph->node_count) {
        return 0.0;
    }

    const uint64_t plane_size = graph->grid_width * graph->grid_height;
    const double dx = static_cast<double>(node_id % graph->grid_width) -
        static_cast<double>(options->goal_id % graph->grid_width);
    const double dy = static_cast<double>((node_id / graph->grid_width) % graph->grid_height) -
        static_cast<double>((options->goal_id / graph->grid_width) % graph->grid_height);
    const double dz = static_cast<double>(node_id / plane_size) -
        static_cast<double>(options->goal_id / plane_size);
    return weight * std::sqrt(dx * dx + dy * dy + dz * dz);
}

template <typename ParentId>
std::vector<uint64_t> reconstruct_path(
    const SearchStates<ParentId>& states,
    uint64_t start_id,
    uint64_t reached_id
) {
    std::vector<uint64_t> reversed;
    uint64_t current = reached_id;
    reversed.push_back(current);

    while (current != start_id) {
        if (!states.has_parent(current)) {
            throw std::runtime_error("missing parent while reconstructing path");
        }
        current = states.parent(current);
        reversed.push_back(current);
    }

    std::reverse(reversed.begin(), reversed.end());
    return reversed;
}

template <typename ParentId>
std::vector<uint64_t> reconstruct_bidirectional_path(
    const SearchStates<ParentId>& forward_states,
    const SearchStates<ParentId>& backward_states,
    uint64_t start_id,
    uint64_t goal_id,
    uint64_t meet_id
) {
    std::vector<uint64_t> path = reconstruct_path(forward_states, start_id, meet_id);

    uint64_t current = meet_id;
    while (current != goal_id) {
        if (!backward_states.has_parent(current)) {
            throw std::runtime_error("missing backward parent while reconstructing path");
        }
        current = backward_states.parent(current);
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
    PP_TRACE(PP_TRACE_SOLUTION, path.empty() ? 0 : path.back(),
        path.size() > 1 ? path[path.size() - 2] : kNoGoalId, path_cost, 0);
}

template <typename ParentId>
int run_linear_search(
    const pp_native_graph* graph,
    const uint8_t* goal_flags,
    uint64_t start_id,
    const pp_search_options* options,
    pp_search_result* result
) {
    const size_t node_count = static_cast<size_t>(graph->node_count);
    SearchStates<ParentId> states(node_count);
    states.g_cost(start_id) = 0.0;
    PP_TRACE(PP_TRACE_DISCOVER, start_id, kNoGoalId, 0.0, 0);

    std::deque<uint64_t> frontier;
    frontier.push_back(start_id);
    uint64_t expanded = 0;
    uint64_t discovered = 1;
    const AdjacencyView edges = adjacency(graph, false);

    while (!frontier.empty()) {
        uint64_t node_id = frontier.front();
        if (options->algorithm == PP_SEARCH_DFS) {
            node_id = frontier.back();
            frontier.pop_back();
        } else {
            frontier.pop_front();
        }

        const double state_g_cost = states.g_cost(node_id);
        ++expanded;
        PP_TRACE(PP_TRACE_EXPAND, node_id, states.has_parent(node_id) ? states.parent(node_id) : kNoGoalId,
            state_g_cost, 0);
        if (is_goal(goal_flags, options, node_id)) {
            set_success_result(
                result,
                expanded,
                discovered,
                state_g_cost,
                reconstruct_path(states, start_id, node_id)
            );
            return 0;
        }

        if (over_expansion_budget(options, expanded)) {
            set_failure_result(result, PP_SEARCH_STOP_MAX_ITERS, expanded, discovered);
            return 0;
        }

        const uint64_t edge_begin = (*edges.offsets)[static_cast<size_t>(node_id)];
        const uint64_t edge_end = (*edges.offsets)[static_cast<size_t>(node_id) + 1];
        auto add_neighbor = [&](uint64_t edge_index) {
            const double edge_cost = (*edges.edge_costs)[static_cast<size_t>(edge_index)];
            if (!std::isfinite(edge_cost)) {
                return;
            }
            const uint64_t neighbor_id = (*edges.neighbor_ids)[static_cast<size_t>(edge_index)];
            if (std::isfinite(states.g_cost(neighbor_id))) {
                return;
            }

            const double tentative = state_g_cost + edge_cost;
            if (!std::isfinite(tentative)) {
                return;
            }

            states.g_cost(neighbor_id) = tentative;
            states.set_parent(neighbor_id, node_id);
            frontier.push_back(neighbor_id);
            ++discovered;
            PP_TRACE(PP_TRACE_DISCOVER, neighbor_id, node_id, tentative, 0);
            PP_TRACE(PP_TRACE_PARENT, neighbor_id, node_id, tentative, 0);
        };

        if (options->algorithm == PP_SEARCH_DFS) {
            for (uint64_t edge_index = edge_end; edge_index > edge_begin; --edge_index) {
                add_neighbor(edge_index - 1);
            }
        } else {
            for (uint64_t edge_index = edge_begin; edge_index < edge_end; ++edge_index) {
                add_neighbor(edge_index);
            }
        }
    }

    set_failure_result(result, PP_SEARCH_STOP_NO_PROGRESS, expanded, discovered);
    return 0;
}

template <typename ParentId>
int run_best_first_search(
    const pp_native_graph* graph,
    const uint8_t* goal_flags,
    const double* heuristic_values,
    uint64_t start_id,
    const pp_search_options* options,
    pp_search_result* result
) {
    const size_t node_count = static_cast<size_t>(graph->node_count);
    SearchStates<ParentId> states(node_count);
    std::priority_queue<QueueEntry, std::vector<QueueEntry>, QueueEntryGreater> open;
    states.g_cost(start_id) = 0.0;
    PP_TRACE(PP_TRACE_DISCOVER, start_id, kNoGoalId, 0.0, 0);

    const bool greedy = options->algorithm == PP_SEARCH_GREEDY_BEST_FIRST;
    const bool use_heuristic =
        options->algorithm == PP_SEARCH_ASTAR ||
        options->algorithm == PP_SEARCH_WEIGHTED_ASTAR ||
        options->algorithm == PP_SEARCH_GREEDY_BEST_FIRST;
    const double heuristic_weight = use_heuristic ? options->heuristic_weight : 0.0;

    uint64_t order = 0;
    const double start_h =
        compute_heuristic(graph, heuristic_values, start_id, options, heuristic_weight);
    open.push(QueueEntry{start_h, start_h, order++, start_id});

    uint64_t expanded = 0;
    uint64_t discovered = 1;
    const AdjacencyView edges = adjacency(graph, false);

    while (!open.empty()) {
        const QueueEntry entry = open.top();
        open.pop();

        if (states.is_closed(entry.node_id)) {
            continue;
        }

        states.mark_closed(entry.node_id);
        ++expanded;
        PP_TRACE(PP_TRACE_EXPAND, entry.node_id,
            states.has_parent(entry.node_id) ? states.parent(entry.node_id) : kNoGoalId,
            states.g_cost(entry.node_id), 0);
        if (is_goal(goal_flags, options, entry.node_id)) {
            set_success_result(
                result,
                expanded,
                discovered,
                states.g_cost(entry.node_id),
                reconstruct_path(states, start_id, entry.node_id)
            );
            return 0;
        }

        if (over_expansion_budget(options, expanded)) {
            set_failure_result(result, PP_SEARCH_STOP_MAX_ITERS, expanded, discovered);
            return 0;
        }

        const uint64_t edge_begin = (*edges.offsets)[static_cast<size_t>(entry.node_id)];
        const uint64_t edge_end = (*edges.offsets)[static_cast<size_t>(entry.node_id) + 1];
        for (uint64_t edge_index = edge_begin; edge_index < edge_end; ++edge_index) {
            const double edge_cost = (*edges.edge_costs)[static_cast<size_t>(edge_index)];
            if (!std::isfinite(edge_cost)) {
                continue;
            }

            const uint64_t neighbor_id = (*edges.neighbor_ids)[static_cast<size_t>(edge_index)];
            const double tentative = states.g_cost(entry.node_id) + edge_cost;
            if (!std::isfinite(tentative)) {
                continue;
            }

            if (states.is_closed(neighbor_id)) {
                continue;
            }
            if (greedy) {
                if (std::isfinite(states.g_cost(neighbor_id))) {
                    continue;
                }
            } else if (tentative >= states.g_cost(neighbor_id)) {
                continue;
            }

            const bool first_discovery = !std::isfinite(states.g_cost(neighbor_id));
            if (first_discovery) {
                ++discovered;
            }
            states.g_cost(neighbor_id) = tentative;
            states.set_parent(neighbor_id, entry.node_id);
            if (first_discovery) {
                PP_TRACE(PP_TRACE_DISCOVER, neighbor_id, entry.node_id, tentative, 0);
            }
            PP_TRACE(PP_TRACE_PARENT, neighbor_id, entry.node_id, tentative, 0);

            const double h_score = compute_heuristic(
                graph,
                heuristic_values,
                neighbor_id,
                options,
                heuristic_weight
            );
            const double f_score = greedy ? h_score : tentative + h_score;
            open.push(QueueEntry{f_score, h_score, order++, neighbor_id});
        }
    }

    set_failure_result(result, PP_SEARCH_STOP_NO_PROGRESS, expanded, discovered);
    return 0;
}

template <typename ParentId>
BidirectionalExpansion expand_bidirectional_frontier(
    const pp_native_graph* graph,
    bool reverse,
    std::priority_queue<QueueEntry, std::vector<QueueEntry>, QueueEntryGreater>* heap,
    SearchStates<ParentId>* states_this,
    const SearchStates<ParentId>& states_other,
    uint64_t* tie_breaker,
    uint64_t* discovered_this,
    uint64_t* expanded
) {
    const AdjacencyView edges = adjacency(graph, reverse);
    while (!heap->empty()) {
        const QueueEntry entry = heap->top();
        heap->pop();

        if (states_this->is_closed(entry.node_id)) {
            continue;
        }

        states_this->mark_closed(entry.node_id);
        ++(*expanded);
        PP_TRACE(PP_TRACE_EXPAND, entry.node_id,
            states_this->has_parent(entry.node_id) ? states_this->parent(entry.node_id) : kNoGoalId,
            states_this->g_cost(entry.node_id), reverse ? 2 : 1);
        BidirectionalExpansion outcome;
        outcome.expanded = true;
        const double other_cost = states_other.g_cost(entry.node_id);
        if (std::isfinite(other_cost)) {
            outcome.best_cost = states_this->g_cost(entry.node_id) + other_cost;
            outcome.meet_id = entry.node_id;
            outcome.has_meet = true;
        }

        const uint64_t edge_begin = (*edges.offsets)[static_cast<size_t>(entry.node_id)];
        const uint64_t edge_end = (*edges.offsets)[static_cast<size_t>(entry.node_id) + 1];
        for (uint64_t edge_index = edge_begin; edge_index < edge_end; ++edge_index) {
            const double edge_cost = (*edges.edge_costs)[static_cast<size_t>(edge_index)];
            if (!std::isfinite(edge_cost)) {
                continue;
            }
            const uint64_t neighbor_id = (*edges.neighbor_ids)[static_cast<size_t>(edge_index)];
            const double tentative = states_this->g_cost(entry.node_id) + edge_cost;
            if (!std::isfinite(tentative)) {
                continue;
            }

            if (states_this->is_closed(neighbor_id) ||
                tentative >= states_this->g_cost(neighbor_id)) {
                continue;
            }

            const bool first_discovery = !std::isfinite(states_this->g_cost(neighbor_id));
            if (first_discovery) {
                ++(*discovered_this);
            }
            states_this->g_cost(neighbor_id) = tentative;
            states_this->set_parent(neighbor_id, entry.node_id);
            if (first_discovery) {
                PP_TRACE(PP_TRACE_DISCOVER, neighbor_id, entry.node_id, tentative, reverse ? 2 : 1);
            }
            PP_TRACE(PP_TRACE_PARENT, neighbor_id, entry.node_id, tentative, reverse ? 2 : 1);
            heap->push(QueueEntry{tentative, 0.0, (*tie_breaker)++, neighbor_id});

            const double other_neighbor_cost = states_other.g_cost(neighbor_id);
            if (std::isfinite(other_neighbor_cost)) {
                const double candidate = tentative + other_neighbor_cost;
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

template <typename ParentId>
int run_bidirectional_search(
    const pp_native_graph* graph,
    uint64_t start_id,
    const pp_search_options* options,
    pp_search_result* result
) {
    const uint64_t goal_id = options->goal_id;
    if (goal_id >= graph->node_count) {
        set_failure_result(result, PP_SEARCH_STOP_NO_PROGRESS, 0, 1);
        return 0;
    }
    if (start_id == goal_id) {
        set_success_result(result, 0, 1, 0.0, std::vector<uint64_t>{start_id});
        return 0;
    }

    const size_t node_count = static_cast<size_t>(graph->node_count);
    SearchStates<ParentId> forward_states(node_count);
    SearchStates<ParentId> backward_states(node_count);
    forward_states.g_cost(start_id) = 0.0;
    backward_states.g_cost(goal_id) = 0.0;
    PP_TRACE(PP_TRACE_DISCOVER, start_id, kNoGoalId, 0.0, 1);
    PP_TRACE(PP_TRACE_DISCOVER, goal_id, kNoGoalId, 0.0, 2);
    ensure_reverse_edges(graph);
    uint64_t forward_discovered = 1;
    uint64_t backward_discovered = 1;

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
                false,
                &forward_heap,
                &forward_states,
                backward_states,
                &tie_breaker,
                &forward_discovered,
                &expanded
            );
        } else {
            local = expand_bidirectional_frontier(
                graph,
                true,
                &backward_heap,
                &backward_states,
                forward_states,
                &tie_breaker,
                &backward_discovered,
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
            PP_TRACE(PP_TRACE_SOLUTION, meet_id, kNoGoalId, best_cost, 0);
        }

        const double forward_bound = forward_heap.empty() ? kInfinity : forward_heap.top().f_score;
        const double backward_bound = backward_heap.empty() ? kInfinity : backward_heap.top().f_score;
        if (has_meet && (forward_bound + backward_bound) >= best_cost) {
            break;
        }
    }

    const uint64_t node_count_seen = forward_discovered + backward_discovered;
    if (!has_meet) {
        set_failure_result(
            result,
            options->has_max_expansions ? PP_SEARCH_STOP_MAX_ITERS : PP_SEARCH_STOP_NO_PROGRESS,
            expanded,
            node_count_seen
        );
        return 0;
    }

    set_success_result(
        result,
        expanded,
        node_count_seen,
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

template <typename ParentId>
int run_bidirectional_astar_search(
    const pp_native_graph* graph,
    const double* heuristic_values,
    uint64_t start_id,
    const pp_search_options* options,
    pp_search_result* result
) {
    const uint64_t goal_id = options->goal_id;
    if (goal_id >= graph->node_count) {
        set_failure_result(result, PP_SEARCH_STOP_NO_PROGRESS, 0, 1);
        return 0;
    }
    if (start_id == goal_id) {
        set_success_result(result, 0, 1, 0.0, std::vector<uint64_t>{start_id});
        return 0;
    }

    auto potential = [&](uint64_t node_id) {
        return compute_heuristic(graph, heuristic_values, node_id, options, 1.0);
    };
    // A consistent potential makes every reduced edge non-negative. Validate
    // the whole CSR once so the bidirectional stopping bound remains sound.
    for (uint64_t source = 0; source < graph->node_count; ++source) {
        const uint64_t edge_begin = graph->offsets[static_cast<size_t>(source)];
        const uint64_t edge_end = graph->offsets[static_cast<size_t>(source) + 1];
        const double source_h = potential(source);
        for (uint64_t edge = edge_begin; edge < edge_end; ++edge) {
            const double cost = graph->edge_costs[static_cast<size_t>(edge)];
            if (!std::isfinite(cost)) {
                continue;
            }
            const uint64_t target = graph->neighbor_ids[static_cast<size_t>(edge)];
            const double target_h = potential(target);
            const double reduced_cost = cost + target_h - source_h;
            const double tolerance = 1e-12 * std::max(
                1.0,
                std::max(std::abs(cost), std::max(std::abs(source_h), std::abs(target_h)))
            );
            if (reduced_cost < -tolerance) {
                throw std::invalid_argument(
                    "bidirectional_astar requires a consistent heuristic"
                );
            }
        }
    }

    const size_t node_count = static_cast<size_t>(graph->node_count);
    SearchStates<ParentId> forward_states(node_count);
    SearchStates<ParentId> backward_states(node_count);
    forward_states.g_cost(start_id) = 0.0;
    backward_states.g_cost(goal_id) = 0.0;
    PP_TRACE(PP_TRACE_DISCOVER, start_id, kNoGoalId, 0.0, 1);
    PP_TRACE(PP_TRACE_DISCOVER, goal_id, kNoGoalId, 0.0, 2);
    ensure_reverse_edges(graph);
    uint64_t forward_discovered = 1;
    uint64_t backward_discovered = 1;

    std::priority_queue<QueueEntry, std::vector<QueueEntry>, QueueEntryGreater> forward_heap;
    std::priority_queue<QueueEntry, std::vector<QueueEntry>, QueueEntryGreater> backward_heap;
    uint64_t tie_breaker = 0;
    forward_heap.push(QueueEntry{0.0, 0.0, tie_breaker++, start_id});
    backward_heap.push(QueueEntry{0.0, 0.0, tie_breaker++, goal_id});

    double best_reduced_cost = kInfinity;
    uint64_t meet_id = 0;
    bool has_meet = false;
    uint64_t expanded = 0;

    while (!forward_heap.empty() && !backward_heap.empty()) {
        if (over_expansion_budget(options, expanded)) {
            break;
        }

        const bool reverse = forward_heap.top().f_score > backward_heap.top().f_score;
        auto* heap = reverse ? &backward_heap : &forward_heap;
        auto* states_this = reverse ? &backward_states : &forward_states;
        const auto& states_other = reverse ? forward_states : backward_states;
        const AdjacencyView edges = adjacency(graph, reverse);
        const QueueEntry entry = heap->top();
        heap->pop();
        if (states_this->is_closed(entry.node_id)) {
            continue;
        }
        states_this->mark_closed(entry.node_id);
        ++expanded;
        PP_TRACE(PP_TRACE_EXPAND, entry.node_id,
            states_this->has_parent(entry.node_id) ? states_this->parent(entry.node_id) : kNoGoalId,
            states_this->g_cost(entry.node_id), reverse ? 2 : 1);

        const double other_cost = states_other.g_cost(entry.node_id);
        if (std::isfinite(other_cost)) {
            const double candidate = states_this->g_cost(entry.node_id) + other_cost;
            if (candidate < best_reduced_cost) {
                best_reduced_cost = candidate;
                meet_id = entry.node_id;
                has_meet = true;
                PP_TRACE(PP_TRACE_SOLUTION, meet_id, kNoGoalId,
                    candidate + potential(start_id) - potential(goal_id), 0);
            }
        }

        const uint64_t edge_begin = (*edges.offsets)[static_cast<size_t>(entry.node_id)];
        const uint64_t edge_end = (*edges.offsets)[static_cast<size_t>(entry.node_id) + 1];
        const double current_h = potential(entry.node_id);
        for (uint64_t edge = edge_begin; edge < edge_end; ++edge) {
            const double cost = (*edges.edge_costs)[static_cast<size_t>(edge)];
            if (!std::isfinite(cost)) {
                continue;
            }
            const uint64_t neighbor = (*edges.neighbor_ids)[static_cast<size_t>(edge)];
            const double neighbor_h = potential(neighbor);
            const double reduced_cost = reverse
                ? cost + current_h - neighbor_h
                : cost + neighbor_h - current_h;
            const double reduced_nonnegative = std::max(0.0, reduced_cost);
            const double tentative = states_this->g_cost(entry.node_id) + reduced_nonnegative;
            if (!std::isfinite(tentative) || states_this->is_closed(neighbor) ||
                tentative >= states_this->g_cost(neighbor)) {
                continue;
            }
            const bool first_discovery = !std::isfinite(states_this->g_cost(neighbor));
            if (first_discovery) {
                if (reverse) {
                    ++backward_discovered;
                } else {
                    ++forward_discovered;
                }
            }
            states_this->g_cost(neighbor) = tentative;
            states_this->set_parent(neighbor, entry.node_id);
            if (first_discovery) {
                PP_TRACE(PP_TRACE_DISCOVER, neighbor, entry.node_id, tentative, reverse ? 2 : 1);
            }
            PP_TRACE(PP_TRACE_PARENT, neighbor, entry.node_id, tentative, reverse ? 2 : 1);
            heap->push(QueueEntry{tentative, 0.0, tie_breaker++, neighbor});
            const double opposite_cost = states_other.g_cost(neighbor);
            if (std::isfinite(opposite_cost) && tentative + opposite_cost < best_reduced_cost) {
                best_reduced_cost = tentative + opposite_cost;
                meet_id = neighbor;
                has_meet = true;
                PP_TRACE(PP_TRACE_SOLUTION, meet_id, kNoGoalId,
                    best_reduced_cost + potential(start_id) - potential(goal_id), 0);
            }
        }

        const double forward_bound = forward_heap.empty() ? kInfinity : forward_heap.top().f_score;
        const double backward_bound = backward_heap.empty() ? kInfinity : backward_heap.top().f_score;
        if (has_meet && forward_bound + backward_bound >= best_reduced_cost) {
            break;
        }
    }

    const uint64_t discovered = forward_discovered + backward_discovered;
    if (!has_meet) {
        set_failure_result(
            result,
            options->has_max_expansions ? PP_SEARCH_STOP_MAX_ITERS : PP_SEARCH_STOP_NO_PROGRESS,
            expanded,
            discovered
        );
        return 0;
    }

    const double original_cost = best_reduced_cost + potential(start_id) - potential(goal_id);
    set_success_result(
        result,
        expanded,
        discovered,
        original_cost,
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

template <typename ParentId>
int run_anytime_astar_search(
    const pp_native_graph* graph,
    const uint8_t* goal_flags,
    const double* heuristic_values,
    uint64_t start_id,
    const pp_search_options* options,
    pp_search_result* result
) {
    uint64_t total_iters = 0;
    uint64_t total_nodes = 0;
    double best_cost = kInfinity;
    std::vector<uint64_t> best_path;

    for (size_t index = 0; index < options->anytime_weight_count; ++index) {
        PP_TRACE(PP_TRACE_PHASE, static_cast<uint64_t>(index), kNoGoalId,
            options->anytime_weights[index], 0);
        pp_search_options weighted_options = *options;
        weighted_options.algorithm = PP_SEARCH_WEIGHTED_ASTAR;
        weighted_options.heuristic_weight = options->anytime_weights[index];

        pp_search_result candidate;
        reset_result(&candidate);
        const int status = run_best_first_search<ParentId>(
            graph,
            goal_flags,
            heuristic_values,
            start_id,
            &weighted_options,
            &candidate
        );
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
        if (!best_path.empty() && options->anytime_weights[index] <= 1.0) {
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

template <typename ParentId>
int run_search_by_algorithm(
    const pp_native_graph* graph,
    const uint8_t* goal_flags,
    const double* heuristic_values,
    uint64_t start_id,
    const pp_search_options* options,
    pp_search_result* result
) {
    switch (options->algorithm) {
        case PP_SEARCH_BFS:
        case PP_SEARCH_DFS:
            return run_linear_search<ParentId>(graph, goal_flags, start_id, options, result);
        case PP_SEARCH_GREEDY_BEST_FIRST:
        case PP_SEARCH_ASTAR:
        case PP_SEARCH_DIJKSTRA:
        case PP_SEARCH_WEIGHTED_ASTAR:
            return run_best_first_search<ParentId>(
                graph,
                goal_flags,
                heuristic_values,
                start_id,
                options,
                result
            );
        case PP_SEARCH_BIDIRECTIONAL_DIJKSTRA:
            return run_bidirectional_search<ParentId>(graph, start_id, options, result);
        case PP_SEARCH_BIDIRECTIONAL_ASTAR:
            return run_bidirectional_astar_search<ParentId>(
                graph,
                heuristic_values,
                start_id,
                options,
                result
            );
        case PP_SEARCH_ANYTIME_ASTAR:
            return run_anytime_astar_search<ParentId>(
                graph,
                goal_flags,
                heuristic_values,
                start_id,
                options,
                result
            );
        default:
            throw std::invalid_argument("unknown search algorithm");
    }
}

ReverseCsr build_reverse_edges(const pp_native_graph* graph) {
    const size_t node_count = static_cast<size_t>(graph->node_count);
    ReverseCsr reverse;
    reverse.offsets.assign(node_count + 1, 0);
    for (uint64_t neighbor_id : graph->neighbor_ids) {
        ++reverse.offsets[static_cast<size_t>(neighbor_id) + 1];
    }
    for (size_t node_id = 1; node_id < reverse.offsets.size(); ++node_id) {
        reverse.offsets[node_id] += reverse.offsets[node_id - 1];
    }

    reverse.neighbor_ids.resize(graph->neighbor_ids.size());
    reverse.edge_costs.resize(graph->edge_costs.size());
    for (uint64_t source_id = 0; source_id < graph->node_count; ++source_id) {
        const uint64_t edge_begin = graph->offsets[static_cast<size_t>(source_id)];
        const uint64_t edge_end = graph->offsets[static_cast<size_t>(source_id) + 1];
        for (uint64_t edge_index = edge_begin; edge_index < edge_end; ++edge_index) {
            const uint64_t target_id = graph->neighbor_ids[static_cast<size_t>(edge_index)];
            const uint64_t reverse_index = reverse.offsets[static_cast<size_t>(target_id)]++;
            reverse.neighbor_ids[static_cast<size_t>(reverse_index)] = source_id;
            reverse.edge_costs[static_cast<size_t>(reverse_index)] =
                graph->edge_costs[static_cast<size_t>(edge_index)];
        }
    }

    // The prefix offsets served as insertion cursors above; shift them back to
    // row boundaries without allocating a second node-sized cursor array.
    for (size_t node_id = node_count; node_id > 0; --node_id) {
        reverse.offsets[node_id] = reverse.offsets[node_id - 1];
    }
    reverse.offsets[0] = 0;
    return reverse;
}

void ensure_reverse_edges(const pp_native_graph* graph) {
    std::call_once(graph->reverse_edges_once, [graph]() {
        ReverseCsr reverse = build_reverse_edges(graph);
        graph->reverse_offsets = std::move(reverse.offsets);
        graph->reverse_neighbor_ids = std::move(reverse.neighbor_ids);
        graph->reverse_edge_costs = std::move(reverse.edge_costs);
    });
}

}  // namespace

extern "C" int pp_graph_create_csr(
    uint64_t node_count,
    uint64_t edge_count,
    const uint64_t* offsets,
    const uint64_t* neighbor_ids,
    const double* edge_costs,
    pp_native_graph** out_graph,
    char* error_message,
    size_t error_capacity
) {
    if (out_graph == nullptr) {
        write_error(error_message, error_capacity, "out_graph must not be null");
        return 1;
    }
    *out_graph = nullptr;
    try {
        if (node_count == 0) {
            throw std::invalid_argument("graph must contain at least one node");
        }
        if (node_count >= static_cast<uint64_t>(std::numeric_limits<size_t>::max())) {
            throw std::invalid_argument("node count is too large for this platform");
        }
        if (edge_count > static_cast<uint64_t>(std::numeric_limits<size_t>::max())) {
            throw std::invalid_argument("edge count is too large for this platform");
        }
        if (offsets == nullptr) {
            throw std::invalid_argument("CSR offsets must not be null");
        }
        if (offsets[0] != 0 || offsets[node_count] != edge_count) {
            throw std::invalid_argument("CSR offsets must start at zero and end at edge_count");
        }
        if (edge_count > 0 && (neighbor_ids == nullptr || edge_costs == nullptr)) {
            throw std::invalid_argument("CSR edge arrays must not be null when edges exist");
        }

        for (uint64_t node_id = 0; node_id < node_count; ++node_id) {
            if (offsets[node_id] > offsets[node_id + 1]) {
                throw std::invalid_argument("CSR offsets must be monotonically non-decreasing");
            }
        }
        for (uint64_t edge_index = 0; edge_index < edge_count; ++edge_index) {
            if (neighbor_ids[edge_index] >= node_count) {
                throw std::invalid_argument("neighbor id is outside the graph");
            }
        }

        auto graph = std::make_unique<pp_native_graph>();
        graph->node_count = node_count;
        graph->offsets.assign(offsets, offsets + static_cast<size_t>(node_count) + 1);
        if (edge_count > 0) {
            graph->neighbor_ids.assign(
                neighbor_ids,
                neighbor_ids + static_cast<size_t>(edge_count)
            );
            graph->edge_costs.assign(edge_costs, edge_costs + static_cast<size_t>(edge_count));
        }
        *out_graph = graph.release();
        if (error_message != nullptr && error_capacity > 0) {
            error_message[0] = '\0';
        }
        return 0;
    } catch (const std::exception& exc) {
        write_error(error_message, error_capacity, exc.what());
        return 1;
    } catch (...) {
        write_error(error_message, error_capacity, "unknown native graph creation error");
        return 1;
    }
}

#if defined(PP_ENABLE_TRACE) && PP_ENABLE_TRACE
extern "C" int pp_graph_create_csr_view(
    const pp_graph_csr_view* view,
    pp_native_graph** out_graph,
    char* error_message,
    size_t error_capacity
) {
    if (view == nullptr || out_graph == nullptr) {
        write_error(error_message, error_capacity, "CSR view and out_graph must not be null");
        return 1;
    }
    *out_graph = nullptr;
    pp_native_graph* raw_graph = nullptr;
    const int status = pp_graph_create_csr(
        view->node_count,
        view->edge_count,
        view->offsets,
        view->neighbor_ids,
        view->edge_costs,
        &raw_graph,
        error_message,
        error_capacity
    );
    if (status != 0) {
        return status;
    }

    std::unique_ptr<pp_native_graph> graph(raw_graph);
    try {
        const bool has_grid_metadata = view->grid_width != 0 || view->grid_height != 0 ||
            view->grid_depth != 0 || view->euclidean_grid_heuristic != 0;
        if (has_grid_metadata) {
            if (view->grid_width == 0 || view->grid_height == 0 || view->grid_depth == 0 ||
                view->grid_width > view->node_count / view->grid_height ||
                view->grid_width * view->grid_height > view->node_count / view->grid_depth ||
                view->grid_width * view->grid_height * view->grid_depth != view->node_count) {
                throw std::invalid_argument("CSR grid metadata does not match node count");
            }
            graph->grid_width = view->grid_width;
            graph->grid_height = view->grid_height;
            graph->grid_depth = view->grid_depth;
            graph->euclidean_grid_heuristic = view->euclidean_grid_heuristic != 0;
        }
        *out_graph = graph.release();
        if (error_message != nullptr && error_capacity > 0) {
            error_message[0] = '\0';
        }
        return 0;
    } catch (const std::exception& exc) {
        write_error(error_message, error_capacity, exc.what());
        return 1;
    }
}

extern "C" uint64_t pp_graph_storage_bytes(const pp_native_graph* graph) {
    if (graph == nullptr) return 0;

    uint64_t total = sizeof(*graph);
    const auto add_capacity = [&total](size_t count, size_t item_size) {
        if (item_size != 0 && count > std::numeric_limits<uint64_t>::max() / item_size) {
            return false;
        }
        const uint64_t bytes = static_cast<uint64_t>(count * item_size);
        if (bytes > std::numeric_limits<uint64_t>::max() - total) return false;
        total += bytes;
        return true;
    };

    if (!add_capacity(graph->offsets.capacity(), sizeof(uint64_t)) ||
        !add_capacity(graph->neighbor_ids.capacity(), sizeof(uint64_t)) ||
        !add_capacity(graph->edge_costs.capacity(), sizeof(double)) ||
        !add_capacity(graph->reverse_offsets.capacity(), sizeof(uint64_t)) ||
        !add_capacity(graph->reverse_neighbor_ids.capacity(), sizeof(uint64_t)) ||
        !add_capacity(graph->reverse_edge_costs.capacity(), sizeof(double))) {
        return std::numeric_limits<uint64_t>::max();
    }
    return total;
}
#endif

extern "C" int pp_graph_export_csr_view(
    const pp_native_graph* graph,
    pp_graph_csr_view* out_view
) {
    if (graph == nullptr || out_view == nullptr) {
        return 1;
    }
    *out_view = {
        graph->node_count,
        static_cast<uint64_t>(graph->neighbor_ids.size()),
        graph->offsets.data(),
        graph->neighbor_ids.data(),
        graph->edge_costs.data(),
        graph->grid_width,
        graph->grid_height,
        graph->grid_depth,
        graph->euclidean_grid_heuristic ? 1 : 0,
    };
    return 0;
}

extern "C" int pp_graph_create_grid_ex(
    uint64_t width,
    uint64_t height,
    uint64_t depth,
    uint64_t dimensions,
    int euclidean_heuristic,
    const int32_t* motions,
    size_t motion_count,
    const uint8_t* valid_nodes,
    pp_native_graph** out_graph,
    char* error_message,
    size_t error_capacity
) {
    try {
        if (out_graph == nullptr) {
            throw std::invalid_argument("out_graph must not be null");
        }
        *out_graph = nullptr;
        if (width == 0 || height == 0 || depth == 0) {
            throw std::invalid_argument("grid dimensions must be positive");
        }
        if ((dimensions != 2 && dimensions != 3) || (dimensions == 2 && depth != 1)) {
            throw std::invalid_argument("grid must have two or three dimensions");
        }
        if (motion_count == 0 || motions == nullptr) {
            throw std::invalid_argument("grid motions must not be empty");
        }
        const uint64_t max_coordinate =
            static_cast<uint64_t>(std::numeric_limits<int64_t>::max());
        if (width > max_coordinate || height > max_coordinate || depth > max_coordinate) {
            throw std::invalid_argument("grid dimensions exceed supported coordinate range");
        }
        if (width > std::numeric_limits<uint64_t>::max() / height ||
            width * height > std::numeric_limits<uint64_t>::max() / depth) {
            throw std::invalid_argument("grid node count overflows uint64");
        }

        const uint64_t node_count = width * height * depth;
        if (node_count >= static_cast<uint64_t>(std::numeric_limits<size_t>::max())) {
            throw std::invalid_argument("grid is too large for this platform");
        }

        struct GridMotion {
            int32_t dx;
            int32_t dy;
            int32_t dz;
            double cost;
        };
        std::vector<GridMotion> valid_motions;
        valid_motions.reserve(motion_count);
        for (size_t index = 0; index < motion_count; ++index) {
            const int32_t dx = motions[index * 3];
            const int32_t dy = motions[index * 3 + 1];
            const int32_t dz = dimensions == 3 ? motions[index * 3 + 2] : 0;
            if (std::abs(static_cast<int64_t>(dx)) > 1 ||
                std::abs(static_cast<int64_t>(dy)) > 1 ||
                std::abs(static_cast<int64_t>(dz)) > 1 ||
                (dx == 0 && dy == 0 && dz == 0)) {
                continue;
            }
            const double squared = static_cast<double>(dx * dx + dy * dy + dz * dz);
            valid_motions.push_back(GridMotion{dx, dy, dz, std::sqrt(squared)});
        }

        std::vector<uint64_t> offsets;
        std::vector<uint64_t> neighbor_ids;
        std::vector<double> edge_costs;
        offsets.reserve(static_cast<size_t>(node_count) + 1);
        size_t valid_node_count = static_cast<size_t>(node_count);
        if (valid_nodes != nullptr) {
            valid_node_count = static_cast<size_t>(std::count_if(
                valid_nodes,
                valid_nodes + static_cast<size_t>(node_count),
                [](uint8_t is_valid) { return is_valid != 0; }
            ));
        }
        const bool dense_grid =
            valid_nodes == nullptr || valid_node_count >= node_count - node_count / 4;
        if (dense_grid && !valid_motions.empty() &&
            valid_node_count <= neighbor_ids.max_size() / valid_motions.size() &&
            valid_node_count <= edge_costs.max_size() / valid_motions.size()) {
            const size_t edge_capacity = valid_node_count * valid_motions.size();
            neighbor_ids.reserve(edge_capacity);
            edge_costs.reserve(edge_capacity);
        }
        offsets.push_back(0);
        for (uint64_t node_id = 0; node_id < node_count; ++node_id) {
            if (valid_nodes == nullptr || valid_nodes[node_id] != 0) {
                const uint64_t x = node_id % width;
                const uint64_t y = (node_id / width) % height;
                const uint64_t z = node_id / (width * height);
                for (const GridMotion& motion : valid_motions) {
                    const int64_t next_x = static_cast<int64_t>(x) + motion.dx;
                    const int64_t next_y = static_cast<int64_t>(y) + motion.dy;
                    const int64_t next_z = static_cast<int64_t>(z) + motion.dz;
                    if (next_x < 0 || next_x >= static_cast<int64_t>(width) ||
                        next_y < 0 || next_y >= static_cast<int64_t>(height) ||
                        next_z < 0 || next_z >= static_cast<int64_t>(depth)) {
                        continue;
                    }
                    const uint64_t neighbor_id =
                        static_cast<uint64_t>(next_x) +
                        width * (static_cast<uint64_t>(next_y) + height * static_cast<uint64_t>(next_z));
                    if (valid_nodes != nullptr && valid_nodes[neighbor_id] == 0) {
                        continue;
                    }
                    neighbor_ids.push_back(neighbor_id);
                    edge_costs.push_back(motion.cost);
                }
            }
            offsets.push_back(static_cast<uint64_t>(neighbor_ids.size()));
        }

        auto graph = std::make_unique<pp_native_graph>();
        graph->node_count = node_count;
        graph->grid_width = width;
        graph->grid_height = height;
        graph->grid_depth = depth;
        graph->euclidean_grid_heuristic = euclidean_heuristic != 0;
        graph->offsets = std::move(offsets);
        graph->neighbor_ids = std::move(neighbor_ids);
        graph->edge_costs = std::move(edge_costs);
        *out_graph = graph.release();
        if (error_message != nullptr && error_capacity > 0) {
            error_message[0] = '\0';
        }
        return 0;
    } catch (const std::exception& exc) {
        write_error(error_message, error_capacity, exc.what());
        return 1;
    } catch (...) {
        write_error(error_message, error_capacity, "unknown native grid creation error");
        return 1;
    }
}

extern "C" int pp_graph_create_grid(
    uint64_t width,
    uint64_t height,
    uint64_t depth,
    uint64_t dimensions,
    const int32_t* motions,
    size_t motion_count,
    const uint8_t* valid_nodes,
    pp_native_graph** out_graph,
    char* error_message,
    size_t error_capacity
) {
    return pp_graph_create_grid_ex(
        width,
        height,
        depth,
        dimensions,
        0,
        motions,
        motion_count,
        valid_nodes,
        out_graph,
        error_message,
        error_capacity
    );
}

extern "C" void pp_graph_free(pp_native_graph* graph) {
    delete graph;
}

extern "C" int pp_native_search_plan(
    const pp_native_graph* graph,
    const uint8_t* goal_flags,
    const double* heuristic_values,
    uint64_t start_id,
    const pp_search_options* options,
    pp_search_result* result
) {
    try {
        validate_graph_and_result(
            graph,
            goal_flags,
            start_id,
            options,
            result
        );
        validate_search_options(options, graph->node_count);
        reset_result(result);

        // Keep max(ParentId) available as the no-parent sentinel.
        if (graph->node_count <= std::numeric_limits<uint32_t>::max()) {
            return run_search_by_algorithm<uint32_t>(
                graph,
                goal_flags,
                heuristic_values,
                start_id,
                options,
                result
            );
        }
        return run_search_by_algorithm<uint64_t>(
            graph,
            goal_flags,
            heuristic_values,
            start_id,
            options,
            result
        );
    } catch (const std::exception& exc) {
        set_error(result, exc.what());
        return 1;
    } catch (...) {
        set_error(result, "unknown native search error");
        return 1;
    }
}

#if defined(PP_ENABLE_TRACE) && PP_ENABLE_TRACE
extern "C" int pp_native_search_plan_traced(
    const pp_native_graph* graph,
    const uint8_t* goal_flags,
    const double* heuristic_values,
    uint64_t start_id,
    const pp_search_options* options,
    pp_search_result* result,
    uint64_t max_bytes,
    pp_trace_result* trace
) {
    if (trace == nullptr) {
        set_error(result, "trace result must not be null");
        return 1;
    }
    TraceRecorder recorder(trace, max_bytes);
    TraceScope scope(&recorder);
    return pp_native_search_plan(
        graph, goal_flags, heuristic_values, start_id, options, result
    );
}

extern "C" uint32_t pp_search_trace_abi_version(void) {
    return PP_SEARCH_TRACE_ABI_VERSION;
}

extern "C" void pp_search_trace_free_result(pp_trace_result* trace) {
    if (trace == nullptr) {
        return;
    }
    std::free(trace->events);
    std::free(trace->points);
    *trace = pp_trace_result{};
}
#endif

extern "C" int pp_search_plan(
    const pp_graph_callbacks* callbacks,
    uint64_t start_id,
    const pp_search_options* options,
    pp_search_result* result
) {
    constexpr size_t kMaxCallbackGraphNodes = 1'000'000;
    try {
        if (callbacks == nullptr) {
            throw std::invalid_argument("graph callbacks must not be null");
        }
        if (callbacks->is_goal == nullptr || callbacks->heuristic == nullptr ||
            callbacks->neighbors == nullptr) {
            throw std::invalid_argument("all graph callbacks must be provided");
        }
        if (options == nullptr) {
            throw std::invalid_argument("search options must not be null");
        }
        if (result == nullptr) {
            throw std::invalid_argument("result must not be null");
        }
        reset_result(result);

        std::unordered_map<uint64_t, uint64_t> dense_ids;
        std::vector<uint64_t> original_ids;
        std::vector<std::vector<std::pair<uint64_t, double>>> rows;
        auto ensure_node = [&](uint64_t original_id) {
            auto found = dense_ids.find(original_id);
            if (found != dense_ids.end()) {
                return found->second;
            }
            if (original_ids.size() >= kMaxCallbackGraphNodes) {
                throw std::runtime_error(
                    "callback graph exceeds the legacy snapshot limit of 1,000,000 nodes"
                );
            }
            const uint64_t dense_id = static_cast<uint64_t>(original_ids.size());
            dense_ids.emplace(original_id, dense_id);
            original_ids.push_back(original_id);
            rows.emplace_back();
            return dense_id;
        };

        const uint64_t dense_start_id = ensure_node(start_id);
        std::vector<uint64_t> pending_nodes{dense_start_id};
        std::vector<uint8_t> queued{1};
        if (options->has_goal_id) {
            ensure_node(options->goal_id);
            queued.resize(original_ids.size(), 0);
        }

        for (size_t cursor = 0; cursor < pending_nodes.size(); ++cursor) {
            const uint64_t dense_id = pending_nodes[cursor];
            const uint64_t original_id = original_ids[static_cast<size_t>(dense_id)];
            const uint64_t* neighbor_ids = nullptr;
            const double* edge_costs = nullptr;
            size_t edge_count = 0;
            if (callbacks->neighbors(
                    callbacks->user_data,
                    original_id,
                    &neighbor_ids,
                    &edge_costs,
                    &edge_count
                ) != 0) {
                throw std::runtime_error("neighbors callback failed during graph snapshot");
            }
            if (edge_count > 0 && (neighbor_ids == nullptr || edge_costs == nullptr)) {
                throw std::runtime_error("neighbors callback returned null edge arrays");
            }

            std::vector<std::pair<uint64_t, double>> row;
            row.reserve(edge_count);
            for (size_t edge = 0; edge < edge_count; ++edge) {
                const uint64_t dense_neighbor = ensure_node(neighbor_ids[edge]);
                row.emplace_back(dense_neighbor, edge_costs[edge]);
                if (queued.size() < original_ids.size()) {
                    queued.resize(original_ids.size(), 0);
                }
                if (queued[static_cast<size_t>(dense_neighbor)] == 0) {
                    pending_nodes.push_back(dense_neighbor);
                    queued[static_cast<size_t>(dense_neighbor)] = 1;
                }
            }
            rows[static_cast<size_t>(dense_id)] = std::move(row);
        }

        std::vector<uint64_t> offsets;
        std::vector<uint64_t> dense_neighbors;
        std::vector<double> snapshot_costs;
        std::vector<uint8_t> goal_flags(original_ids.size(), 0);
        std::vector<double> heuristic_values(original_ids.size(), 0.0);
        offsets.reserve(original_ids.size() + 1);
        offsets.push_back(0);
        for (const auto& row : rows) {
            for (const auto& edge : row) {
                dense_neighbors.push_back(edge.first);
                snapshot_costs.push_back(edge.second);
            }
            offsets.push_back(static_cast<uint64_t>(dense_neighbors.size()));
        }

        const bool use_heuristic =
            options->algorithm == PP_SEARCH_GREEDY_BEST_FIRST ||
            options->algorithm == PP_SEARCH_ASTAR ||
            options->algorithm == PP_SEARCH_WEIGHTED_ASTAR ||
            options->algorithm == PP_SEARCH_ANYTIME_ASTAR;
        for (size_t node_id = 0; node_id < original_ids.size(); ++node_id) {
            int is_goal_value = 0;
            if (callbacks->is_goal(callbacks->user_data, original_ids[node_id], &is_goal_value) != 0) {
                throw std::runtime_error("is_goal callback failed during graph snapshot");
            }
            if (is_goal_value != 0 && is_goal_value != 1) {
                throw std::runtime_error("is_goal callback returned a value other than 0 or 1");
            }
            goal_flags[node_id] = is_goal_value != 0 ? 1 : 0;

            if (use_heuristic) {
                double heuristic_value = 0.0;
                if (callbacks->heuristic(
                        callbacks->user_data,
                        original_ids[node_id],
                        &heuristic_value
                    ) != 0) {
                    throw std::runtime_error("heuristic callback failed during graph snapshot");
                }
                heuristic_values[node_id] = heuristic_value;
            }
        }

        pp_native_graph* raw_graph = nullptr;
        char graph_error[512] = {};
        const int graph_status = pp_graph_create_csr(
            static_cast<uint64_t>(original_ids.size()),
            static_cast<uint64_t>(dense_neighbors.size()),
            offsets.data(),
            dense_neighbors.data(),
            snapshot_costs.data(),
            &raw_graph,
            graph_error,
            sizeof(graph_error)
        );
        std::unique_ptr<pp_native_graph, decltype(&pp_graph_free)> graph(raw_graph, pp_graph_free);
        if (graph_status != 0) {
            throw std::runtime_error(graph_error[0] == '\0' ? "native graph snapshot failed" : graph_error);
        }

        pp_search_options dense_options = *options;
        const auto start = dense_ids.find(start_id);
        if (start == dense_ids.end()) {
            throw std::runtime_error("start node is missing from callback graph snapshot");
        }
        if (options->has_goal_id) {
            dense_options.goal_id = dense_ids.at(options->goal_id);
        }

        const int status = pp_native_search_plan(
            graph.get(),
            goal_flags.data(),
            heuristic_values.data(),
            start->second,
            &dense_options,
            result
        );
        if (status != 0 || result->path_ids == nullptr) {
            return status;
        }

        uint64_t* original_path = static_cast<uint64_t*>(
            std::malloc(result->path_length * sizeof(uint64_t))
        );
        if (original_path == nullptr) {
            pp_search_free_result(result);
            throw std::bad_alloc();
        }
        for (size_t index = 0; index < result->path_length; ++index) {
            original_path[index] = original_ids[static_cast<size_t>(result->path_ids[index])];
        }
        std::free(result->path_ids);
        result->path_ids = original_path;
        return 0;
    } catch (const std::exception& exc) {
        set_error(result, exc.what());
        return 1;
    } catch (...) {
        set_error(result, "unknown callback graph snapshot error");
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

    pp_search_options search_options{};
    search_options.algorithm = PP_SEARCH_ASTAR;
    search_options.has_max_expansions = options->has_max_expansions;
    search_options.max_expansions = options->max_expansions;
    search_options.heuristic_weight = options->heuristic_weight;
    search_options.reserve_nodes = options->reserve_nodes;
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

extern "C" uint32_t pp_search_abi_version(void) {
    return PP_SEARCH_ABI_VERSION;
}

extern "C" const char* pp_search_engine_version(void) {
    return kVersion;
}
