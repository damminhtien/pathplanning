#include "search_engine.h"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdlib>
#include <cstring>
#include <functional>
#include <limits>
#include <memory>
#include <queue>
#include <stdexcept>
#include <string>
#include <tuple>
#include <unordered_map>
#include <unordered_set>
#include <utility>
#include <vector>

namespace {

constexpr double kInfinity = std::numeric_limits<double>::infinity();
constexpr uint64_t kNoNode = std::numeric_limits<uint64_t>::max();

struct EdgeKey {
    uint64_t source;
    uint64_t target;

    bool operator==(const EdgeKey& other) const {
        return source == other.source && target == other.target;
    }
};

struct EdgeKeyHash {
    size_t operator()(const EdgeKey& key) const {
        const size_t source_hash = std::hash<uint64_t>{}(key.source);
        const size_t target_hash = std::hash<uint64_t>{}(key.target);
        return source_hash ^ (target_hash + 0x9e3779b9U + (source_hash << 6) + (source_hash >> 2));
    }
};

struct DStarKey {
    double first;
    double second;
};

struct DStarQueueEntry {
    DStarKey key;
    uint64_t sequence;
    uint64_t node;
    uint64_t version;
};

struct DStarQueueGreater {
    bool operator()(const DStarQueueEntry& left, const DStarQueueEntry& right) const {
        return std::tie(left.key.first, left.key.second, left.sequence) >
            std::tie(right.key.first, right.key.second, right.sequence);
    }
};

bool key_less(const DStarKey& left, const DStarKey& right) {
    return std::tie(left.first, left.second) < std::tie(right.first, right.second);
}

class DStarTraceBuffer {
public:
    explicit DStarTraceBuffer(uint64_t max_bytes)
        : max_events_(static_cast<size_t>(max_bytes / sizeof(pp_trace_event))) {}

    void record(uint32_t kind, uint64_t node, uint64_t parent, double value) {
        if (events_.size() >= max_events_) {
            truncated_ = true;
            return;
        }
        events_.push_back({node, parent, value, kind, 0});
    }

    const std::vector<pp_trace_event>& events() const { return events_; }
    bool truncated() const { return truncated_; }

private:
    size_t max_events_;
    bool truncated_ = false;
    std::vector<pp_trace_event> events_;
};

void write_error(char* output, size_t capacity, const std::string& message) {
    if (output == nullptr || capacity == 0) {
        return;
    }
    const size_t count = std::min(capacity - 1, message.size());
    std::memcpy(output, message.data(), count);
    output[count] = '\0';
}

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

bool valid_cost(double cost) {
    return !std::isnan(cost) && cost >= 0.0;
}

}  // namespace

class DStarLiteSession {
public:
    DStarLiteSession(
        uint64_t node_count,
        uint64_t edge_count,
        const uint64_t* offsets,
        const uint64_t* neighbor_ids,
        const double* edge_costs,
        uint64_t start_id,
        uint64_t goal_id,
        const double* heuristic_values
    ) : node_count_(node_count),
        start_(start_id),
        goal_(goal_id),
        outgoing_(static_cast<size_t>(node_count)),
        incoming_(static_cast<size_t>(node_count)),
        g_(static_cast<size_t>(node_count), kInfinity),
        rhs_(static_cast<size_t>(node_count), kInfinity),
        next_(static_cast<size_t>(node_count), kNoNode),
        heuristic_values_(static_cast<size_t>(node_count), 0.0),
        versions_(static_cast<size_t>(node_count), 0) {
        if (node_count == 0 || offsets == nullptr || start_id >= node_count || goal_id >= node_count) {
            throw std::invalid_argument("D* Lite graph, start, or goal is invalid");
        }
        if (offsets[0] != 0 || offsets[node_count] != edge_count ||
            (edge_count > 0 && (neighbor_ids == nullptr || edge_costs == nullptr))) {
            throw std::invalid_argument("D* Lite CSR arrays are invalid");
        }
        if (heuristic_values == nullptr) {
            throw std::invalid_argument("D* Lite heuristic array must not be null");
        }
        for (uint64_t node = 0; node < node_count; ++node) {
            if (offsets[node] > offsets[node + 1]) {
                throw std::invalid_argument("D* Lite CSR offsets must be non-decreasing");
            }
            const double heuristic = heuristic_values[node];
            if (!std::isfinite(heuristic) || heuristic < 0.0) {
                throw std::invalid_argument("D* Lite heuristic values must be finite and non-negative");
            }
            heuristic_values_[static_cast<size_t>(node)] = heuristic;
        }
        edges_.reserve(static_cast<size_t>(edge_count));
        edge_by_pair_.reserve(static_cast<size_t>(edge_count));
        for (uint64_t source = 0; source < node_count; ++source) {
            for (uint64_t index = offsets[source]; index < offsets[source + 1]; ++index) {
                const uint64_t target = neighbor_ids[index];
                const double cost = edge_costs[index];
                if (target >= node_count || !valid_cost(cost)) {
                    throw std::invalid_argument("D* Lite edges must have valid endpoints and non-negative costs");
                }
                const EdgeKey key{source, target};
                if (edge_by_pair_.find(key) != edge_by_pair_.end()) {
                    throw std::invalid_argument("D* Lite does not allow duplicate directed edges");
                }
                const size_t edge_id = edges_.size();
                edges_.push_back({source, target, cost, cost});
                edge_by_pair_.emplace(key, edge_id);
                outgoing_[static_cast<size_t>(source)].push_back(edge_id);
                incoming_[static_cast<size_t>(target)].push_back(edge_id);
            }
        }
        rhs_[static_cast<size_t>(goal_)] = 0.0;
        enqueue(goal_);
    }

    int plan(
        bool has_max_expansions,
        uint64_t max_expansions,
        double max_runtime_ms,
        pp_search_result* result,
        DStarTraceBuffer* trace
    ) {
        reset_result(result);
        if (has_max_expansions && max_expansions == 0) {
            throw std::invalid_argument("max_expansions must be positive");
        }
        if (!std::isfinite(max_runtime_ms) || max_runtime_ms < 0.0) {
            throw std::invalid_argument("max_runtime_ms must be finite and non-negative");
        }
        const auto search_started = std::chrono::steady_clock::now();
        if (trace != nullptr) {
            trace->record(PP_TRACE_DISCOVER, start_, kNoNode, 0.0);
        }
        uint64_t expanded = 0;
        bool budget_exhausted = false;
        bool time_exhausted = false;
        while (!consistent_start()) {
            if (has_max_expansions && expanded >= max_expansions) {
                budget_exhausted = true;
                break;
            }
            if (max_runtime_ms > 0.0 && std::chrono::duration<double, std::milli>(
                    std::chrono::steady_clock::now() - search_started).count() >= max_runtime_ms) {
                time_exhausted = true;
                break;
            }
            DStarQueueEntry entry{};
            if (!pop(&entry)) {
                break;
            }
            const DStarKey new_key = key(entry.node);
            if (key_less(entry.key, new_key)) {
                enqueue(entry.node);
                continue;
            }
            const size_t node = static_cast<size_t>(entry.node);
            if (g_[node] > rhs_[node]) {
                g_[node] = rhs_[node];
                if (trace != nullptr) {
                    trace->record(PP_TRACE_EXPAND, entry.node, next_[node], g_[node]);
                }
                for (size_t edge_id : incoming_[node]) {
                    update_vertex(edges_[edge_id].source, trace);
                }
            } else {
                g_[node] = kInfinity;
                if (trace != nullptr) {
                    trace->record(PP_TRACE_EXPAND, entry.node, next_[node], g_[node]);
                }
                update_vertex(entry.node, trace);
                for (size_t edge_id : incoming_[node]) {
                    update_vertex(edges_[edge_id].source, trace);
                }
            }
            ++expanded;
        }

        const bool complete = consistent_start();
        std::vector<uint64_t> path;
        double path_cost = kInfinity;
        if (!extract_path(&path, &path_cost)) {
            result->success = 0;
            result->stop_reason = budget_exhausted ? PP_SEARCH_STOP_MAX_ITERS :
                time_exhausted ? PP_SEARCH_STOP_TIME_BUDGET : PP_SEARCH_STOP_NO_PROGRESS;
            result->iters = expanded;
            result->nodes = discovered_nodes();
            return 0;
        }
        result->success = 1;
        result->stop_reason = complete ? PP_SEARCH_STOP_SUCCESS :
            budget_exhausted ? PP_SEARCH_STOP_MAX_ITERS : PP_SEARCH_STOP_TIME_BUDGET;
        result->iters = expanded;
        result->nodes = discovered_nodes();
        result->path_cost = path_cost;
        result->path_length = path.size();
        if (trace != nullptr) {
            trace->record(PP_TRACE_SOLUTION, goal_, kNoNode, path_cost);
        }
        result->path_ids = static_cast<uint64_t*>(std::malloc(path.size() * sizeof(uint64_t)));
        if (result->path_ids == nullptr) {
            result->path_length = 0;
            throw std::bad_alloc();
        }
        std::memcpy(result->path_ids, path.data(), path.size() * sizeof(uint64_t));
        return 0;
    }

    void move_start(uint64_t start_id, const double* heuristic_values, double heuristic_delta) {
        if (start_id >= node_count_ || heuristic_values == nullptr ||
            !std::isfinite(heuristic_delta) || heuristic_delta < 0.0) {
            throw std::invalid_argument("D* Lite start or heuristic update is invalid");
        }
        validate_heuristics(heuristic_values);
        const double updated_km = km_ + heuristic_delta;
        if (!std::isfinite(updated_km)) {
            throw std::invalid_argument("D* Lite key modifier overflowed");
        }
        km_ = updated_km;
        start_ = start_id;
        std::copy(heuristic_values, heuristic_values + static_cast<size_t>(node_count_),
            heuristic_values_.begin());
    }

    void update_edges(const pp_dstar_edge_update* updates, size_t count) {
        if (count > 0 && updates == nullptr) {
            throw std::invalid_argument("D* Lite edge update array must not be null");
        }
        std::vector<std::pair<size_t, double>> pending;
        pending.reserve(count);
        std::unordered_set<size_t> seen;
        seen.reserve(count);
        for (size_t index = 0; index < count; ++index) {
            const pp_dstar_edge_update& update = updates[index];
            const auto found = edge_by_pair_.find({update.source_id, update.target_id});
            if (found == edge_by_pair_.end()) {
                throw std::invalid_argument("D* Lite edge update does not match a stable graph edge");
            }
            if (update.restore_base_cost != 0 && update.restore_base_cost != 1) {
                throw std::invalid_argument("restore_base_cost must be zero or one");
            }
            if (update.restore_base_cost == 0 && !valid_cost(update.cost)) {
                throw std::invalid_argument("edge costs must be non-negative and not NaN");
            }
            const size_t edge_id = found->second;
            if (!seen.insert(edge_id).second) {
                throw std::invalid_argument("D* Lite edge update batch contains duplicate edges");
            }
            pending.emplace_back(
                edge_id,
                update.restore_base_cost != 0 ? edges_[edge_id].base_cost : update.cost
            );
        }
        std::vector<uint64_t> changed_sources;
        changed_sources.reserve(pending.size());
        for (const auto& update : pending) {
            Edge& edge = edges_[update.first];
            if (edge.cost != update.second) {
                edge.cost = update.second;
                changed_sources.push_back(edge.source);
            }
        }
        std::sort(changed_sources.begin(), changed_sources.end());
        changed_sources.erase(
            std::unique(changed_sources.begin(), changed_sources.end()),
            changed_sources.end()
        );
        for (uint64_t source : changed_sources) {
            update_vertex(source, nullptr);
        }
    }

    void reset(uint64_t start_id, uint64_t goal_id, const double* heuristic_values) {
        if (start_id >= node_count_ || goal_id >= node_count_ || heuristic_values == nullptr) {
            throw std::invalid_argument("D* Lite reset start, goal, or heuristic is invalid");
        }
        validate_heuristics(heuristic_values);
        start_ = start_id;
        goal_ = goal_id;
        km_ = 0.0;
        std::fill(g_.begin(), g_.end(), kInfinity);
        std::fill(rhs_.begin(), rhs_.end(), kInfinity);
        std::fill(next_.begin(), next_.end(), kNoNode);
        std::fill(versions_.begin(), versions_.end(), 0);
        std::copy(heuristic_values, heuristic_values + static_cast<size_t>(node_count_),
            heuristic_values_.begin());
        queue_ = decltype(queue_){};
        rhs_[static_cast<size_t>(goal_)] = 0.0;
        enqueue(goal_);
    }

private:
    struct Edge {
        uint64_t source;
        uint64_t target;
        double base_cost;
        double cost;
    };

    DStarKey key(uint64_t node) const {
        const size_t index = static_cast<size_t>(node);
        const double value = std::min(g_[index], rhs_[index]);
        return {value + heuristic_values_[index] + km_, value};
    }

    void enqueue(uint64_t node) {
        const uint64_t version = ++versions_[static_cast<size_t>(node)];
        queue_.push({key(node), sequence_++, node, version});
    }

    void invalidate(uint64_t node) {
        ++versions_[static_cast<size_t>(node)];
    }

    bool top_key(DStarKey* output) {
        while (!queue_.empty() &&
            queue_.top().version != versions_[static_cast<size_t>(queue_.top().node)]) {
            queue_.pop();
        }
        if (queue_.empty()) {
            *output = {kInfinity, kInfinity};
            return false;
        }
        *output = queue_.top().key;
        return true;
    }

    bool pop(DStarQueueEntry* output) {
        while (!queue_.empty()) {
            DStarQueueEntry entry = queue_.top();
            queue_.pop();
            if (entry.version == versions_[static_cast<size_t>(entry.node)]) {
                invalidate(entry.node);
                *output = entry;
                return true;
            }
        }
        return false;
    }

    bool consistent_start() {
        DStarKey top{};
        const bool has_top = top_key(&top);
        return (!has_top || !key_less(top, key(start_))) &&
            rhs_[static_cast<size_t>(start_)] == g_[static_cast<size_t>(start_)];
    }

    void update_vertex(uint64_t node, DStarTraceBuffer* trace) {
        const size_t index = static_cast<size_t>(node);
        if (node != goal_) {
            double best = kInfinity;
            uint64_t best_successor = kNoNode;
            for (size_t edge_id : outgoing_[index]) {
                const Edge& edge = edges_[edge_id];
                const double candidate = edge.cost + g_[static_cast<size_t>(edge.target)];
                if (candidate < best) {
                    best = candidate;
                    best_successor = edge.target;
                }
            }
            rhs_[index] = best;
            next_[index] = best_successor;
        }
        invalidate(node);
        if (g_[index] != rhs_[index]) {
            enqueue(node);
        }
        if (trace != nullptr && best_finite(rhs_[index])) {
            trace->record(PP_TRACE_PARENT, node, next_[index], rhs_[index]);
        }
    }

    static bool best_finite(double value) { return std::isfinite(value); }

    void validate_heuristics(const double* values) const {
        for (uint64_t node = 0; node < node_count_; ++node) {
            const double value = values[node];
            if (!std::isfinite(value) || value < 0.0) {
                throw std::invalid_argument("D* Lite heuristic values must be finite and non-negative");
            }
        }
    }

    uint64_t discovered_nodes() const {
        uint64_t count = 0;
        for (size_t node = 0; node < g_.size(); ++node) {
            if (std::isfinite(g_[node]) || std::isfinite(rhs_[node])) {
                ++count;
            }
        }
        return count;
    }

    bool extract_path(std::vector<uint64_t>* path, double* path_cost) const {
        path->clear();
        path->push_back(start_);
        std::unordered_set<uint64_t> visited{start_};
        double total = 0.0;
        uint64_t current = start_;
        while (current != goal_) {
            const size_t index = static_cast<size_t>(current);
            double best = kInfinity;
            uint64_t successor = kNoNode;
            double selected_cost = kInfinity;
            for (size_t edge_id : outgoing_[index]) {
                const Edge& edge = edges_[edge_id];
                if (!std::isfinite(edge.cost) || visited.find(edge.target) != visited.end()) {
                    continue;
                }
                const double value = edge.cost + g_[static_cast<size_t>(edge.target)];
                if (value < best) {
                    best = value;
                    successor = edge.target;
                    selected_cost = edge.cost;
                }
            }
            if (successor == kNoNode || !std::isfinite(best)) {
                path->clear();
                return false;
            }
            total += selected_cost;
            current = successor;
            visited.insert(current);
            path->push_back(current);
            if (path->size() > static_cast<size_t>(node_count_)) {
                path->clear();
                return false;
            }
        }
        *path_cost = total;
        return true;
    }

    uint64_t node_count_;
    uint64_t start_;
    uint64_t goal_;
    double km_ = 0.0;
    uint64_t sequence_ = 0;
    std::vector<Edge> edges_;
    std::vector<std::vector<size_t>> outgoing_;
    std::vector<std::vector<size_t>> incoming_;
    std::unordered_map<EdgeKey, size_t, EdgeKeyHash> edge_by_pair_;
    std::vector<double> g_;
    std::vector<double> rhs_;
    std::vector<uint64_t> next_;
    std::vector<double> heuristic_values_;
    std::vector<uint64_t> versions_;
    std::priority_queue<DStarQueueEntry, std::vector<DStarQueueEntry>, DStarQueueGreater> queue_;
};

struct pp_native_dstar_lite {
    std::unique_ptr<DStarLiteSession> session;
};

namespace {

int copy_trace(const DStarTraceBuffer& captured, pp_trace_result* trace, pp_search_result* result) {
    const auto& events = captured.events();
    if (!events.empty()) {
        trace->events = static_cast<pp_trace_event*>(
            std::malloc(events.size() * sizeof(pp_trace_event))
        );
        if (trace->events == nullptr) {
            pp_search_free_result(result);
            return 1;
        }
        std::memcpy(trace->events, events.data(), events.size() * sizeof(pp_trace_event));
        trace->event_count = events.size();
    }
    trace->truncated = captured.truncated() ? 1 : 0;
    return 0;
}

}  // namespace

extern "C" int pp_dstar_lite_create(
    uint64_t node_count,
    uint64_t edge_count,
    const uint64_t* offsets,
    const uint64_t* neighbor_ids,
    const double* edge_costs,
    uint64_t start_id,
    uint64_t goal_id,
    const double* heuristic_values,
    pp_native_dstar_lite** out_planner,
    char* error_message,
    size_t error_capacity
) {
    if (out_planner == nullptr) {
        write_error(error_message, error_capacity, "out_planner must not be null");
        return 1;
    }
    *out_planner = nullptr;
    try {
        auto planner = std::make_unique<pp_native_dstar_lite>();
        planner->session = std::make_unique<DStarLiteSession>(
            node_count,
            edge_count,
            offsets,
            neighbor_ids,
            edge_costs,
            start_id,
            goal_id,
            heuristic_values
        );
        *out_planner = planner.release();
        write_error(error_message, error_capacity, "");
        return 0;
    } catch (const std::exception& error) {
        write_error(error_message, error_capacity, error.what());
        return 1;
    } catch (...) {
        write_error(error_message, error_capacity, "unknown D* Lite initialization error");
        return 1;
    }
}

extern "C" void pp_dstar_lite_free(pp_native_dstar_lite* planner) {
    delete planner;
}

extern "C" int pp_dstar_lite_plan(
    pp_native_dstar_lite* planner,
    int has_max_expansions,
    uint64_t max_expansions,
    double max_runtime_ms,
    pp_search_result* result
) {
    if (planner == nullptr || result == nullptr) {
        return 1;
    }
    try {
        return planner->session->plan(
            has_max_expansions != 0,
            max_expansions,
            max_runtime_ms,
            result,
            nullptr
        );
    } catch (const std::exception& error) {
        set_error(result, error.what());
        return 1;
    } catch (...) {
        set_error(result, "unknown D* Lite search error");
        return 1;
    }
}

extern "C" int pp_dstar_lite_plan_traced(
    pp_native_dstar_lite* planner,
    int has_max_expansions,
    uint64_t max_expansions,
    double max_runtime_ms,
    uint64_t trace_max_bytes,
    pp_search_result* result,
    pp_trace_result* trace
) {
    if (planner == nullptr || result == nullptr || trace == nullptr) {
        return 1;
    }
    *trace = pp_trace_result{};
    DStarTraceBuffer captured(trace_max_bytes);
    int status = 0;
    try {
        status = planner->session->plan(
            has_max_expansions != 0,
            max_expansions,
            max_runtime_ms,
            result,
            &captured
        );
    } catch (const std::exception& error) {
        set_error(result, error.what());
        status = 1;
    } catch (...) {
        set_error(result, "unknown D* Lite search error");
        status = 1;
    }
    if (copy_trace(captured, trace, result) != 0) {
        pp_dstar_lite_trace_free_result(trace);
        set_error(result, "could not allocate D* Lite trace events");
        return 1;
    }
    return status;
}

extern "C" int pp_dstar_lite_move_start(
    pp_native_dstar_lite* planner,
    uint64_t start_id,
    const double* heuristic_values,
    double heuristic_delta,
    char* error_message,
    size_t error_capacity
) {
    if (planner == nullptr) {
        write_error(error_message, error_capacity, "planner must not be null");
        return 1;
    }
    try {
        planner->session->move_start(start_id, heuristic_values, heuristic_delta);
        write_error(error_message, error_capacity, "");
        return 0;
    } catch (const std::exception& error) {
        write_error(error_message, error_capacity, error.what());
        return 1;
    } catch (...) {
        write_error(error_message, error_capacity, "unknown D* Lite start update error");
        return 1;
    }
}

extern "C" int pp_dstar_lite_update_edges(
    pp_native_dstar_lite* planner,
    const pp_dstar_edge_update* updates,
    size_t update_count,
    char* error_message,
    size_t error_capacity
) {
    if (planner == nullptr) {
        write_error(error_message, error_capacity, "planner must not be null");
        return 1;
    }
    try {
        planner->session->update_edges(updates, update_count);
        write_error(error_message, error_capacity, "");
        return 0;
    } catch (const std::exception& error) {
        write_error(error_message, error_capacity, error.what());
        return 1;
    } catch (...) {
        write_error(error_message, error_capacity, "unknown D* Lite edge update error");
        return 1;
    }
}

extern "C" int pp_dstar_lite_reset(
    pp_native_dstar_lite* planner,
    uint64_t start_id,
    uint64_t goal_id,
    const double* heuristic_values,
    char* error_message,
    size_t error_capacity
) {
    if (planner == nullptr) {
        write_error(error_message, error_capacity, "planner must not be null");
        return 1;
    }
    try {
        planner->session->reset(start_id, goal_id, heuristic_values);
        write_error(error_message, error_capacity, "");
        return 0;
    } catch (const std::exception& error) {
        write_error(error_message, error_capacity, error.what());
        return 1;
    } catch (...) {
        write_error(error_message, error_capacity, "unknown D* Lite reset error");
        return 1;
    }
}

extern "C" void pp_dstar_lite_trace_free_result(pp_trace_result* trace) {
    if (trace == nullptr) {
        return;
    }
    std::free(trace->events);
    std::free(trace->points);
    *trace = pp_trace_result{};
}
