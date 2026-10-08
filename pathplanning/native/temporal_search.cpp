#include "search_engine.h"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdlib>
#include <cstring>
#include <functional>
#include <limits>
#include <map>
#include <new>
#include <optional>
#include <queue>
#include <set>
#include <stdexcept>
#include <string>
#include <tuple>
#include <vector>

namespace {

constexpr double kInfinity = std::numeric_limits<double>::infinity();
constexpr uint64_t kNoState = std::numeric_limits<uint64_t>::max();

struct TimeInterval {
    double start;
    double end;
};

struct QueueEntry {
    double arrival;
    uint64_t sequence;
    uint64_t state;
};

struct QueueEntryGreater {
    bool operator()(const QueueEntry& left, const QueueEntry& right) const {
        return std::tie(left.arrival, left.sequence) > std::tie(right.arrival, right.sequence);
    }
};

struct OpenFEntry {
    double f;
    uint64_t sequence;
    uint64_t state;
    uint64_t version;
};

struct OpenFEntryLess {
    bool operator()(const OpenFEntry& left, const OpenFEntry& right) const {
        return std::tie(left.f, left.sequence) < std::tie(right.f, right.sequence);
    }
};

struct FocalEntry {
    uint64_t hops;
    double f;
    uint64_t sequence;
    uint64_t state;
    uint64_t version;
};

struct FocalEntryGreater {
    bool operator()(const FocalEntry& left, const FocalEntry& right) const {
        return std::tie(left.hops, left.f, left.sequence) >
            std::tie(right.hops, right.f, right.sequence);
    }
};

class TraceCapture {
public:
    explicit TraceCapture(uint64_t max_bytes)
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

void reset_result(pp_sipp_result* result) {
    *result = pp_sipp_result{};
    result->stop_reason = PP_SEARCH_STOP_ERROR;
    result->path_cost = kInfinity;
    result->arrival_time = kInfinity;
}

void set_error(pp_sipp_result* result, const std::string& message) {
    reset_result(result);
    result->error_message = static_cast<char*>(std::malloc(message.size() + 1));
    if (result->error_message != nullptr) {
        std::memcpy(result->error_message, message.c_str(), message.size() + 1);
    }
}

bool valid_interval(double start, double end) {
    return !std::isnan(start) && !std::isnan(end) && start < end;
}

class SippSearch {
public:
    SippSearch(
        uint64_t node_count,
        uint64_t edge_count,
        const uint64_t* offsets,
        const uint64_t* neighbor_ids,
        const double* edge_durations,
        const uint64_t* safe_offsets,
        const double* safe_starts,
        const double* safe_ends,
        uint64_t safe_count,
        const uint64_t* edge_block_offsets,
        const double* edge_block_starts,
        const double* edge_block_ends,
        uint64_t edge_block_count,
        uint64_t start_id,
        uint64_t goal_id,
        double start_time
    ) : node_count_(node_count),
        edge_count_(edge_count),
        offsets_(offsets),
        neighbor_ids_(neighbor_ids),
        edge_durations_(edge_durations),
        safe_offsets_(safe_offsets),
        safe_starts_(safe_starts),
        safe_ends_(safe_ends),
        safe_count_(safe_count),
        edge_block_offsets_(edge_block_offsets),
        edge_block_starts_(edge_block_starts),
        edge_block_ends_(edge_block_ends),
        edge_block_count_(edge_block_count),
        start_(start_id),
        goal_(goal_id),
        start_time_(start_time) {
        validate();
    }

    int plan(
        int has_max_expansions,
        uint64_t max_expansions,
        double max_runtime_ms,
        double focal_weight,
        pp_sipp_result* result,
        TraceCapture* trace
    ) const {
        reset_result(result);
        if ((has_max_expansions != 0 && max_expansions == 0) ||
            (has_max_expansions != 0 && has_max_expansions != 1)) {
            throw std::invalid_argument("max_expansions must be positive when enabled");
        }
        if (!std::isfinite(max_runtime_ms) || max_runtime_ms < 0.0) {
            throw std::invalid_argument("max_runtime_ms must be finite and non-negative");
        }
        if (!std::isfinite(focal_weight) || focal_weight < 0.0 ||
            (focal_weight > 0.0 && focal_weight < 1.0)) {
            throw std::invalid_argument("suboptimality weight must be zero or finite and at least 1");
        }
        const bool use_focal = focal_weight > 0.0;

        const size_t state_count = static_cast<size_t>(safe_count_);
        std::vector<uint64_t> state_nodes(state_count);
        std::vector<double> arrivals(state_count, kInfinity);
        std::vector<uint64_t> parents(state_count, kNoState);
        std::priority_queue<QueueEntry, std::vector<QueueEntry>, QueueEntryGreater> open;
        std::set<OpenFEntry, OpenFEntryLess> open_by_f;
        std::priority_queue<FocalEntry, std::vector<FocalEntry>, FocalEntryGreater> focal;
        std::vector<std::optional<OpenFEntry>> current_open;
        std::vector<uint64_t> state_versions;
        std::vector<uint64_t> focal_versions;
        std::vector<double> heuristic;
        std::vector<uint64_t> hop_heuristic;
        uint64_t sequence = 0;
        for (uint64_t node = 0; node < node_count_; ++node) {
            for (uint64_t state = safe_offsets_[node]; state < safe_offsets_[node + 1]; ++state) {
                state_nodes[static_cast<size_t>(state)] = node;
            }
        }

        uint64_t start_state = kNoState;
        for (uint64_t state = safe_offsets_[start_]; state < safe_offsets_[start_ + 1]; ++state) {
            if (safe_starts_[state] <= start_time_ && start_time_ < safe_ends_[state]) {
                start_state = state;
                break;
            }
        }
        if (start_state == kNoState) {
            result->stop_reason = PP_SEARCH_STOP_NO_PROGRESS;
            return 0;
        }

        if (use_focal) {
            compute_static_heuristics(&heuristic, &hop_heuristic);
            current_open.resize(state_count);
            state_versions.assign(state_count, 0);
            focal_versions.assign(state_count, kNoState);
        }

        arrivals[static_cast<size_t>(start_state)] = start_time_;
        if (use_focal) {
            const uint64_t version = ++state_versions[static_cast<size_t>(start_state)];
            const OpenFEntry entry{
                heuristic[static_cast<size_t>(start_)] + (start_time_ - start_time_),
                sequence++, start_state, version
            };
            current_open[static_cast<size_t>(start_state)] = entry;
            open_by_f.insert(entry);
        } else {
            open.push({start_time_, sequence++, start_state});
        }
        if (trace != nullptr) {
            trace->record(PP_TRACE_DISCOVER, start_, kNoState, start_time_);
        }
        uint64_t expanded = 0;
        uint64_t discovered = 1;
        uint64_t best_goal = start_ == goal_ ? start_state : kNoState;
        bool complete = best_goal != kNoState;
        bool expansion_limited = false;
        bool time_limited = false;
        double focal_bound = -kInfinity;
        const auto search_started = std::chrono::steady_clock::now();

        while (!complete) {
            if ((use_focal && open_by_f.empty()) || (!use_focal && open.empty())) {
                break;
            }
            if (has_max_expansions != 0 && expanded >= max_expansions) {
                expansion_limited = true;
                break;
            }
            if (max_runtime_ms > 0.0 && std::chrono::duration<double, std::milli>(
                    std::chrono::steady_clock::now() - search_started).count() >= max_runtime_ms) {
                time_limited = true;
                break;
            }

            uint64_t current_state = kNoState;
            if (use_focal) {
                const double minimum_f = open_by_f.begin()->f;
                const double new_bound = focal_weight * minimum_f;
                if (new_bound > focal_bound) {
                    const OpenFEntry lower_bound{focal_bound, kNoState, 0, 0};
                    for (auto iterator = open_by_f.upper_bound(lower_bound);
                        iterator != open_by_f.end() && iterator->f <= new_bound;
                        ++iterator) {
                        const size_t state = static_cast<size_t>(iterator->state);
                        if (focal_versions[state] != iterator->version) {
                            focal.push({
                                hop_heuristic[static_cast<size_t>(state_nodes[state])],
                                iterator->f, sequence++, iterator->state, iterator->version
                            });
                            focal_versions[state] = iterator->version;
                        }
                    }
                    focal_bound = new_bound;
                }
                while (!focal.empty()) {
                    const FocalEntry& candidate = focal.top();
                    const size_t state = static_cast<size_t>(candidate.state);
                    if (current_open[state].has_value() &&
                        current_open[state]->version == candidate.version) {
                        break;
                    }
                    if (focal_versions[state] == candidate.version) {
                        focal_versions[state] = kNoState;
                    }
                    focal.pop();
                }
                if (focal.empty()) {
                    throw std::logic_error("FocalSIPP OPEN/FOCAL invariant was violated");
                }
                const FocalEntry entry = focal.top();
                focal.pop();
                current_state = entry.state;
                focal_versions[static_cast<size_t>(current_state)] = kNoState;
                open_by_f.erase(*current_open[static_cast<size_t>(current_state)]);
                current_open[static_cast<size_t>(current_state)].reset();
            } else {
                const QueueEntry entry = open.top();
                open.pop();
                if (entry.arrival != arrivals[static_cast<size_t>(entry.state)]) {
                    continue;
                }
                current_state = entry.state;
            }

            const double current_arrival = arrivals[static_cast<size_t>(current_state)];
            const uint64_t node = state_nodes[static_cast<size_t>(current_state)];
            if (node == goal_) {
                best_goal = current_state;
                complete = true;
                break;
            }
            ++expanded;
            if (trace != nullptr) {
                const uint64_t parent = parents[static_cast<size_t>(current_state)];
                trace->record(
                    PP_TRACE_EXPAND,
                    node,
                    parent == kNoState ? kNoState : state_nodes[static_cast<size_t>(parent)],
                    current_arrival
                );
            }

            const double source_interval_end = safe_ends_[current_state];
            for (uint64_t edge = offsets_[node]; edge < offsets_[node + 1]; ++edge) {
                const uint64_t target = neighbor_ids_[edge];
                const double duration = edge_durations_[edge];
                const uint64_t first_target_state = safe_offsets_[target];
                const uint64_t last_target_state = safe_offsets_[target + 1];
                for (uint64_t target_state = first_target_state;
                    target_state < last_target_state;
                    ++target_state) {
                    double departure = std::max(
                        current_arrival,
                        safe_starts_[target_state] - duration
                    );
                    bool feasible = true;
                    const uint64_t first_block = edge_block_offsets_[edge];
                    const uint64_t last_block = edge_block_offsets_[edge + 1];
                    double arrival = kInfinity;
                    while (true) {
                        if (!std::isfinite(departure) || departure >= source_interval_end) {
                            feasible = false;
                            break;
                        }
                        arrival = departure + duration;
                        if (!std::isfinite(arrival) || arrival >= safe_ends_[target_state]) {
                            feasible = false;
                            break;
                        }
                        if (arrival < safe_starts_[target_state]) {
                            departure += safe_starts_[target_state] - arrival;
                            continue;
                        }
                        bool shifted = false;
                        for (uint64_t block = first_block; block < last_block; ++block) {
                            if (departure < edge_block_ends_[block] &&
                                arrival > edge_block_starts_[block]) {
                                departure = edge_block_ends_[block];
                                shifted = true;
                                break;
                            }
                        }
                        if (!shifted) {
                            break;
                        }
                    }
                    if (!feasible) {
                        continue;
                    }
                    const size_t state_index = static_cast<size_t>(target_state);
                    if (arrival >= arrivals[state_index]) {
                        continue;
                    }
                    if (!std::isfinite(arrivals[state_index])) {
                        ++discovered;
                    }
                    arrivals[state_index] = arrival;
                    parents[state_index] = current_state;
                    if (use_focal) {
                        if (current_open[state_index].has_value()) {
                            open_by_f.erase(*current_open[state_index]);
                        }
                        const uint64_t version = ++state_versions[state_index];
                        const double f = (arrival - start_time_) +
                            heuristic[static_cast<size_t>(target)];
                        const OpenFEntry entry{f, sequence++, target_state, version};
                        current_open[state_index] = entry;
                        open_by_f.insert(entry);
                        if (f <= focal_bound && focal_versions[state_index] != version) {
                            focal.push({
                                hop_heuristic[static_cast<size_t>(target)],
                                f, sequence++, target_state, version
                            });
                            focal_versions[state_index] = version;
                        }
                    } else {
                        open.push({arrival, sequence++, target_state});
                    }
                    if (trace != nullptr) {
                        trace->record(PP_TRACE_DISCOVER, target, node, arrival);
                        trace->record(PP_TRACE_PARENT, target, node, arrival);
                    }
                    if (target == goal_ && (best_goal == kNoState ||
                        arrival < arrivals[static_cast<size_t>(best_goal)])) {
                        best_goal = target_state;
                    }
                }
            }
        }

        result->iters = expanded;
        result->nodes = discovered;
        if (best_goal == kNoState) {
            result->stop_reason = expansion_limited ? PP_SEARCH_STOP_MAX_ITERS :
                time_limited ? PP_SEARCH_STOP_TIME_BUDGET : PP_SEARCH_STOP_NO_PROGRESS;
            return 0;
        }

        std::vector<uint64_t> reverse_states;
        for (uint64_t state = best_goal; state != kNoState; state = parents[static_cast<size_t>(state)]) {
            reverse_states.push_back(state);
            if (reverse_states.size() > state_count) {
                throw std::runtime_error("SIPP parent chain contains a cycle");
            }
        }
        std::reverse(reverse_states.begin(), reverse_states.end());
        const size_t path_length = reverse_states.size();
        if (trace != nullptr) {
            trace->record(PP_TRACE_SOLUTION, goal_, kNoState, arrivals[static_cast<size_t>(best_goal)]);
        }
        result->path_ids = static_cast<uint64_t*>(std::malloc(path_length * sizeof(uint64_t)));
        result->arrival_times = static_cast<double*>(std::malloc(path_length * sizeof(double)));
        if (result->path_ids == nullptr || result->arrival_times == nullptr) {
            std::free(result->path_ids);
            std::free(result->arrival_times);
            result->path_ids = nullptr;
            result->arrival_times = nullptr;
            throw std::bad_alloc();
        }
        for (size_t index = 0; index < path_length; ++index) {
            const uint64_t state = reverse_states[index];
            result->path_ids[index] = state_nodes[static_cast<size_t>(state)];
            result->arrival_times[index] = arrivals[static_cast<size_t>(state)];
        }
        result->path_length = path_length;
        result->arrival_time = arrivals[static_cast<size_t>(best_goal)];
        result->path_cost = result->arrival_time - start_time_;
        result->success = 1;
        result->stop_reason = complete ? PP_SEARCH_STOP_SUCCESS : expansion_limited ?
            PP_SEARCH_STOP_MAX_ITERS : PP_SEARCH_STOP_TIME_BUDGET;
        return 0;
    }

private:
    void compute_static_heuristics(
        std::vector<double>* travel_time,
        std::vector<uint64_t>* hop_count
    ) const {
        std::vector<std::vector<std::pair<uint64_t, double>>> reverse_edges(
            static_cast<size_t>(node_count_)
        );
        for (uint64_t source = 0; source < node_count_; ++source) {
            for (uint64_t edge = offsets_[source]; edge < offsets_[source + 1]; ++edge) {
                reverse_edges[static_cast<size_t>(neighbor_ids_[edge])].push_back(
                    {source, edge_durations_[edge]}
                );
            }
        }

        travel_time->assign(static_cast<size_t>(node_count_), kInfinity);
        hop_count->assign(static_cast<size_t>(node_count_), kNoState);
        using DistanceEntry = std::pair<double, uint64_t>;
        std::priority_queue<
            DistanceEntry, std::vector<DistanceEntry>, std::greater<DistanceEntry>
        > distance_open;
        (*travel_time)[static_cast<size_t>(goal_)] = 0.0;
        distance_open.push({0.0, goal_});
        while (!distance_open.empty()) {
            const auto [distance, node] = distance_open.top();
            distance_open.pop();
            if (distance != (*travel_time)[static_cast<size_t>(node)]) {
                continue;
            }
            for (const auto& [predecessor, duration] : reverse_edges[static_cast<size_t>(node)]) {
                const double candidate = distance + duration;
                const size_t predecessor_index = static_cast<size_t>(predecessor);
                if (candidate < (*travel_time)[predecessor_index]) {
                    (*travel_time)[predecessor_index] = candidate;
                    distance_open.push({candidate, predecessor});
                }
            }
        }

        std::queue<uint64_t> hop_open;
        (*hop_count)[static_cast<size_t>(goal_)] = 0;
        hop_open.push(goal_);
        while (!hop_open.empty()) {
            const uint64_t node = hop_open.front();
            hop_open.pop();
            for (const auto& [predecessor, unused_duration] : reverse_edges[static_cast<size_t>(node)]) {
                static_cast<void>(unused_duration);
                const size_t predecessor_index = static_cast<size_t>(predecessor);
                if ((*hop_count)[predecessor_index] == kNoState) {
                    (*hop_count)[predecessor_index] =
                        (*hop_count)[static_cast<size_t>(node)] + 1;
                    hop_open.push(predecessor);
                }
            }
        }
    }

    void validate() const {
        if (node_count_ == 0 || offsets_ == nullptr || safe_offsets_ == nullptr ||
            edge_block_offsets_ == nullptr || start_ >= node_count_ || goal_ >= node_count_ ||
            !std::isfinite(start_time_)) {
            throw std::invalid_argument("SIPP graph, interval arrays, endpoints, or start time are invalid");
        }
        if (edge_count_ > 0 && (neighbor_ids_ == nullptr || edge_durations_ == nullptr)) {
            throw std::invalid_argument("SIPP graph edge arrays must not be null");
        }
        if (safe_count_ > 0 && (safe_starts_ == nullptr || safe_ends_ == nullptr)) {
            throw std::invalid_argument("SIPP safe interval arrays must not be null");
        }
        if (edge_block_count_ > 0 && (edge_block_starts_ == nullptr || edge_block_ends_ == nullptr)) {
            throw std::invalid_argument("SIPP edge block arrays must not be null");
        }
        if (offsets_[0] != 0 || offsets_[node_count_] != edge_count_ ||
            safe_offsets_[0] != 0 || safe_offsets_[node_count_] != safe_count_ ||
            edge_block_offsets_[0] != 0 || edge_block_offsets_[edge_count_] != edge_block_count_) {
            throw std::invalid_argument("SIPP CSR or interval offsets do not match their array lengths");
        }
        for (uint64_t node = 0; node < node_count_; ++node) {
            if (offsets_[node] > offsets_[node + 1] ||
                safe_offsets_[node] > safe_offsets_[node + 1]) {
                throw std::invalid_argument("SIPP offsets must be non-decreasing");
            }
            double previous_end = -kInfinity;
            for (uint64_t interval = safe_offsets_[node]; interval < safe_offsets_[node + 1]; ++interval) {
                const double start = safe_starts_[interval];
                const double end = safe_ends_[interval];
                if (!valid_interval(start, end) || start < previous_end) {
                    throw std::invalid_argument("SIPP safe intervals must be sorted and disjoint");
                }
                previous_end = end;
            }
        }
        for (uint64_t edge = 0; edge < edge_count_; ++edge) {
            if (neighbor_ids_[edge] >= node_count_ ||
                !std::isfinite(edge_durations_[edge]) || edge_durations_[edge] <= 0.0 ||
                edge_block_offsets_[edge] > edge_block_offsets_[edge + 1]) {
                throw std::invalid_argument("SIPP edges require valid endpoints and positive durations");
            }
            double previous_end = -kInfinity;
            for (uint64_t interval = edge_block_offsets_[edge];
                interval < edge_block_offsets_[edge + 1];
                ++interval) {
                const double start = edge_block_starts_[interval];
                const double end = edge_block_ends_[interval];
                if (!valid_interval(start, end) || start < previous_end) {
                    throw std::invalid_argument("SIPP edge blocks must be sorted and disjoint");
                }
                previous_end = end;
            }
        }
    }

    uint64_t node_count_;
    uint64_t edge_count_;
    const uint64_t* offsets_;
    const uint64_t* neighbor_ids_;
    const double* edge_durations_;
    const uint64_t* safe_offsets_;
    const double* safe_starts_;
    const double* safe_ends_;
    uint64_t safe_count_;
    const uint64_t* edge_block_offsets_;
    const double* edge_block_starts_;
    const double* edge_block_ends_;
    uint64_t edge_block_count_;
    uint64_t start_;
    uint64_t goal_;
    double start_time_;
};

struct KinodynamicState {
    uint64_t node;
    uint64_t safe_interval;
    double time_low;
    double time_high;
    double reachable_high;
    uint64_t parent;
    double parent_departure;
    bool closed;
};

struct KinodynamicQueueEntry {
    double f;
    uint64_t sequence;
    uint64_t state;
};

struct KinodynamicQueueEntryGreater {
    bool operator()(const KinodynamicQueueEntry& left, const KinodynamicQueueEntry& right) const {
        return std::tie(left.f, left.sequence) > std::tie(right.f, right.sequence);
    }
};

class KinodynamicSippSearch {
public:
    KinodynamicSippSearch(
        uint64_t node_count,
        uint64_t edge_count,
        const uint64_t* offsets,
        const uint64_t* neighbor_ids,
        const uint8_t* waitable_nodes,
        const double* edge_durations,
        const uint64_t* safe_offsets,
        const double* safe_starts,
        const double* safe_ends,
        uint64_t safe_count,
        const uint64_t* edge_block_offsets,
        const double* edge_block_starts,
        const double* edge_block_ends,
        uint64_t edge_block_count,
        uint64_t start_id,
        uint64_t goal_id,
        double start_time
    ) : node_count_(node_count),
        edge_count_(edge_count),
        offsets_(offsets),
        neighbor_ids_(neighbor_ids),
        waitable_nodes_(waitable_nodes),
        edge_durations_(edge_durations),
        safe_offsets_(safe_offsets),
        safe_starts_(safe_starts),
        safe_ends_(safe_ends),
        safe_count_(safe_count),
        edge_block_offsets_(edge_block_offsets),
        edge_block_starts_(edge_block_starts),
        edge_block_ends_(edge_block_ends),
        edge_block_count_(edge_block_count),
        start_(start_id),
        goal_(goal_id),
        start_time_(start_time) {
        validate();
    }

    int plan(
        int has_max_expansions,
        uint64_t max_expansions,
        double max_runtime_ms,
        pp_sipp_result* result,
        TraceCapture* trace
    ) const {
        reset_result(result);
        if ((has_max_expansions != 0 && max_expansions == 0) ||
            (has_max_expansions != 0 && has_max_expansions != 1)) {
            throw std::invalid_argument("max_expansions must be positive when enabled");
        }
        if (!std::isfinite(max_runtime_ms) || max_runtime_ms < 0.0) {
            throw std::invalid_argument("max_runtime_ms must be finite and non-negative");
        }

        std::vector<double> heuristic(static_cast<size_t>(node_count_), kInfinity);
        compute_heuristic(&heuristic);
        std::vector<KinodynamicState> states;
        std::map<std::tuple<uint64_t, uint64_t, double, double>, uint64_t> state_ids;
        std::priority_queue<
            KinodynamicQueueEntry,
            std::vector<KinodynamicQueueEntry>,
            KinodynamicQueueEntryGreater
        > open;
        uint64_t sequence = 0;

        uint64_t start_safe_interval = kNoState;
        for (uint64_t safe_id = safe_offsets_[start_];
            safe_id < safe_offsets_[start_ + 1];
            ++safe_id) {
            if (safe_starts_[safe_id] <= start_time_ && start_time_ <= safe_ends_[safe_id]) {
                start_safe_interval = safe_id;
                break;
            }
        }
        if (start_safe_interval == kNoState) {
            result->stop_reason = PP_SEARCH_STOP_NO_PROGRESS;
            return 0;
        }

        const double start_high = waitable_nodes_[start_] != 0 ?
            safe_ends_[start_safe_interval] : start_time_;
        const uint64_t start_state = add_state(
            start_, start_safe_interval, start_time_, start_high, start_time_,
            kNoState, start_time_, &states, &state_ids, &open, &sequence, trace
        );
        uint64_t best_goal = start_ == goal_ ? start_state : kNoState;
        bool complete = best_goal != kNoState;
        bool expansion_limited = false;
        bool time_limited = false;
        uint64_t expanded = 0;
        const auto search_started = std::chrono::steady_clock::now();

        while (!complete && !open.empty()) {
            if (has_max_expansions != 0 && expanded >= max_expansions) {
                expansion_limited = true;
                break;
            }
            if (max_runtime_ms > 0.0 && std::chrono::duration<double, std::milli>(
                    std::chrono::steady_clock::now() - search_started).count() >= max_runtime_ms) {
                time_limited = true;
                break;
            }
            const KinodynamicQueueEntry entry = open.top();
            open.pop();
            KinodynamicState& stored_state = states[static_cast<size_t>(entry.state)];
            if (stored_state.closed) {
                continue;
            }
            stored_state.closed = true;
            const KinodynamicState source = stored_state;
            if (source.node == goal_) {
                best_goal = entry.state;
                complete = true;
                break;
            }
            ++expanded;
            if (trace != nullptr) {
                const uint64_t parent_node = source.parent == kNoState ? kNoState :
                    states[static_cast<size_t>(source.parent)].node;
                trace->record(PP_TRACE_EXPAND, source.node, parent_node, source.time_low);
            }

            for (uint64_t edge = offsets_[source.node]; edge < offsets_[source.node + 1]; ++edge) {
                const uint64_t target = neighbor_ids_[edge];
                const double duration = edge_durations_[edge];
                const auto departures = valid_departure_intervals(
                    source.time_low,
                    source.time_high,
                    edge_block_offsets_[edge],
                    edge_block_offsets_[edge + 1],
                    duration
                );
                for (const auto& departure_interval : departures) {
                    const double arrival_low = departure_interval.first + duration;
                    const double arrival_high = departure_interval.second + duration;
                    for (uint64_t target_safe = safe_offsets_[target];
                        target_safe < safe_offsets_[target + 1];
                        ++target_safe) {
                        const double raw_low = std::max(arrival_low, safe_starts_[target_safe]);
                        const double raw_high = std::min(arrival_high, safe_ends_[target_safe]);
                        if (raw_low > raw_high) {
                            continue;
                        }
                        const double wait_high = waitable_nodes_[target] != 0 ?
                            safe_ends_[target_safe] : raw_high;
                        const uint64_t next_state = add_state(
                            target,
                            target_safe,
                            raw_low,
                            wait_high,
                            raw_high,
                            entry.state,
                            raw_low - duration,
                            &states,
                            &state_ids,
                            &open,
                            &sequence,
                            trace
                        );
                        if (target == goal_ && (best_goal == kNoState ||
                            states[static_cast<size_t>(next_state)].time_low <
                                states[static_cast<size_t>(best_goal)].time_low)) {
                            best_goal = next_state;
                        }
                    }
                }
            }
        }

        result->iters = expanded;
        result->nodes = states.size();
        if (best_goal == kNoState) {
            result->stop_reason = expansion_limited ? PP_SEARCH_STOP_MAX_ITERS :
                time_limited ? PP_SEARCH_STOP_TIME_BUDGET : PP_SEARCH_STOP_NO_PROGRESS;
            return 0;
        }

        std::vector<uint64_t> reverse_nodes;
        std::vector<double> reverse_times;
        uint64_t state = best_goal;
        double arrival = states[static_cast<size_t>(state)].time_low;
        while (true) {
            const KinodynamicState& current = states[static_cast<size_t>(state)];
            reverse_nodes.push_back(current.node);
            reverse_times.push_back(arrival);
            if (current.parent == kNoState) {
                break;
            }
            const KinodynamicState& parent = states[static_cast<size_t>(current.parent)];
            const double departure = current.parent_departure;
            if (departure <= parent.reachable_high) {
                arrival = departure;
            } else if (waitable_nodes_[parent.node] != 0) {
                arrival = parent.time_low;
            } else {
                throw std::runtime_error("kinodynamic SIPP interval projection lost its parent schedule");
            }
            if (arrival < parent.time_low || arrival > parent.time_high) {
                throw std::runtime_error("kinodynamic SIPP parent time is outside its wait interval");
            }
            state = current.parent;
            if (reverse_nodes.size() > states.size()) {
                throw std::runtime_error("kinodynamic SIPP parent chain contains a cycle");
            }
        }
        std::reverse(reverse_nodes.begin(), reverse_nodes.end());
        std::reverse(reverse_times.begin(), reverse_times.end());
        if (trace != nullptr) {
            trace->record(PP_TRACE_SOLUTION, goal_, kNoState, reverse_times.back());
        }
        const size_t path_length = reverse_nodes.size();
        result->path_ids = static_cast<uint64_t*>(std::malloc(path_length * sizeof(uint64_t)));
        result->arrival_times = static_cast<double*>(std::malloc(path_length * sizeof(double)));
        if (result->path_ids == nullptr || result->arrival_times == nullptr) {
            std::free(result->path_ids);
            std::free(result->arrival_times);
            result->path_ids = nullptr;
            result->arrival_times = nullptr;
            throw std::bad_alloc();
        }
        for (size_t index = 0; index < path_length; ++index) {
            result->path_ids[index] = reverse_nodes[index];
            result->arrival_times[index] = reverse_times[index];
        }
        result->path_length = path_length;
        result->arrival_time = reverse_times.back();
        result->path_cost = result->arrival_time - start_time_;
        result->success = 1;
        result->stop_reason = complete ? PP_SEARCH_STOP_SUCCESS : expansion_limited ?
            PP_SEARCH_STOP_MAX_ITERS : PP_SEARCH_STOP_TIME_BUDGET;
        return 0;
    }

private:
    std::vector<std::pair<double, double>> valid_departure_intervals(
        double low,
        double high,
        uint64_t first_block,
        uint64_t last_block,
        double duration
    ) const {
        std::vector<std::pair<double, double>> intervals;
        double cursor = low;
        for (uint64_t block = first_block; block < last_block && cursor <= high; ++block) {
            const double forbidden_low = edge_block_starts_[block] - duration + 1.0;
            const double forbidden_high = edge_block_ends_[block] - 1.0;
            if (forbidden_high < cursor || forbidden_low > high) {
                continue;
            }
            if (forbidden_low > cursor) {
                intervals.push_back({cursor, std::min(high, forbidden_low - 1.0)});
            }
            cursor = std::max(cursor, forbidden_high + 1.0);
        }
        if (cursor <= high) {
            intervals.push_back({cursor, high});
        }
        return intervals;
    }

    uint64_t add_state(
        uint64_t node,
        uint64_t safe_interval,
        double time_low,
        double time_high,
        double reachable_high,
        uint64_t parent,
        double parent_departure,
        std::vector<KinodynamicState>* states,
        std::map<std::tuple<uint64_t, uint64_t, double, double>, uint64_t>* state_ids,
        std::priority_queue<
            KinodynamicQueueEntry,
            std::vector<KinodynamicQueueEntry>,
            KinodynamicQueueEntryGreater
        >* open,
        uint64_t* sequence,
        TraceCapture* trace
    ) const {
        const auto key = std::make_tuple(node, safe_interval, time_low, time_high);
        const auto existing = state_ids->find(key);
        if (existing != state_ids->end()) {
            return existing->second;
        }
        const uint64_t state_id = static_cast<uint64_t>(states->size());
        state_ids->emplace(key, state_id);
        states->push_back({
            node, safe_interval, time_low, time_high, reachable_high,
            parent, parent_departure, false
        });
        open->push({time_low + heuristic_for(node), (*sequence)++, state_id});
        if (trace != nullptr) {
            trace->record(
                PP_TRACE_DISCOVER,
                node,
                parent == kNoState ? kNoState : (*states)[static_cast<size_t>(parent)].node,
                time_low
            );
            if (parent != kNoState) {
                trace->record(
                    PP_TRACE_PARENT,
                    node,
                    (*states)[static_cast<size_t>(parent)].node,
                    time_low
                );
            }
        }
        return state_id;
    }

    double heuristic_for(uint64_t node) const {
        return heuristic_cache_[static_cast<size_t>(node)];
    }

    void compute_heuristic(std::vector<double>* heuristic) const {
        std::vector<std::vector<std::pair<uint64_t, double>>> reverse_edges(
            static_cast<size_t>(node_count_)
        );
        for (uint64_t source = 0; source < node_count_; ++source) {
            for (uint64_t edge = offsets_[source]; edge < offsets_[source + 1]; ++edge) {
                reverse_edges[static_cast<size_t>(neighbor_ids_[edge])].push_back(
                    {source, edge_durations_[edge]}
                );
            }
        }
        heuristic->assign(static_cast<size_t>(node_count_), kInfinity);
        using DistanceEntry = std::pair<double, uint64_t>;
        std::priority_queue<
            DistanceEntry, std::vector<DistanceEntry>, std::greater<DistanceEntry>
        > open;
        (*heuristic)[static_cast<size_t>(goal_)] = 0.0;
        open.push({0.0, goal_});
        while (!open.empty()) {
            const auto [distance, node] = open.top();
            open.pop();
            if (distance != (*heuristic)[static_cast<size_t>(node)]) {
                continue;
            }
            for (const auto& [predecessor, duration] : reverse_edges[static_cast<size_t>(node)]) {
                const double candidate = distance + duration;
                const size_t predecessor_index = static_cast<size_t>(predecessor);
                if (candidate < (*heuristic)[predecessor_index]) {
                    (*heuristic)[predecessor_index] = candidate;
                    open.push({candidate, predecessor});
                }
            }
        }
        heuristic_cache_ = *heuristic;
    }

    void validate() const {
        if (node_count_ == 0 || offsets_ == nullptr || waitable_nodes_ == nullptr ||
            safe_offsets_ == nullptr || edge_block_offsets_ == nullptr ||
            start_ >= node_count_ || goal_ >= node_count_ || !std::isfinite(start_time_) ||
            std::floor(start_time_) != start_time_) {
            throw std::invalid_argument("kinodynamic SIPP graph, endpoints, or start time are invalid");
        }
        if (edge_count_ > 0 && (neighbor_ids_ == nullptr || edge_durations_ == nullptr)) {
            throw std::invalid_argument("kinodynamic SIPP edge arrays must not be null");
        }
        if (safe_count_ > 0 && (safe_starts_ == nullptr || safe_ends_ == nullptr)) {
            throw std::invalid_argument("kinodynamic SIPP safe interval arrays must not be null");
        }
        if (edge_block_count_ > 0 && (edge_block_starts_ == nullptr || edge_block_ends_ == nullptr)) {
            throw std::invalid_argument("kinodynamic SIPP edge block arrays must not be null");
        }
        if (offsets_[0] != 0 || offsets_[node_count_] != edge_count_ ||
            safe_offsets_[0] != 0 || safe_offsets_[node_count_] != safe_count_ ||
            edge_block_offsets_[0] != 0 || edge_block_offsets_[edge_count_] != edge_block_count_) {
            throw std::invalid_argument("kinodynamic SIPP CSR offsets do not match array lengths");
        }
        for (uint64_t node = 0; node < node_count_; ++node) {
            if (waitable_nodes_[node] > 1 || offsets_[node] > offsets_[node + 1] ||
                safe_offsets_[node] > safe_offsets_[node + 1]) {
                throw std::invalid_argument("kinodynamic SIPP node data is invalid");
            }
            double previous_end = -kInfinity;
            for (uint64_t safe = safe_offsets_[node]; safe < safe_offsets_[node + 1]; ++safe) {
                const double start = safe_starts_[safe];
                const double end = safe_ends_[safe];
                if (!std::isfinite(start) || std::floor(start) != start ||
                    std::isnan(end) || (std::isfinite(end) && std::floor(end) != end) ||
                    start > end || start <= previous_end) {
                    throw std::invalid_argument("kinodynamic SIPP safe intervals must be sorted inclusive ticks");
                }
                previous_end = end;
            }
        }
        for (uint64_t edge = 0; edge < edge_count_; ++edge) {
            const double duration = edge_durations_[edge];
            if (neighbor_ids_[edge] >= node_count_ || !std::isfinite(duration) || duration <= 0.0 ||
                std::floor(duration) != duration || edge_block_offsets_[edge] > edge_block_offsets_[edge + 1]) {
                throw std::invalid_argument("kinodynamic SIPP edges require positive integer durations");
            }
            double previous_end = -kInfinity;
            for (uint64_t block = edge_block_offsets_[edge];
                block < edge_block_offsets_[edge + 1];
                ++block) {
                const double start = edge_block_starts_[block];
                const double end = edge_block_ends_[block];
                if (std::isnan(start) || std::isnan(end) || start >= end ||
                    (std::isfinite(start) && std::floor(start) != start) ||
                    (std::isfinite(end) && std::floor(end) != end) || start < previous_end) {
                    throw std::invalid_argument("kinodynamic SIPP edge blocks must be sorted integer intervals");
                }
                previous_end = end;
            }
        }
    }

    uint64_t node_count_;
    uint64_t edge_count_;
    const uint64_t* offsets_;
    const uint64_t* neighbor_ids_;
    const uint8_t* waitable_nodes_;
    const double* edge_durations_;
    const uint64_t* safe_offsets_;
    const double* safe_starts_;
    const double* safe_ends_;
    uint64_t safe_count_;
    const uint64_t* edge_block_offsets_;
    const double* edge_block_starts_;
    const double* edge_block_ends_;
    uint64_t edge_block_count_;
    uint64_t start_;
    uint64_t goal_;
    double start_time_;
    mutable std::vector<double> heuristic_cache_;
};

#if defined(PP_ENABLE_TRACE) && PP_ENABLE_TRACE
int copy_trace(const TraceCapture& capture, pp_trace_result* trace) {
    const auto& events = capture.events();
    if (!events.empty()) {
        trace->events = static_cast<pp_trace_event*>(
            std::malloc(events.size() * sizeof(pp_trace_event))
        );
        if (trace->events == nullptr) {
            return 1;
        }
        std::memcpy(trace->events, events.data(), events.size() * sizeof(pp_trace_event));
        trace->event_count = events.size();
    }
    trace->truncated = capture.truncated() ? 1 : 0;
    return 0;
}
#endif

int run_sipp(
    uint64_t node_count,
    uint64_t edge_count,
    const uint64_t* offsets,
    const uint64_t* neighbor_ids,
    const double* edge_durations,
    const uint64_t* safe_offsets,
    const double* safe_starts,
    const double* safe_ends,
    uint64_t safe_count,
    const uint64_t* edge_block_offsets,
    const double* edge_block_starts,
    const double* edge_block_ends,
    uint64_t edge_block_count,
    uint64_t start_id,
    uint64_t goal_id,
    double start_time,
    int has_max_expansions,
    uint64_t max_expansions,
    double max_runtime_ms,
    double focal_weight,
    pp_sipp_result* result,
    TraceCapture* capture
) {
    if (result == nullptr) {
        return 1;
    }
    try {
        SippSearch search(
            node_count, edge_count, offsets, neighbor_ids, edge_durations,
            safe_offsets, safe_starts, safe_ends, safe_count,
            edge_block_offsets, edge_block_starts, edge_block_ends, edge_block_count,
            start_id, goal_id, start_time
        );
        return search.plan(
            has_max_expansions, max_expansions, max_runtime_ms, focal_weight, result, capture
        );
    } catch (const std::exception& error) {
        set_error(result, error.what());
        return 1;
    } catch (...) {
        set_error(result, "unknown SIPP search error");
        return 1;
    }
}

int run_kinodynamic_sipp(
    uint64_t node_count,
    uint64_t edge_count,
    const uint64_t* offsets,
    const uint64_t* neighbor_ids,
    const uint8_t* waitable_nodes,
    const double* edge_durations,
    const uint64_t* safe_offsets,
    const double* safe_starts,
    const double* safe_ends,
    uint64_t safe_count,
    const uint64_t* edge_block_offsets,
    const double* edge_block_starts,
    const double* edge_block_ends,
    uint64_t edge_block_count,
    uint64_t start_id,
    uint64_t goal_id,
    double start_time,
    int has_max_expansions,
    uint64_t max_expansions,
    double max_runtime_ms,
    pp_sipp_result* result,
    TraceCapture* capture
) {
    if (result == nullptr) {
        return 1;
    }
    try {
        KinodynamicSippSearch search(
            node_count, edge_count, offsets, neighbor_ids, waitable_nodes, edge_durations,
            safe_offsets, safe_starts, safe_ends, safe_count,
            edge_block_offsets, edge_block_starts, edge_block_ends, edge_block_count,
            start_id, goal_id, start_time
        );
        return search.plan(has_max_expansions, max_expansions, max_runtime_ms, result, capture);
    } catch (const std::exception& error) {
        set_error(result, error.what());
        return 1;
    } catch (...) {
        set_error(result, "unknown kinodynamic SIPP search error");
        return 1;
    }
}

}  // namespace

extern "C" int pp_sipp_plan(
    uint64_t node_count,
    uint64_t edge_count,
    const uint64_t* offsets,
    const uint64_t* neighbor_ids,
    const double* edge_durations,
    const uint64_t* node_safe_offsets,
    const double* node_safe_starts,
    const double* node_safe_ends,
    uint64_t node_safe_interval_count,
    const uint64_t* edge_block_offsets,
    const double* edge_block_starts,
    const double* edge_block_ends,
    uint64_t edge_block_interval_count,
    uint64_t start_id,
    uint64_t goal_id,
    double start_time,
    int has_max_expansions,
    uint64_t max_expansions,
    double max_runtime_ms,
    pp_sipp_result* result
) {
    if (result == nullptr) {
        return 1;
    }
    return run_sipp(
        node_count, edge_count, offsets, neighbor_ids, edge_durations,
        node_safe_offsets, node_safe_starts, node_safe_ends, node_safe_interval_count,
        edge_block_offsets, edge_block_starts, edge_block_ends, edge_block_interval_count,
        start_id, goal_id, start_time, has_max_expansions, max_expansions, max_runtime_ms, 0.0,
        result, nullptr
    );
}

extern "C" int pp_bounded_sipp_plan(
    uint64_t node_count,
    uint64_t edge_count,
    const uint64_t* offsets,
    const uint64_t* neighbor_ids,
    const double* edge_durations,
    const uint64_t* node_safe_offsets,
    const double* node_safe_starts,
    const double* node_safe_ends,
    uint64_t node_safe_interval_count,
    const uint64_t* edge_block_offsets,
    const double* edge_block_starts,
    const double* edge_block_ends,
    uint64_t edge_block_interval_count,
    uint64_t start_id,
    uint64_t goal_id,
    double start_time,
    int has_max_expansions,
    uint64_t max_expansions,
    double max_runtime_ms,
    double suboptimality_weight,
    pp_sipp_result* result
) {
    if (result == nullptr) {
        return 1;
    }
    if (!std::isfinite(suboptimality_weight) || suboptimality_weight < 1.0) {
        set_error(result, "suboptimality weight must be finite and at least 1");
        return 1;
    }
    return run_sipp(
        node_count, edge_count, offsets, neighbor_ids, edge_durations,
        node_safe_offsets, node_safe_starts, node_safe_ends, node_safe_interval_count,
        edge_block_offsets, edge_block_starts, edge_block_ends, edge_block_interval_count,
        start_id, goal_id, start_time, has_max_expansions, max_expansions, max_runtime_ms,
        suboptimality_weight, result, nullptr
    );
}

extern "C" int pp_kinodynamic_sipp_plan(
    uint64_t node_count,
    uint64_t edge_count,
    const uint64_t* offsets,
    const uint64_t* neighbor_ids,
    const uint8_t* waitable_nodes,
    const double* edge_durations,
    const uint64_t* node_safe_offsets,
    const double* node_safe_starts,
    const double* node_safe_ends,
    uint64_t node_safe_interval_count,
    const uint64_t* edge_block_offsets,
    const double* edge_block_starts,
    const double* edge_block_ends,
    uint64_t edge_block_interval_count,
    uint64_t start_id,
    uint64_t goal_id,
    double start_time,
    int has_max_expansions,
    uint64_t max_expansions,
    double max_runtime_ms,
    pp_sipp_result* result
) {
    return run_kinodynamic_sipp(
        node_count, edge_count, offsets, neighbor_ids, waitable_nodes, edge_durations,
        node_safe_offsets, node_safe_starts, node_safe_ends, node_safe_interval_count,
        edge_block_offsets, edge_block_starts, edge_block_ends, edge_block_interval_count,
        start_id, goal_id, start_time, has_max_expansions, max_expansions, max_runtime_ms,
        result, nullptr
    );
}

#if defined(PP_ENABLE_TRACE) && PP_ENABLE_TRACE
extern "C" int pp_sipp_plan_traced(
    uint64_t node_count,
    uint64_t edge_count,
    const uint64_t* offsets,
    const uint64_t* neighbor_ids,
    const double* edge_durations,
    const uint64_t* node_safe_offsets,
    const double* node_safe_starts,
    const double* node_safe_ends,
    uint64_t node_safe_interval_count,
    const uint64_t* edge_block_offsets,
    const double* edge_block_starts,
    const double* edge_block_ends,
    uint64_t edge_block_interval_count,
    uint64_t start_id,
    uint64_t goal_id,
    double start_time,
    int has_max_expansions,
    uint64_t max_expansions,
    double max_runtime_ms,
    uint64_t trace_max_bytes,
    pp_sipp_result* result,
    pp_trace_result* trace
) {
    if (trace == nullptr) {
        return 1;
    }
    *trace = pp_trace_result{};
    TraceCapture capture(trace_max_bytes);
    const int status = run_sipp(
        node_count, edge_count, offsets, neighbor_ids, edge_durations,
        node_safe_offsets, node_safe_starts, node_safe_ends, node_safe_interval_count,
        edge_block_offsets, edge_block_starts, edge_block_ends, edge_block_interval_count,
        start_id, goal_id, start_time, has_max_expansions, max_expansions, max_runtime_ms, 0.0,
        result, &capture
    );
    if (copy_trace(capture, trace) != 0) {
        pp_sipp_free_result(result);
        pp_search_trace_free_result(trace);
        set_error(result, "could not allocate SIPP trace events");
        return 1;
    }
    return status;
}

extern "C" int pp_bounded_sipp_plan_traced(
    uint64_t node_count,
    uint64_t edge_count,
    const uint64_t* offsets,
    const uint64_t* neighbor_ids,
    const double* edge_durations,
    const uint64_t* node_safe_offsets,
    const double* node_safe_starts,
    const double* node_safe_ends,
    uint64_t node_safe_interval_count,
    const uint64_t* edge_block_offsets,
    const double* edge_block_starts,
    const double* edge_block_ends,
    uint64_t edge_block_interval_count,
    uint64_t start_id,
    uint64_t goal_id,
    double start_time,
    int has_max_expansions,
    uint64_t max_expansions,
    double max_runtime_ms,
    double suboptimality_weight,
    uint64_t trace_max_bytes,
    pp_sipp_result* result,
    pp_trace_result* trace
) {
    if (trace == nullptr) {
        return 1;
    }
    *trace = pp_trace_result{};
    if (result == nullptr) {
        return 1;
    }
    if (!std::isfinite(suboptimality_weight) || suboptimality_weight < 1.0) {
        set_error(result, "suboptimality weight must be finite and at least 1");
        return 1;
    }
    TraceCapture capture(trace_max_bytes);
    const int status = run_sipp(
        node_count, edge_count, offsets, neighbor_ids, edge_durations,
        node_safe_offsets, node_safe_starts, node_safe_ends, node_safe_interval_count,
        edge_block_offsets, edge_block_starts, edge_block_ends, edge_block_interval_count,
        start_id, goal_id, start_time, has_max_expansions, max_expansions, max_runtime_ms,
        suboptimality_weight, result, &capture
    );
    if (copy_trace(capture, trace) != 0) {
        pp_sipp_free_result(result);
        pp_search_trace_free_result(trace);
        set_error(result, "could not allocate bounded SIPP trace events");
        return 1;
    }
    return status;
}

extern "C" int pp_kinodynamic_sipp_plan_traced(
    uint64_t node_count,
    uint64_t edge_count,
    const uint64_t* offsets,
    const uint64_t* neighbor_ids,
    const uint8_t* waitable_nodes,
    const double* edge_durations,
    const uint64_t* node_safe_offsets,
    const double* node_safe_starts,
    const double* node_safe_ends,
    uint64_t node_safe_interval_count,
    const uint64_t* edge_block_offsets,
    const double* edge_block_starts,
    const double* edge_block_ends,
    uint64_t edge_block_interval_count,
    uint64_t start_id,
    uint64_t goal_id,
    double start_time,
    int has_max_expansions,
    uint64_t max_expansions,
    double max_runtime_ms,
    uint64_t trace_max_bytes,
    pp_sipp_result* result,
    pp_trace_result* trace
) {
    if (trace == nullptr) {
        return 1;
    }
    *trace = pp_trace_result{};
    if (result == nullptr) {
        return 1;
    }
    TraceCapture capture(trace_max_bytes);
    const int status = run_kinodynamic_sipp(
        node_count, edge_count, offsets, neighbor_ids, waitable_nodes, edge_durations,
        node_safe_offsets, node_safe_starts, node_safe_ends, node_safe_interval_count,
        edge_block_offsets, edge_block_starts, edge_block_ends, edge_block_interval_count,
        start_id, goal_id, start_time, has_max_expansions, max_expansions, max_runtime_ms,
        result, &capture
    );
    if (copy_trace(capture, trace) != 0) {
        pp_sipp_free_result(result);
        pp_search_trace_free_result(trace);
        set_error(result, "could not allocate kinodynamic SIPP trace events");
        return 1;
    }
    return status;
}
#endif

extern "C" void pp_sipp_free_result(pp_sipp_result* result) {
    if (result == nullptr) {
        return;
    }
    std::free(result->path_ids);
    std::free(result->arrival_times);
    std::free(result->error_message);
    *result = pp_sipp_result{};
}
