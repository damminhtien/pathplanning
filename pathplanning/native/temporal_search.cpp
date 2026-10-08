#include "search_engine.h"

#include <algorithm>
#include <chrono>
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

        const size_t state_count = static_cast<size_t>(safe_count_);
        std::vector<uint64_t> state_nodes(state_count);
        std::vector<double> arrivals(state_count, kInfinity);
        std::vector<uint64_t> parents(state_count, kNoState);
        std::priority_queue<QueueEntry, std::vector<QueueEntry>, QueueEntryGreater> open;
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

        arrivals[static_cast<size_t>(start_state)] = start_time_;
        open.push({start_time_, sequence++, start_state});
        if (trace != nullptr) {
            trace->record(PP_TRACE_DISCOVER, start_, kNoState, start_time_);
        }
        uint64_t expanded = 0;
        uint64_t discovered = 1;
        uint64_t best_goal = start_ == goal_ ? start_state : kNoState;
        bool complete = best_goal != kNoState;
        bool expansion_limited = false;
        bool time_limited = false;
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
            const QueueEntry entry = open.top();
            open.pop();
            if (entry.arrival != arrivals[static_cast<size_t>(entry.state)]) {
                continue;
            }
            const uint64_t node = state_nodes[static_cast<size_t>(entry.state)];
            if (node == goal_) {
                best_goal = entry.state;
                complete = true;
                break;
            }
            ++expanded;
            if (trace != nullptr) {
                trace->record(PP_TRACE_EXPAND, node, parents[static_cast<size_t>(entry.state)] == kNoState ?
                    kNoState : state_nodes[static_cast<size_t>(parents[static_cast<size_t>(entry.state)])],
                    entry.arrival);
            }

            const double source_interval_end = safe_ends_[entry.state];
            for (uint64_t edge = offsets_[node]; edge < offsets_[node + 1]; ++edge) {
                const uint64_t target = neighbor_ids_[edge];
                const double duration = edge_durations_[edge];
                const uint64_t first_target_state = safe_offsets_[target];
                const uint64_t last_target_state = safe_offsets_[target + 1];
                for (uint64_t target_state = first_target_state;
                    target_state < last_target_state;
                    ++target_state) {
                    double departure = std::max(
                        entry.arrival,
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
                    parents[state_index] = entry.state;
                    open.push({arrival, sequence++, target_state});
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
        return search.plan(has_max_expansions, max_expansions, max_runtime_ms, result, capture);
    } catch (const std::exception& error) {
        set_error(result, error.what());
        return 1;
    } catch (...) {
        set_error(result, "unknown SIPP search error");
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
    return run_sipp(
        node_count, edge_count, offsets, neighbor_ids, edge_durations,
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
        start_id, goal_id, start_time, has_max_expansions, max_expansions, max_runtime_ms,
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
