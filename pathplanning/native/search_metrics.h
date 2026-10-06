// Optional diagnostic ABI. Only the metrics extension exports these functions.
#ifndef PATHPLANNING_NATIVE_SEARCH_METRICS_H_
#define PATHPLANNING_NATIVE_SEARCH_METRICS_H_

#include <stdint.h>

#include "abi_version.h"

#ifdef __cplusplus
extern "C" {
#endif

enum {
    PP_METRICS_CAP_WORK_COUNTERS = 1u,
    PP_METRICS_CAP_TRACKED_QUERY_ALLOCATIONS = 2u,
    PP_METRICS_CAP_NO_REOPEN = 4u,
};

// Set struct_size to sizeof(pp_search_metrics) before every call. All counts
// are exact integers. Allocation bytes cover state, frontier, reconstructed
// path vectors, and the native result path; allocator metadata and graph
// preparation are outside this query-workspace accounting boundary.
typedef struct pp_search_metrics {
    uint64_t struct_size;
    uint64_t capability_bits;
    uint64_t expanded;
    uint64_t expanded_forward;
    uint64_t expanded_backward;
    uint64_t discovered_first;
    uint64_t discovered_forward;
    uint64_t discovered_backward;
    uint64_t edges_examined;
    uint64_t relaxation_attempts;
    uint64_t relaxation_successes_first;
    uint64_t relaxation_successes_improved;
    uint64_t closed_neighbor_skips;
    uint64_t nonimproving_skips;
    uint64_t greedy_known_skips;
    uint64_t frontier_pushes;
    uint64_t frontier_pops;
    uint64_t stale_pops;
    uint64_t frontier_peak_entries;
    uint64_t frontier_peak_forward;
    uint64_t frontier_peak_backward;
    uint64_t goal_tests;
    uint64_t heuristic_lookups;
    uint64_t heuristic_computations;
    uint64_t validation_heuristic_lookups;
    uint64_t validation_heuristic_computations;
    uint64_t validation_edge_checks;
    uint64_t state_slots_allocated;
    uint64_t parent_id_bytes;
    uint64_t state_capacity_bytes_peak;
    uint64_t frontier_capacity_bytes_peak;
    uint64_t path_workspace_bytes_peak;
    uint64_t result_path_bytes;
    uint64_t query_workspace_peak_bytes;
    uint64_t native_requested_bytes_peak;
} pp_search_metrics;

uint32_t pp_search_metrics_abi_version(void);
uint64_t pp_search_metrics_struct_size(void);

#ifdef __cplusplus
}
#endif

#endif  // PATHPLANNING_NATIVE_SEARCH_METRICS_H_
