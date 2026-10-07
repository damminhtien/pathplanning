// C ABI for reusable discrete graph-search kernels.
//
// Graphs are copied into an opaque native CSR handle before search begins. The
// search ABI receives flat goal and heuristic arrays and never calls back into
// the host language while expanding nodes.

#ifndef PATHPLANNING_NATIVE_SEARCH_ENGINE_H_
#define PATHPLANNING_NATIVE_SEARCH_ENGINE_H_

#include <stddef.h>
#include <stdint.h>

#include "abi_version.h"
#if defined(PP_ENABLE_TRACE) && PP_ENABLE_TRACE
#include "trace_engine.h"
#endif
#if defined(PP_ENABLE_METRICS) && PP_ENABLE_METRICS
#include "search_metrics.h"
#endif

#ifdef __cplusplus
extern "C" {
#endif

typedef enum pp_search_stop_reason {
    PP_SEARCH_STOP_SUCCESS = 0,
    PP_SEARCH_STOP_MAX_ITERS = 1,
    PP_SEARCH_STOP_NO_PROGRESS = 2,
    PP_SEARCH_STOP_ERROR = 3,
    PP_SEARCH_STOP_TIME_BUDGET = 4,
} pp_search_stop_reason;

typedef enum pp_search_algorithm {
    PP_SEARCH_BFS = 1,
    PP_SEARCH_DFS = 2,
    PP_SEARCH_GREEDY_BEST_FIRST = 3,
    PP_SEARCH_ASTAR = 4,
    PP_SEARCH_DIJKSTRA = 5,
    PP_SEARCH_WEIGHTED_ASTAR = 6,
    PP_SEARCH_BIDIRECTIONAL_DIJKSTRA = 7,
    PP_SEARCH_ANYTIME_ASTAR = 8,
    PP_SEARCH_BIDIRECTIONAL_ASTAR = 9,
    PP_SEARCH_REEXP_ASTAR = 10,
} pp_search_algorithm;

typedef enum pp_search_reopen_mode {
    PP_SEARCH_REOPEN_ABS = 0,
    PP_SEARCH_REOPEN_REL_EDGE = 1,
    PP_SEARCH_REOPEN_REL_G = 2,
} pp_search_reopen_mode;

typedef enum pp_search_tie_break {
    PP_SEARCH_TIE_G_LOW = 0,
    PP_SEARCH_TIE_G_HIGH = 1,
} pp_search_tie_break;

typedef struct pp_native_graph pp_native_graph;

// Borrowed pointers remain valid until the graph is freed. Keep the owner
// alive while copying this view into another native library.
typedef struct pp_graph_csr_view {
    uint64_t node_count;
    uint64_t edge_count;
    const uint64_t* offsets;
    const uint64_t* neighbor_ids;
    const double* edge_costs;
    uint64_t grid_width;
    uint64_t grid_height;
    uint64_t grid_depth;
    int euclidean_grid_heuristic;
} pp_graph_csr_view;

// Capacity bytes are requested payload bytes, not resident memory. The graph
// object contains the vector headers and synchronization state.
typedef struct pp_graph_storage_info {
    uint64_t struct_size;
    uint64_t graph_object_bytes;
    uint64_t base_csr_capacity_bytes;
    uint64_t reverse_csr_capacity_bytes;
    uint64_t node_count;
    uint64_t edge_count;
} pp_graph_storage_info;

typedef struct pp_search_options {
    int algorithm;
    int has_max_expansions;
    uint64_t max_expansions;
    double heuristic_weight;
    uint64_t reserve_nodes;
    int has_goal_id;
    uint64_t goal_id;
    const double* anytime_weights;
    size_t anytime_weight_count;
    double reopen_threshold;  // Non-negative; positive infinity disables reopening.
    int reopen_mode;          // pp_search_reopen_mode
    int tie_break;            // pp_search_tie_break
    double max_runtime_ms;    // Milliseconds; zero disables the runtime budget.
} pp_search_options;

// Native-owned path_ids and error_message must only be released by
// pp_search_free_result.
typedef struct pp_search_result {
    int success;
    int stop_reason;
    uint64_t iters;
    uint64_t reopens;
    uint64_t nodes;
    double path_cost;
    uint64_t* path_ids;
    size_t path_length;
    char* error_message;
} pp_search_result;

typedef struct pp_jps_metrics {
    uint64_t struct_size;
    uint64_t motion_checks;
    uint64_t jump_points_expanded;
} pp_jps_metrics;

// Returns PP_SEARCH_ABI_VERSION. This exact-match version covers exported
// functions, structures, enums, and callback semantics, independently of the
// package and engine implementation versions.
uint32_t pp_search_abi_version(void);

// Copies all input arrays before returning. On success, *out_graph is an owned
// opaque handle and must be released with pp_graph_free exactly once. On
// failure, the diagnostic is written to the caller-owned bounded buffer.
int pp_graph_create_csr(
    uint64_t node_count,
    uint64_t edge_count,
    const uint64_t* offsets,
    const uint64_t* neighbor_ids,
    const double* edge_costs,
    pp_native_graph** out_graph,
    char* error_message,
    size_t error_capacity
);

// Exports a borrowed view; the arrays are never copied by this call.
int pp_graph_export_csr_view(const pp_native_graph* graph, pp_graph_csr_view* out_view);

// Prepare retained reverse adjacency once; repeated calls are idempotent.
int pp_graph_prepare_reverse(
    const pp_native_graph* graph,
    char* error_message,
    size_t error_capacity
);

// Set out_info->struct_size before calling. Does not trigger reverse prepare.
int pp_graph_get_storage_info(
    const pp_native_graph* graph,
    pp_graph_storage_info* out_info
);

// Copies motion and validity arrays before returning and follows the same
// output-handle and error-buffer ownership rules as pp_graph_create_csr.
int pp_graph_create_grid(
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
);

int pp_graph_create_grid_ex(
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
);

// Accepts NULL. Frees a graph handle created by this library.
void pp_graph_free(pp_native_graph* graph);

// Returns zero when the call ran and nonzero on an API error; plan success and
// stop reason are reported in result. The graph and arrays are borrowed for
// this call. Result buffers are owned by the result and must be released with
// pp_search_free_result before the result storage is reused.
int pp_native_search_plan(
    const pp_native_graph* graph,
    const uint8_t* goal_flags,
    const double* heuristic_values,
    uint64_t start_id,
    const pp_search_options* options,
    pp_search_result* result
);

// Runs Jump Point Search directly on a row-major validity grid. Diagonal moves
// require both adjacent orthogonal cells to be valid.
int pp_native_jps_grid(
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
);

#if defined(PP_ENABLE_TRACE) && PP_ENABLE_TRACE
int pp_native_jps_grid_traced(
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
);
#endif

#if (defined(PP_ENABLE_TRACE) && PP_ENABLE_TRACE) || \
    (defined(PP_ENABLE_METRICS) && PP_ENABLE_METRICS)
// Imports a borrowed CSR view into this library's own graph handle.
int pp_graph_create_csr_view(
    const pp_graph_csr_view* view,
    pp_native_graph** out_graph,
    char* error_message,
    size_t error_capacity
);

// Diagnostic-library entry. A full trace stops recording when max_bytes is
// exhausted, while the search continues. The result and trace are independent
// native-owned outputs and must each be freed in the same library.
#if defined(PP_ENABLE_TRACE) && PP_ENABLE_TRACE
int pp_native_search_plan_traced(
    const pp_native_graph* graph,
    const uint8_t* goal_flags,
    const double* heuristic_values,
    uint64_t start_id,
    const pp_search_options* options,
    pp_search_result* result,
    uint64_t max_bytes,
    pp_trace_result* trace
);

uint32_t pp_search_trace_abi_version(void);
uint64_t pp_graph_storage_bytes(const pp_native_graph* graph);
void pp_search_trace_free_result(pp_trace_result* trace);
#endif
#endif

#if defined(PP_ENABLE_METRICS) && PP_ENABLE_METRICS
int pp_native_search_plan_measured(
    const pp_native_graph* graph,
    const uint8_t* goal_flags,
    const double* heuristic_values,
    uint64_t start_id,
    const pp_search_options* options,
    pp_search_result* result,
    pp_search_metrics* metrics
);
#endif

// Accepts NULL; after a result call, releases its owned buffers and resets it.
void pp_search_free_result(pp_search_result* result);

const char* pp_search_engine_version(void);

#ifdef __cplusplus
}
#endif

#endif  // PATHPLANNING_NATIVE_SEARCH_ENGINE_H_
