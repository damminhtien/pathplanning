// C ABI for reusable discrete graph-search kernels.
//
// Graphs are copied into an opaque native CSR handle before search begins. The
// search ABI receives flat goal and heuristic arrays and never calls back into
// the host language while expanding nodes.

#ifndef PATHPLANNING_NATIVE_SEARCH_ENGINE_H_
#define PATHPLANNING_NATIVE_SEARCH_ENGINE_H_

#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef enum pp_search_stop_reason {
    PP_SEARCH_STOP_SUCCESS = 0,
    PP_SEARCH_STOP_MAX_ITERS = 1,
    PP_SEARCH_STOP_NO_PROGRESS = 2,
    PP_SEARCH_STOP_ERROR = 3,
} pp_search_stop_reason;

typedef enum pp_search_algorithm {
    PP_SEARCH_BFS = 1,
    PP_SEARCH_DFS = 2,
    PP_SEARCH_GREEDY_BEST_FIRST = 3,
    PP_SEARCH_ASTAR = 4,
    PP_SEARCH_DIJKSTRA = 5,
    PP_SEARCH_WEIGHTED_ASTAR = 6,
    PP_SEARCH_BIDIRECTIONAL_ASTAR = 7,
    PP_SEARCH_ANYTIME_ASTAR = 8,
} pp_search_algorithm;

// Legacy callback input is retained for ABI compatibility. Its graph is
// snapshotted before entering a search kernel.
typedef int (*pp_goal_callback)(void* user_data, uint64_t node_id, int* out_is_goal);
typedef int (*pp_heuristic_callback)(void* user_data, uint64_t node_id, double* out_value);
typedef int (*pp_neighbors_callback)(
    void* user_data,
    uint64_t node_id,
    const uint64_t** out_neighbor_ids,
    const double** out_edge_costs,
    size_t* out_count
);

typedef struct pp_graph_callbacks {
    void* user_data;
    pp_goal_callback is_goal;
    pp_heuristic_callback heuristic;
    pp_neighbors_callback neighbors;
} pp_graph_callbacks;

typedef struct pp_astar_options {
    int has_max_expansions;
    uint64_t max_expansions;
    double heuristic_weight;
    uint64_t reserve_nodes;
} pp_astar_options;

typedef struct pp_native_graph pp_native_graph;

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
} pp_search_options;

typedef struct pp_search_result {
    int success;
    int stop_reason;
    uint64_t iters;
    uint64_t nodes;
    double path_cost;
    uint64_t* path_ids;
    size_t path_length;
    char* error_message;
} pp_search_result;

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

void pp_graph_free(pp_native_graph* graph);

int pp_search_plan(
    const pp_graph_callbacks* graph,
    uint64_t start_id,
    const pp_search_options* options,
    pp_search_result* result
);

int pp_native_search_plan(
    const pp_native_graph* graph,
    const uint8_t* goal_flags,
    const double* heuristic_values,
    uint64_t start_id,
    const pp_search_options* options,
    pp_search_result* result
);

int pp_astar_plan(
    const pp_graph_callbacks* graph,
    uint64_t start_id,
    const pp_astar_options* options,
    pp_search_result* result
);

void pp_search_free_result(pp_search_result* result);

const char* pp_search_engine_version(void);

#ifdef __cplusplus
}
#endif

#endif  // PATHPLANNING_NATIVE_SEARCH_ENGINE_H_
