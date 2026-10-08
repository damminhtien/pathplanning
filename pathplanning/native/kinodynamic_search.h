#ifndef PATHPLANNING_NATIVE_KINODYNAMIC_SEARCH_H_
#define PATHPLANNING_NATIVE_KINODYNAMIC_SEARCH_H_

#include <stddef.h>
#include <stdint.h>

#include "abi_version.h"
#include "trace_engine.h"

#ifdef __cplusplus
extern "C" {
#endif

/* Return 0 for invalid, 1 for valid, or any other value on callback failure. */
typedef int (*pp_hybrid_state_valid_callback)(void *user_data,
                                              const double *state,
                                              size_t dimension);

typedef struct pp_hybrid_astar_space {
    uint64_t grid_width;
    uint64_t grid_height;
    const uint8_t *occupancy;
    double grid_resolution;
    double origin_x;
    double origin_y;
    double wheelbase;
    double max_steering_angle;
    double footprint_length;
    double footprint_width;
    pp_hybrid_state_valid_callback state_valid;
    void *user_data;
} pp_hybrid_astar_space;

typedef struct pp_hybrid_astar_options {
    uint64_t max_expansions;
    double max_runtime_ms;
    double xy_resolution;
    uint64_t heading_bins;
    double primitive_length;
    double collision_step;
    double goal_xy_tolerance;
    double goal_yaw_tolerance;
    double analytic_expansion_distance;
    uint64_t analytic_expansion_interval;
    double heuristic_weight;
    double reverse_penalty;
    double steering_penalty;
    double direction_switch_penalty;
    int allow_reverse;
} pp_hybrid_astar_options;

typedef struct pp_hybrid_astar_result {
    int success;
    int stop_reason;
    uint64_t iters;
    uint64_t nodes;
    uint64_t motion_checks;
    uint64_t analytic_expansions;
    uint64_t first_solution_iter;
    double path_cost;
    double elapsed_s;
    double *poses;
    int8_t *directions;
    size_t path_length;
    char *error_message;
} pp_hybrid_astar_result;

typedef struct pp_state_lattice_space {
    uint64_t grid_width;
    uint64_t grid_height;
    const uint8_t *occupancy;
    double grid_resolution;
    double origin_x;
    double origin_y;
    double footprint_length;
    double footprint_width;
    double rotation_radius;
    pp_hybrid_state_valid_callback state_valid;
    void *user_data;
} pp_state_lattice_space;

typedef struct pp_state_lattice_primitive {
    const double *relative_poses;
    size_t pose_count;
    int direction;
    double cost;
} pp_state_lattice_primitive;

typedef struct pp_state_lattice_options {
    uint64_t max_expansions;
    double max_runtime_ms;
    double xy_resolution;
    uint64_t heading_bins;
    double collision_step;
    double goal_xy_tolerance;
    double goal_yaw_tolerance;
    double heuristic_weight;
    double reverse_penalty;
    double direction_switch_penalty;
} pp_state_lattice_options;

uint32_t pp_kinodynamic_abi_version(void);

int pp_hybrid_astar_plan(const pp_hybrid_astar_space *space,
                         const double *start,
                         const double *goal,
                         const pp_hybrid_astar_options *options,
                         pp_hybrid_astar_result *result);

void pp_hybrid_astar_free_result(pp_hybrid_astar_result *result);

int pp_state_lattice_plan(const pp_state_lattice_space *space,
                          const double *start,
                          const double *goal,
                          const pp_state_lattice_primitive *primitives,
                          size_t primitive_count,
                          const pp_state_lattice_options *options,
                          pp_hybrid_astar_result *result);

void pp_state_lattice_free_result(pp_hybrid_astar_result *result);

#if defined(PP_ENABLE_TRACE) && PP_ENABLE_TRACE
uint32_t pp_kinodynamic_trace_abi_version(void);

int pp_hybrid_astar_plan_traced(const pp_hybrid_astar_space *space,
                                const double *start,
                                const double *goal,
                                const pp_hybrid_astar_options *options,
                                uint64_t trace_max_bytes,
                                pp_hybrid_astar_result *result,
                                pp_trace_result *trace);

int pp_state_lattice_plan_traced(const pp_state_lattice_space *space,
                                 const double *start,
                                 const double *goal,
                                 const pp_state_lattice_primitive *primitives,
                                 size_t primitive_count,
                                 const pp_state_lattice_options *options,
                                 uint64_t trace_max_bytes,
                                 pp_hybrid_astar_result *result,
                                 pp_trace_result *trace);
#endif

#ifdef __cplusplus
}
#endif

#endif  // PATHPLANNING_NATIVE_KINODYNAMIC_SEARCH_H_
