#ifndef PATHPLANNING_NATIVE_CONTINUOUS_ENGINE_H_
#define PATHPLANNING_NATIVE_CONTINUOUS_ENGINE_H_

#include <stddef.h>
#include <stdint.h>

#include "abi_version.h"
#ifdef PP_ENABLE_TRACE
#include "trace_engine.h"
#endif

#ifdef __cplusplus
extern "C" {
#endif

typedef enum pp_continuous_algorithm {
    PP_CONTINUOUS_RRT = 1,
    PP_CONTINUOUS_RRT_STAR = 2,
    PP_CONTINUOUS_INFORMED_RRT_STAR = 3,
    PP_CONTINUOUS_FMT_STAR = 4,
    PP_CONTINUOUS_BIT_STAR = 5,
    PP_CONTINUOUS_ABIT_STAR = 6,
    PP_CONTINUOUS_RRT_CONNECT = 7,
    PP_CONTINUOUS_AIT_STAR = 8
} pp_continuous_algorithm;

// Return 0 on success and nonzero on failure. out_state is read only on success.
typedef int (*pp_sample_free_callback)(void *user_data, double *out_state, size_t dimension);

// Return 0 for false, 1 for true, and a negative value or a value greater than
// 1 for failure.
typedef int (*pp_state_valid_callback)(void *user_data, const double *state, size_t dimension);
typedef int (*pp_motion_valid_callback)(void *user_data, const double *start,
                                        const double *end, size_t dimension,
                                        double collision_step);

// Return a finite, non-negative value. Negative or non-finite values fail the
// plan.
typedef double (*pp_distance_callback)(void *user_data, const double *start,
                                       const double *end, size_t dimension);

// Return 0 on success and nonzero on failure. out_state is read only on success.
typedef int (*pp_steer_callback)(void *user_data, const double *start,
                                 const double *target, size_t dimension,
                                 double step_size, double *out_state);

// Return 0 for false, 1 for true, and a negative value or a value greater than
// 1 for failure.
typedef int (*pp_goal_callback)(void *user_data, const double *state, size_t dimension);

// Return a non-negative value; positive infinity means no usable estimate.
// NaN or a negative value fails the plan.
typedef double (*pp_goal_distance_callback)(void *user_data, const double *state,
                                            size_t dimension);

// Return a finite value. Non-finite values fail the plan.
typedef double (*pp_path_objective_callback)(void *user_data, const double *path,
                                              size_t path_length, size_t dimension);

typedef struct pp_continuous_space_model {
    const double *lower_bounds;
    const double *upper_bounds;
    const double *box_minima;
    const double *box_maxima;
    size_t box_count;
    const double *sphere_centers;
    const double *sphere_radii;
    size_t sphere_count;
    const double *obb_centers;
    const double *obb_extents;
    const double *obb_orientations;
    size_t obb_count;
} pp_continuous_space_model;

typedef struct pp_continuous_callbacks {
    void *user_data;
    pp_sample_free_callback sample_free;
    pp_state_valid_callback state_valid;
    pp_motion_valid_callback motion_valid;
    pp_distance_callback distance;
    pp_steer_callback steer;
    pp_goal_callback is_goal;
    pp_goal_distance_callback goal_distance;
    pp_path_objective_callback path_objective;
    const pp_continuous_space_model *native_space;
    int native_goal;
    double goal_radius;
} pp_continuous_callbacks;

typedef struct pp_continuous_options {
    int algorithm;
    uint64_t max_iters;
    uint64_t sample_count;
    uint64_t batch_size;
    uint64_t max_sample_tries;
    uint64_t seed;
    double step_size;
    double goal_sample_rate;
    double collision_step;
    double goal_reach_tolerance;
    double rrt_star_radius_gamma;
    double rrt_star_radius_max_factor;
    double abit_inflation_parameter;
    double abit_truncation_parameter;
    double time_budget_s;
    int use_euclidean_index;
} pp_continuous_options;

typedef struct pp_continuous_result {
    int success;
    int stop_reason;
    uint64_t iters;
    uint64_t nodes;
    uint64_t sample_count;
    uint64_t batches;
    uint64_t motion_checks;
    uint64_t rewires;
    double path_cost;
    double elapsed_s;
    double *path;
    size_t path_length;
    size_t dimension;
    char *error_message;
} pp_continuous_result;

typedef struct pp_dynamic_rrt_result {
    pp_continuous_result plan;
    double *tree_points;
    uint32_t *tree_parents;
    size_t tree_count;
    uint32_t *invalid_nodes;
    size_t invalid_count;
} pp_dynamic_rrt_result;

typedef struct pp_prm_star_roadmap pp_prm_star_roadmap;

// Returns PP_CONTINUOUS_ABI_VERSION. This exact-match version covers exported
// functions, structures, enums, and callback semantics.
uint32_t pp_continuous_abi_version(void);

// Inputs, callbacks, user_data, and model arrays are borrowed for the duration
// of the call. The result owns its path and error_message buffers; call the
// matching free function after every call, including failures, and before
// reusing result storage. Error messages can be NULL if allocation fails.
int pp_continuous_plan(const pp_continuous_callbacks *callbacks,
                       const double *start, const double *goal,
                       size_t dimension, int has_goal_point,
                       const pp_continuous_options *options,
                       pp_continuous_result *result);
void pp_continuous_free_result(pp_continuous_result *result);

// Uses the same borrowed-input rules. Its result owns the nested plan buffers
// and tree arrays and must be released with pp_dynamic_rrt_free_result.
int pp_dynamic_rrt_plan(const pp_continuous_callbacks *callbacks,
                        const double *start, const double *goal,
                        const double *initial_points,
                        const uint32_t *initial_parents,
                        size_t initial_count, size_t dimension,
                        const pp_continuous_options *options,
                        double waypoint_sample_rate,
                        int prune_only,
                        pp_dynamic_rrt_result *result);
void pp_dynamic_rrt_free_result(pp_dynamic_rrt_result *result);

int pp_prm_star_create(size_t dimension, uint64_t sample_count, double gamma,
                       uint64_t max_sample_tries, uint64_t seed,
                       pp_prm_star_roadmap **out, char *error, size_t error_capacity);
int pp_prm_star_set_lazy(pp_prm_star_roadmap *roadmap, int lazy);
int pp_prm_star_build(pp_prm_star_roadmap *roadmap,
                      const pp_continuous_callbacks *callbacks,
                      double collision_step, double time_budget_s,
                      pp_continuous_result *result);
int pp_prm_star_query(pp_prm_star_roadmap *roadmap,
                      const pp_continuous_callbacks *callbacks,
                      const double *start, const double *goal,
                      double collision_step, uint64_t max_expansions,
                      double time_budget_s, pp_continuous_result *result);
void pp_prm_star_clear_query(pp_prm_star_roadmap *roadmap);
void pp_prm_star_reset(pp_prm_star_roadmap *roadmap);
void pp_prm_star_free(pp_prm_star_roadmap *roadmap);
const char *pp_continuous_engine_version(void);

#ifdef PP_ENABLE_TRACE
// Diagnostic-only entry points. The caller owns both result buffers and must
// release them with the matching functions from this diagnostic library.
uint32_t pp_continuous_trace_abi_version(void);
int pp_continuous_plan_traced(const pp_continuous_callbacks *callbacks,
                              const double *start, const double *goal,
                              size_t dimension, int has_goal_point,
                              const pp_continuous_options *options,
                              pp_continuous_result *result,
                              uint64_t max_bytes, pp_trace_result *trace);
int pp_dynamic_rrt_plan_traced(const pp_continuous_callbacks *callbacks,
                               const double *start, const double *goal,
                               const double *initial_points,
                               const uint32_t *initial_parents,
                               size_t initial_count, size_t dimension,
                               const pp_continuous_options *options,
                               double waypoint_sample_rate, int prune_only,
                               pp_dynamic_rrt_result *result,
                               uint64_t max_bytes, pp_trace_result *trace);
void pp_continuous_trace_free_result(pp_trace_result *trace);
int pp_prm_star_query_traced(pp_prm_star_roadmap *roadmap,
                             const pp_continuous_callbacks *callbacks,
                             const double *start, const double *goal,
                             double collision_step, uint64_t max_expansions,
                             double time_budget_s, pp_continuous_result *result,
                             uint64_t max_bytes, pp_trace_result *trace);
#endif

#ifdef __cplusplus
}
#endif

#endif
