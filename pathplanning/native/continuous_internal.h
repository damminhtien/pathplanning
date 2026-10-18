#ifndef PATHPLANNING_NATIVE_CONTINUOUS_INTERNAL_H_
#define PATHPLANNING_NATIVE_CONTINUOUS_INTERNAL_H_

#include "continuous_collision.h"

#define PP_C_NONE UINT32_MAX
#define PP_C_KD_LEVELS 32

typedef struct pp_rit_state pp_rit_state;
typedef struct pp_jit_state pp_jit_state;

#ifdef PP_ENABLE_TRACE
typedef struct pp_trace_buffer {
    pp_trace_result *out;
    uint64_t max_bytes;
    size_t event_capacity;
    size_t point_capacity;
    size_t map_capacity;
    uint64_t *node_ids;
    uint64_t pending_sample;
    uint64_t last_solution_node;
    double last_solution_cost;
    uint32_t side;
    int restoring_initial;
    uint64_t original_id;
} pp_trace_buffer;
#endif

typedef struct {
    uint32_t *ids;
    size_t count;
} pp_kd_block;

typedef struct {
    pp_kd_block blocks[PP_C_KD_LEVELS];
    size_t dimension;
} pp_kd_index;

typedef struct {
    uint64_t state;
    uint64_t increment;
} pp_rng;

typedef struct {
    double *points;
    double *cost;
    double *edge_cost;
    uint32_t *parent;
    uint32_t *first_child;
    uint32_t *next_sibling;
    uint8_t *flags;
    uint32_t count;
    uint32_t capacity;
    size_t dimension;
#ifdef PP_ENABLE_TRACE
    pp_trace_buffer *trace;
#endif
} pp_nodes;

typedef struct {
    const pp_continuous_callbacks *callbacks;
    const pp_continuous_options *options;
    size_t dimension;
    int has_goal_point;
    const double *goal;
    pp_rng rng;
    pp_nodes nodes;
    pp_kd_index indices[2];
    pp_collision_index *collision_index;
    const pp_continuous_space_model *collision_model;
    uint64_t iterations;
    uint64_t samples;
    uint64_t batches;
    uint64_t motion_checks;
    uint64_t rewires;
    uint64_t metric_evaluations;
    uint64_t metric_updates;
    uint64_t jit_biased_samples;
    uint64_t jit_ancestor_candidates;
    uint64_t manipulability_evaluations;
    double started;
    pp_rit_state *rit;
    pp_jit_state *jit;
#ifdef PP_ENABLE_TRACE
    pp_trace_buffer *trace;
#endif
} pp_context;

double pp_now(void);
void pp_set_error(pp_continuous_result *result, const char *message);
void pp_rng_seed(pp_rng *rng, uint64_t seed);
double pp_uniform(pp_rng *rng);
double pp_distance2(const double *a, const double *b, size_t dimension);
void pp_kd_free(pp_kd_index *index);
int pp_nodes_reserve(pp_nodes *nodes, uint32_t requested);
uint32_t pp_node_add(pp_nodes *nodes, const double *state, uint32_t parent,
                     double cost, double edge_cost, uint8_t flags);
void pp_nodes_free(pp_nodes *nodes);
const double *pp_point(const pp_nodes *nodes, uint32_t id);
int pp_valid_state(pp_context *ctx, const double *state);
int pp_valid_motion(pp_context *ctx, const double *a, const double *b);
int pp_sample(pp_context *ctx, double *out);
int pp_steer(pp_context *ctx, const double *from, const double *to,
             double *out, double *distance);
int pp_nearest(pp_context *ctx, const pp_kd_index *index,
               const double *target, uint32_t *out);
int pp_add_index(pp_context *ctx, pp_kd_index *index, uint32_t id);
int pp_path_to_result(pp_context *ctx, const uint32_t *ids, size_t length,
                      double path_cost, int stop_reason,
                      pp_continuous_result *result);
size_t pp_reconstruct(const pp_nodes *nodes, uint32_t end, uint32_t *path);

#ifdef PP_ENABLE_TRACE
uint64_t pp_trace_append_point(pp_trace_buffer *trace, const double *point);
int pp_trace_append_event(pp_trace_buffer *trace, uint32_t kind, uint64_t node,
                          uint64_t parent, double value, uint32_t side);
int pp_trace_map_node(pp_trace_buffer *trace, uint32_t native_id, uint64_t id);
uint64_t pp_trace_id(const pp_trace_buffer *trace, uint32_t native_id);
void pp_trace_native_event(pp_trace_buffer *trace, uint32_t kind,
                           uint32_t node, uint32_t parent, double value,
                           uint32_t side);
void pp_trace_sample(pp_context *ctx, const double *point);
void pp_trace_phase(pp_context *ctx, double phase);
void pp_trace_init(pp_trace_buffer *buffer, pp_trace_result *out,
                   size_t dimension, uint64_t max_bytes);
void pp_trace_finish(pp_trace_buffer *buffer);
#endif

#endif
