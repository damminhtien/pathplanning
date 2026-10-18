#include "continuous_internal.h"

#include <math.h>
#include <stdlib.h>
#include <string.h>

struct pp_dynamic_rrt_session {
    pp_continuous_callbacks callbacks;
    pp_continuous_options options;
    pp_context context;
    double *start;
    double *goal;
    double waypoint_sample_rate;
};

static int pp_dynamic_return_path(pp_context *ctx, uint32_t goal_id,
                                  pp_continuous_result *result) {
    uint32_t *ids = (uint32_t *)malloc((size_t)ctx->nodes.count * sizeof(uint32_t));
    size_t length;
    int status;
    if (ids == NULL) return -1;
    length = pp_reconstruct(&ctx->nodes, goal_id, ids);
    status = length == 0 ? -1 : pp_path_to_result(ctx, ids, length,
                                                   ctx->nodes.cost[goal_id], 0,
                                                   result);
    free(ids);
    return status;
}

static int pp_dynamic_return_tree(const pp_nodes *nodes,
                                  pp_dynamic_rrt_result *result) {
    size_t point_count = (size_t)nodes->count * nodes->dimension;
    if (nodes->count == 0 || point_count > SIZE_MAX / sizeof(double)) return -1;
    result->tree_points = (double *)malloc(point_count * sizeof(double));
    result->tree_parents = (uint32_t *)malloc((size_t)nodes->count * sizeof(uint32_t));
    if (result->tree_points == NULL || result->tree_parents == NULL) return -1;
    memcpy(result->tree_points, nodes->points, point_count * sizeof(double));
    memcpy(result->tree_parents, nodes->parent, (size_t)nodes->count * sizeof(uint32_t));
    result->tree_count = nodes->count;
    return 0;
}

static int pp_dynamic_options_valid(const pp_continuous_callbacks *callbacks,
                                    const double *start, const double *goal,
                                    size_t dimension,
                                    const pp_continuous_options *options,
                                    double waypoint_sample_rate) {
    size_t axis;
    if (callbacks == NULL || start == NULL || goal == NULL || options == NULL ||
        dimension == 0 || dimension > 1024 ||
        (callbacks->native_space == NULL &&
         (callbacks->sample_free == NULL || callbacks->state_valid == NULL ||
          callbacks->motion_valid == NULL || callbacks->distance == NULL ||
          callbacks->steer == NULL)) ||
        !(waypoint_sample_rate >= 0.0 && waypoint_sample_rate <= 1.0) ||
        !(options->goal_sample_rate >= 0.0 && options->goal_sample_rate <= 1.0) ||
        options->goal_sample_rate + waypoint_sample_rate > 1.0 ||
        options->max_iters == UINT64_MAX || options->max_sample_tries == 0 ||
        !isfinite(options->step_size) || !(options->step_size > 0.0) ||
        !isfinite(options->collision_step) || !(options->collision_step > 0.0)) return 0;
    for (axis = 0; axis < dimension; ++axis) {
        if (!isfinite(start[axis]) || !isfinite(goal[axis])) return 0;
    }
    return 1;
}

int pp_dynamic_rrt_session_create(const pp_continuous_callbacks *callbacks,
                                  const double *start, const double *goal,
                                  const double *initial_points,
                                  const uint32_t *initial_parents,
                                  size_t initial_count, size_t dimension,
                                  const pp_continuous_options *options,
                                  double waypoint_sample_rate,
                                  pp_dynamic_rrt_session **out_session,
                                  pp_continuous_result *error_result) {
    pp_dynamic_rrt_session *session = NULL;
    size_t i;
    if (out_session == NULL || error_result == NULL) return -1;
    *out_session = NULL;
    memset(error_result, 0, sizeof(*error_result));
    error_result->stop_reason = 4;
    if (!pp_dynamic_options_valid(callbacks, start, goal, dimension, options,
                                  waypoint_sample_rate) ||
        initial_points == NULL || initial_parents == NULL || initial_count == 0 ||
        initial_count > UINT32_MAX - 1 || initial_count > SIZE_MAX / dimension ||
        initial_count * dimension > SIZE_MAX / sizeof(double) ||
        initial_parents[0] != PP_C_NONE ||
        pp_distance2(initial_points, start, dimension) > 1e-24) {
        pp_set_error(error_result, "dynamic RRT session inputs or options are invalid");
        return -1;
    }
    for (i = 0; i < initial_count; ++i) {
        size_t axis;
        if ((i > 0 && initial_parents[i] >= i)) {
            pp_set_error(error_result, "dynamic RRT parent ids must precede their children");
            return -1;
        }
        for (axis = 0; axis < dimension; ++axis) {
            if (!isfinite(initial_points[i * dimension + axis])) {
                pp_set_error(error_result, "dynamic RRT tree contains a non-finite state");
                return -1;
            }
        }
    }
    session = (pp_dynamic_rrt_session *)calloc(1, sizeof(*session));
    if (session == NULL) goto allocation_failure;
    session->start = (double *)malloc(dimension * sizeof(double));
    session->goal = (double *)malloc(dimension * sizeof(double));
    if (session->start == NULL || session->goal == NULL) goto allocation_failure;
    memcpy(session->start, start, dimension * sizeof(double));
    memcpy(session->goal, goal, dimension * sizeof(double));
    session->callbacks = *callbacks;
    session->options = *options;
    session->waypoint_sample_rate = waypoint_sample_rate;

    session->context.callbacks = &session->callbacks;
    session->context.options = &session->options;
    session->context.dimension = dimension;
    session->context.has_goal_point = 1;
    session->context.goal = session->goal;
    session->context.started = pp_now();
    session->context.nodes.dimension = dimension;
    session->context.indices[0].dimension = dimension;
    pp_rng_seed(&session->context.rng, options->seed);
    if (pp_nodes_reserve(&session->context.nodes, (uint32_t)initial_count) != 0)
        goto allocation_failure;
    for (i = 0; i < initial_count; ++i) {
        uint32_t id = initial_parents[i];
        double edge_cost = id == PP_C_NONE ? 0.0 :
            sqrt(pp_distance2(initial_points + (size_t)id * dimension,
                              initial_points + i * dimension, dimension));
        uint32_t added = pp_node_add(
            &session->context.nodes, initial_points + i * dimension,
            id, id == PP_C_NONE ? 0.0 : session->context.nodes.cost[id] + edge_cost,
            edge_cost, 0);
        if (added != i || pp_add_index(&session->context,
                                       &session->context.indices[0], added) != 0)
            goto allocation_failure;
    }
    *out_session = session;
    return 0;

allocation_failure:
    pp_dynamic_rrt_session_free(session);
    pp_set_error(error_result, "out of memory creating dynamic RRT session");
    return -1;
}

void pp_dynamic_rrt_session_free(pp_dynamic_rrt_session *session) {
    if (session == NULL) return;
    pp_kd_free(&session->context.indices[0]);
    pp_kd_free(&session->context.indices[1]);
    pp_nodes_free(&session->context.nodes);
    pp_collision_index_free(session->context.collision_index);
    free(session->start);
    free(session->goal);
    free(session);
}

void pp_dynamic_rrt_session_free_result(pp_dynamic_rrt_session_result *result) {
    if (result == NULL) return;
    pp_continuous_free_result(&result->plan);
    free(result->tree_points);
    free(result->tree_parents);
    free(result->invalid_nodes);
    memset(result, 0, sizeof(*result));
}

static int pp_dynamic_session_prune(pp_context *ctx,
                                    pp_dynamic_rrt_session_result *result) {
    size_t count = ctx->nodes.count, i;
    uint32_t *remap = NULL, *invalid_nodes = NULL;
    uint8_t *keep = NULL;
    pp_nodes old_nodes = ctx->nodes, next_nodes = {0};
    pp_kd_index old_index = ctx->indices[0], next_index = {0};
    int status = -1;
    if (count == 0 || count > SIZE_MAX / sizeof(uint32_t)) return -1;
    remap = (uint32_t *)malloc(count * sizeof(uint32_t));
    invalid_nodes = (uint32_t *)malloc(count * sizeof(uint32_t));
    keep = (uint8_t *)calloc(count, sizeof(uint8_t));
    if (remap == NULL || invalid_nodes == NULL || keep == NULL) goto restore;
    next_nodes.dimension = ctx->dimension;
#ifdef PP_ENABLE_TRACE
    next_nodes.trace = ctx->trace;
#endif
    next_index.dimension = ctx->dimension;
    if (pp_nodes_reserve(&next_nodes, (uint32_t)count) != 0) goto restore;
    ctx->nodes = next_nodes;
    ctx->indices[0] = next_index;

    for (i = 0; i < count; ++i) {
        uint32_t old_parent = old_nodes.parent[i];
        uint32_t parent = PP_C_NONE;
        uint32_t id;
        double edge_cost = 0.0;
        int valid = 1;
        remap[i] = PP_C_NONE;
        if (i != 0) {
            int edge_valid = pp_valid_motion(
                ctx, pp_point(&old_nodes, old_parent), pp_point(&old_nodes, (uint32_t)i));
            if (edge_valid < 0) goto restore;
            if (!edge_valid) invalid_nodes[result->invalid_count++] = (uint32_t)i;
            valid = keep[old_parent] && edge_valid;
            if (valid) {
                parent = remap[old_parent];
                edge_cost = sqrt(pp_distance2(pp_point(&old_nodes, old_parent),
                                              pp_point(&old_nodes, (uint32_t)i),
                                              ctx->dimension));
            }
        }
        if (!valid) {
#ifdef PP_ENABLE_TRACE
            if (ctx->trace != NULL && !ctx->trace->out->truncated)
                (void)pp_trace_append_event(ctx->trace, PP_TRACE_PRUNE,
                    (uint64_t)i, i == 0 ? UINT64_MAX : (uint64_t)old_parent, 0.0, 0);
#endif
            continue;
        }
        keep[i] = 1;
#ifdef PP_ENABLE_TRACE
        if (ctx->trace != NULL) {
            ctx->trace->restoring_initial = 1;
            ctx->trace->original_id = (uint64_t)i;
        }
#endif
        id = pp_node_add(&ctx->nodes, pp_point(&old_nodes, (uint32_t)i), parent,
                         i == 0 ? 0.0 : ctx->nodes.cost[parent] + edge_cost,
                         edge_cost, old_nodes.flags[i]);
#ifdef PP_ENABLE_TRACE
        if (ctx->trace != NULL) ctx->trace->restoring_initial = 0;
#endif
        if (id == PP_C_NONE || pp_add_index(ctx, &ctx->indices[0], id) != 0)
            goto restore;
        remap[i] = id;
    }
    if (ctx->nodes.count == 0) goto restore;
    pp_nodes_free(&old_nodes);
    pp_kd_free(&old_index);
    result->invalid_nodes = invalid_nodes;
    result->tree_is_snapshot = 1;
    status = 0;
    invalid_nodes = NULL;
    goto done;

restore:
    pp_nodes_free(&ctx->nodes);
    pp_kd_free(&ctx->indices[0]);
    ctx->nodes = old_nodes;
    ctx->indices[0] = old_index;
done:
    free(remap);
    free(invalid_nodes);
    free(keep);
    return status;
}

static int pp_dynamic_session_copy_tree(const pp_nodes *nodes, size_t offset,
                                        int snapshot,
                                        pp_dynamic_rrt_session_result *result) {
    size_t first = snapshot ? 0 : offset;
    size_t count;
    size_t point_count;
    if (first > nodes->count) return -1;
    count = (size_t)nodes->count - first;
    result->tree_offset = first;
    result->tree_is_snapshot = snapshot;
    result->tree_count = count;
    if (count == 0) return 0;
    if (count > SIZE_MAX / nodes->dimension ||
        count * nodes->dimension > SIZE_MAX / sizeof(double)) return -1;
    point_count = count * nodes->dimension;
    result->tree_points = (double *)malloc(point_count * sizeof(double));
    result->tree_parents = (uint32_t *)malloc(count * sizeof(uint32_t));
    if (result->tree_points == NULL || result->tree_parents == NULL) return -1;
    memcpy(result->tree_points, nodes->points + first * nodes->dimension,
           point_count * sizeof(double));
    memcpy(result->tree_parents, nodes->parent + first,
           count * sizeof(uint32_t));
    return 0;
}

static int pp_dynamic_rrt_session_run_impl(
    pp_dynamic_rrt_session *session, const pp_continuous_callbacks *callbacks,
    const double *start, const double *goal,
    const pp_continuous_options *options, double waypoint_sample_rate,
    int prune_tree, int prune_only, pp_dynamic_rrt_session_result *result
#ifdef PP_ENABLE_TRACE
    , pp_trace_buffer *trace
#endif
    ) {
    pp_context *ctx;
    size_t previous_count, i;
    uint32_t goal_id = PP_C_NONE;
    uint64_t iteration;
    double *sample = NULL, *candidate = NULL;
    int status = -1;

    if (result == NULL) return -1;
    memset(result, 0, sizeof(*result));
    result->plan.stop_reason = 4;
    if (session == NULL || (prune_tree != 0 && prune_tree != 1) ||
        (prune_only != 0 && prune_only != 1) || (prune_only && !prune_tree) ||
        !pp_dynamic_options_valid(callbacks, start, goal,
                                  session == NULL ? 0 : session->context.dimension,
                                  options, waypoint_sample_rate)) {
        pp_set_error(&result->plan, "dynamic RRT session inputs or options are invalid");
        return -1;
    }
    ctx = &session->context;
    if (pp_distance2(session->start, start, ctx->dimension) > 1e-24 ||
        pp_distance2(session->goal, goal, ctx->dimension) > 1e-24) {
        pp_set_error(&result->plan,
                     "dynamic RRT session start and goal cannot change; create a new session");
        return -1;
    }
    if (prune_tree || ctx->collision_model != callbacks->native_space) {
        pp_collision_index_free(ctx->collision_index);
        ctx->collision_index = NULL;
        ctx->collision_model = NULL;
    }
    session->callbacks = *callbacks;
    session->options = *options;
    session->waypoint_sample_rate = waypoint_sample_rate;
    ctx->callbacks = &session->callbacks;
    ctx->options = &session->options;
    ctx->goal = session->goal;
    ctx->started = pp_now();
    ctx->iterations = 0;
    ctx->samples = 0;
    ctx->batches = 0;
    ctx->motion_checks = 0;
    ctx->rewires = 0;
    ctx->metric_evaluations = 0;
    ctx->metric_updates = 0;
    ctx->jit_biased_samples = 0;
    ctx->jit_ancestor_candidates = 0;
    ctx->manipulability_evaluations = 0;
    pp_rng_seed(&ctx->rng, options->seed);
    previous_count = ctx->nodes.count;
#ifdef PP_ENABLE_TRACE
    ctx->trace = trace;
    ctx->nodes.trace = trace;
    if (trace != NULL) {
        pp_trace_phase(ctx, 0.0);
        for (i = 0; i < ctx->nodes.count && !trace->out->truncated; ++i) {
            uint64_t trace_id = pp_trace_append_point(
                trace, pp_point(&ctx->nodes, (uint32_t)i));
            uint32_t parent = ctx->nodes.parent[i];
            if (trace_id == UINT64_MAX) break;
            if (prune_tree) {
                (void)pp_trace_append_event(trace, PP_TRACE_DISCOVER, trace_id,
                    parent == PP_C_NONE ? UINT64_MAX : (uint64_t)parent, 0.0, 0);
            } else {
                (void)pp_trace_map_node(trace, (uint32_t)i, trace_id);
                (void)pp_trace_append_event(trace, PP_TRACE_DISCOVER, trace_id,
                    parent == PP_C_NONE ? UINT64_MAX : pp_trace_id(trace, parent),
                    0.0, 0);
            }
        }
        pp_trace_phase(ctx, 1.0);
    }
#endif
    if (prune_tree && pp_dynamic_session_prune(ctx, result) != 0) {
        pp_set_error(&result->plan, "failed to prune dynamic RRT session tree");
        goto done;
    }
    if (ctx->nodes.count == 0) {
        pp_set_error(&result->plan, "dynamic RRT start was removed while pruning the tree");
        goto done;
    }
    for (i = 0; i < ctx->nodes.count; ++i) {
        if (pp_distance2(pp_point(&ctx->nodes, (uint32_t)i), goal,
                         ctx->dimension) <= 1e-24) {
            int valid = pp_valid_state(ctx, pp_point(&ctx->nodes, (uint32_t)i));
            if (valid < 0) {
                pp_set_error(&result->plan,
                             "state-validity callback failed for dynamic RRT goal");
                goto done;
            }
            if (valid) {
                goal_id = (uint32_t)i;
                break;
            }
        }
    }
    if (goal_id != PP_C_NONE) {
        if (pp_dynamic_return_path(ctx, goal_id, &result->plan) != 0) {
            pp_set_error(&result->plan,
                         "failed to reconstruct existing dynamic RRT path");
            goto done;
        }
    } else if (!prune_only) {
#ifdef PP_ENABLE_TRACE
        pp_trace_phase(ctx, 2.0);
#endif
        sample = (double *)malloc(ctx->dimension * sizeof(double));
        candidate = (double *)malloc(ctx->dimension * sizeof(double));
        if (sample == NULL || candidate == NULL) {
            pp_set_error(&result->plan, "out of memory growing dynamic RRT tree");
            goto done;
        }
        for (iteration = 0; iteration <= options->max_iters; ++iteration) {
            double draw = pp_uniform(&ctx->rng);
            uint32_t nearest, duplicate, id;
            double edge_cost, goal_distance;
            int valid;
            if (draw < options->goal_sample_rate) {
                memcpy(sample, goal, ctx->dimension * sizeof(double));
            } else if (draw < options->goal_sample_rate + waypoint_sample_rate) {
                uint32_t waypoint = (uint32_t)(pp_uniform(&ctx->rng) *
                                               (double)ctx->nodes.count);
                if (waypoint >= ctx->nodes.count) waypoint = ctx->nodes.count - 1;
                memcpy(sample, pp_point(&ctx->nodes, waypoint),
                       ctx->dimension * sizeof(double));
            } else {
                valid = pp_sample(ctx, sample);
                if (valid == -1) {
                    pp_set_error(&result->plan,
                                 "sampling callback failed in dynamic RRT");
                    goto done;
                }
                if (valid == -2) {
                    pp_set_error(&result->plan,
                                 "failed to sample a free state in dynamic RRT");
                    goto done;
                }
            }
#ifdef PP_ENABLE_TRACE
            if (draw < options->goal_sample_rate + waypoint_sample_rate)
                pp_trace_sample(ctx, sample);
#endif
            if (pp_nearest(ctx, &ctx->indices[0], sample, &nearest) != 0 ||
                pp_steer(ctx, pp_point(&ctx->nodes, nearest), sample, candidate,
                         &edge_cost) != 0 ||
                pp_nearest(ctx, &ctx->indices[0], candidate, &duplicate) != 0) {
                pp_set_error(&result->plan,
                             "nearest-node query or steering failed in dynamic RRT");
                goto done;
            }
#ifdef PP_ENABLE_TRACE
            pp_trace_native_event(ctx->trace, PP_TRACE_EXPAND, nearest, PP_C_NONE,
                                  (double)ctx->iterations, 0);
#endif
            if (pp_distance2(pp_point(&ctx->nodes, duplicate), candidate,
                             ctx->dimension) <= 1e-24) {
                ++ctx->iterations;
                continue;
            }
            valid = pp_valid_motion(ctx, pp_point(&ctx->nodes, nearest), candidate);
            if (valid < 0) {
                pp_set_error(&result->plan,
                             "motion-validity callback failed in dynamic RRT");
                goto done;
            }
            if (!valid) {
                ++ctx->iterations;
                continue;
            }
            edge_cost = sqrt(pp_distance2(pp_point(&ctx->nodes, nearest), candidate,
                                          ctx->dimension));
            id = pp_node_add(&ctx->nodes, candidate, nearest,
                             ctx->nodes.cost[nearest] + edge_cost, edge_cost, 0);
            if (id == PP_C_NONE ||
                pp_add_index(ctx, &ctx->indices[0], id) != 0) {
                pp_set_error(&result->plan,
                             "out of memory growing dynamic RRT tree");
                goto done;
            }
            goal_distance = sqrt(pp_distance2(candidate, goal, ctx->dimension));
            if (goal_distance <= options->step_size) {
                valid = pp_valid_motion(ctx, candidate, goal);
                if (valid < 0) {
                    pp_set_error(&result->plan,
                                 "motion-validity callback failed near dynamic RRT goal");
                    goto done;
                }
                if (valid) {
                    if (goal_distance <= 1e-12) goal_id = id;
                    else {
                        goal_id = pp_node_add(&ctx->nodes, goal, id,
                            ctx->nodes.cost[id] + goal_distance, goal_distance, 0);
                        if (goal_id == PP_C_NONE ||
                            pp_add_index(ctx, &ctx->indices[0], goal_id) != 0) {
                            pp_set_error(&result->plan,
                                         "out of memory connecting dynamic RRT goal");
                            goto done;
                        }
                    }
                    ++ctx->iterations;
                    break;
                }
            }
            ++ctx->iterations;
        }
        if (goal_id != PP_C_NONE &&
            pp_dynamic_return_path(ctx, goal_id, &result->plan) != 0) {
            pp_set_error(&result->plan,
                         "failed to reconstruct dynamic RRT path");
            goto done;
        }
    }
    if (!result->plan.success) {
        result->plan.stop_reason = prune_only ? 3 : 2;
        result->plan.iters = ctx->iterations;
        result->plan.nodes = ctx->nodes.count;
        result->plan.sample_count = ctx->samples;
        result->plan.motion_checks = ctx->motion_checks;
        result->plan.elapsed_s = pp_now() - ctx->started;
    }
    if (pp_dynamic_session_copy_tree(&ctx->nodes, previous_count,
                                     prune_tree, result) != 0) {
        pp_set_error(&result->plan,
                     "out of memory returning dynamic RRT session tree");
        goto done;
    }
    result->plan.nodes = ctx->nodes.count;
    if (result->tree_is_snapshot) result->tree_offset = 0;
    status = 0;

done:
#ifdef PP_ENABLE_TRACE
    ctx->trace = NULL;
    ctx->nodes.trace = NULL;
#endif
    free(sample);
    free(candidate);
    if (status != 0) {
        free(result->tree_points);
        free(result->tree_parents);
        result->tree_points = NULL;
        result->tree_parents = NULL;
        result->tree_count = 0;
        free(result->invalid_nodes);
        result->invalid_nodes = NULL;
        result->invalid_count = 0;
        result->tree_is_snapshot = 0;
    }
    return status;
}

int pp_dynamic_rrt_session_run(pp_dynamic_rrt_session *session,
                               const pp_continuous_callbacks *callbacks,
                               const double *start, const double *goal,
                               const pp_continuous_options *options,
                               double waypoint_sample_rate, int prune_tree,
                               int prune_only,
                               pp_dynamic_rrt_session_result *result) {
    return pp_dynamic_rrt_session_run_impl(session, callbacks, start, goal,
        options, waypoint_sample_rate, prune_tree, prune_only, result
#ifdef PP_ENABLE_TRACE
        , NULL
#endif
        );
}

#ifdef PP_ENABLE_TRACE
int pp_dynamic_rrt_session_run_traced(
    pp_dynamic_rrt_session *session, const pp_continuous_callbacks *callbacks,
    const double *start, const double *goal,
    const pp_continuous_options *options, double waypoint_sample_rate,
    int prune_tree, int prune_only, pp_dynamic_rrt_session_result *result,
    uint64_t max_bytes, pp_trace_result *trace) {
    pp_trace_buffer buffer;
    int status;
    if (trace == NULL) return -1;
    pp_trace_init(&buffer, trace,
                  session == NULL ? 0 : session->context.dimension, max_bytes);
    status = pp_dynamic_rrt_session_run_impl(session, callbacks, start, goal,
        options, waypoint_sample_rate, prune_tree, prune_only, result, &buffer);
    pp_trace_finish(&buffer);
    return status;
}
#endif
static int pp_dynamic_rrt_plan_impl(const pp_continuous_callbacks *callbacks,
                                    const double *start, const double *goal,
                                    const double *initial_points,
                                    const uint32_t *initial_parents,
                                    size_t initial_count, size_t dimension,
                                    const pp_continuous_options *options,
                                    double waypoint_sample_rate,
                                    int prune_only,
                                    pp_dynamic_rrt_result *result
#ifdef PP_ENABLE_TRACE
                                    , pp_trace_buffer *trace
#endif
                                    ) {
    pp_context ctx;
    uint32_t *remap = NULL, *invalid_nodes = NULL, goal_id = PP_C_NONE;
    uint8_t *keep = NULL;
    double *sample = NULL, *candidate = NULL;
    size_t i;
    uint64_t iteration;
    int status = -1;

    if (result == NULL) return -1;
    memset(result, 0, sizeof(*result));
    result->plan.stop_reason = 4;
    if (callbacks == NULL || start == NULL || goal == NULL || initial_points == NULL ||
        initial_parents == NULL || initial_count == 0 || options == NULL || dimension == 0 ||
        dimension > 1024 || initial_count > UINT32_MAX - 1 ||
        (callbacks->native_space == NULL &&
         (callbacks->sample_free == NULL || callbacks->state_valid == NULL ||
          callbacks->motion_valid == NULL || callbacks->distance == NULL || callbacks->steer == NULL)) ||
        !(waypoint_sample_rate >= 0.0 && waypoint_sample_rate <= 1.0) ||
        (prune_only != 0 && prune_only != 1) ||
        !(options->goal_sample_rate >= 0.0 && options->goal_sample_rate <= 1.0) ||
        options->goal_sample_rate + waypoint_sample_rate > 1.0 ||
        options->max_iters == UINT64_MAX || options->max_sample_tries == 0 ||
        !isfinite(options->step_size) || !(options->step_size > 0.0) ||
        !isfinite(options->collision_step) || !(options->collision_step > 0.0) ||
        initial_count > SIZE_MAX / dimension ||
        initial_count * dimension > SIZE_MAX / sizeof(double)) {
        pp_set_error(&result->plan, "dynamic RRT inputs or options are invalid");
        return -1;
    }
    if (initial_parents[0] != PP_C_NONE ||
        pp_distance2(initial_points, start, dimension) > 1e-24) {
        pp_set_error(&result->plan, "dynamic RRT tree root must match its start state");
        return -1;
    }
    for (i = 0; i < initial_count; ++i) {
        size_t axis;
        if (i > 0 && initial_parents[i] >= i) {
            pp_set_error(&result->plan, "dynamic RRT parent ids must precede their children");
            return -1;
        }
        for (axis = 0; axis < dimension; ++axis) {
            if (!isfinite(initial_points[i * dimension + axis])) {
                pp_set_error(&result->plan, "dynamic RRT tree contains a non-finite state");
                return -1;
            }
        }
    }

    memset(&ctx, 0, sizeof(ctx));
    ctx.callbacks = callbacks;
    ctx.options = options;
    ctx.dimension = dimension;
    ctx.has_goal_point = 1;
    ctx.goal = goal;
    ctx.started = pp_now();
    ctx.nodes.dimension = dimension;
#ifdef PP_ENABLE_TRACE
    ctx.trace = trace;
    ctx.nodes.trace = trace;
#endif
    ctx.indices[0].dimension = dimension;
    pp_rng_seed(&ctx.rng, options->seed);
#ifdef PP_ENABLE_TRACE
    if (ctx.trace != NULL) {
        pp_trace_phase(&ctx, 0.0);  // Caller-supplied tree before pruning.
        for (i = 0; i < initial_count && !ctx.trace->out->truncated; ++i) {
            uint64_t id = pp_trace_append_point(ctx.trace, initial_points + i * dimension);
            if (id == UINT64_MAX) break;
            (void)pp_trace_append_event(ctx.trace, PP_TRACE_DISCOVER, id,
                i == 0 ? UINT64_MAX : (uint64_t)initial_parents[i], 0.0, 0);
        }
        pp_trace_phase(&ctx, 1.0);  // Prune invalid edges and descendants.
    }
#endif
    remap = (uint32_t *)malloc(initial_count * sizeof(uint32_t));
    keep = (uint8_t *)calloc(initial_count, sizeof(uint8_t));
    invalid_nodes = (uint32_t *)malloc(initial_count * sizeof(uint32_t));
    sample = (double *)malloc(dimension * sizeof(double));
    candidate = (double *)malloc(dimension * sizeof(double));
    if (remap == NULL || keep == NULL || invalid_nodes == NULL || sample == NULL || candidate == NULL ||
        pp_nodes_reserve(&ctx.nodes, (uint32_t)initial_count) != 0) {
        pp_set_error(&result->plan, "out of memory initializing dynamic RRT");
        goto done;
    }
    for (i = 0; i < initial_count; ++i) remap[i] = PP_C_NONE;
    for (i = 0; i < initial_count; ++i) {
        uint32_t old_parent = initial_parents[i], parent = PP_C_NONE, id;
        double edge_cost = 0.0;
        int valid = 1;
        if (i != 0) {
            int edge_valid = pp_valid_motion(
                &ctx, initial_points + (size_t)old_parent * dimension,
                initial_points + i * dimension);
            if (edge_valid < 0) {
                pp_set_error(&result->plan, "motion-validity callback failed while pruning tree");
                goto done;
            }
            if (!edge_valid) invalid_nodes[result->invalid_count++] = (uint32_t)i;
            valid = keep[old_parent] && edge_valid;
            if (valid) {
                parent = remap[old_parent];
                edge_cost = sqrt(pp_distance2(
                    initial_points + (size_t)old_parent * dimension,
                    initial_points + i * dimension, dimension));
            }
        }
        if (!valid) {
#ifdef PP_ENABLE_TRACE
            if (ctx.trace != NULL && !ctx.trace->out->truncated)
                (void)pp_trace_append_event(ctx.trace, PP_TRACE_PRUNE,
                    (uint64_t)i, i == 0 ? UINT64_MAX : (uint64_t)old_parent, 0.0, 0);
#endif
            continue;
        }
        keep[i] = 1;
#ifdef PP_ENABLE_TRACE
        if (ctx.trace != NULL) {
            ctx.trace->restoring_initial = 1;
            ctx.trace->original_id = (uint64_t)i;
        }
#endif
        id = pp_node_add(&ctx.nodes, initial_points + i * dimension, parent,
                         i == 0 ? 0.0 : ctx.nodes.cost[parent] + edge_cost,
                         edge_cost, 0);
#ifdef PP_ENABLE_TRACE
        if (ctx.trace != NULL) ctx.trace->restoring_initial = 0;
#endif
        if (id == PP_C_NONE || pp_add_index(&ctx, &ctx.indices[0], id) != 0) {
            pp_set_error(&result->plan, "out of memory restoring dynamic RRT tree");
            goto done;
        }
        remap[i] = id;
    }
    if (ctx.nodes.count == 0) {
        pp_set_error(&result->plan, "dynamic RRT start was removed while pruning the tree");
        goto done;
    }
    for (i = 0; i < ctx.nodes.count; ++i) {
        if (pp_distance2(pp_point(&ctx.nodes, (uint32_t)i), goal, dimension) <= 1e-24) {
            int valid = pp_valid_state(&ctx, pp_point(&ctx.nodes, (uint32_t)i));
            if (valid < 0) {
                pp_set_error(&result->plan, "state-validity callback failed for dynamic RRT goal");
                goto done;
            }
            if (valid) {
                goal_id = (uint32_t)i;
                break;
            }
        }
    }
    if (goal_id != PP_C_NONE) {
        if (pp_dynamic_return_path(&ctx, goal_id, &result->plan) != 0) {
            pp_set_error(&result->plan, "failed to reconstruct existing dynamic RRT path");
            goto done;
        }
    } else if (!prune_only) {
#ifdef PP_ENABLE_TRACE
        pp_trace_phase(&ctx, 2.0);  // Regrowth after the pruned snapshot.
#endif
        for (iteration = 0; iteration <= options->max_iters; ++iteration) {
            double draw = pp_uniform(&ctx.rng);
            uint32_t nearest, duplicate, id;
            double edge_cost, goal_distance;
            int valid;
            if (draw < options->goal_sample_rate) {
                memcpy(sample, goal, dimension * sizeof(double));
            } else if (draw < options->goal_sample_rate + waypoint_sample_rate) {
                uint32_t waypoint = (uint32_t)(pp_uniform(&ctx.rng) * (double)ctx.nodes.count);
                if (waypoint >= ctx.nodes.count) waypoint = ctx.nodes.count - 1;
                memcpy(sample, pp_point(&ctx.nodes, waypoint), dimension * sizeof(double));
            } else {
                valid = pp_sample(&ctx, sample);
                if (valid == -1) {
                    pp_set_error(&result->plan, "sampling callback failed in dynamic RRT");
                    goto done;
                }
                if (valid == -2) {
                    pp_set_error(&result->plan, "failed to sample a free state in dynamic RRT");
                    goto done;
                }
            }
#ifdef PP_ENABLE_TRACE
            if (draw < options->goal_sample_rate + waypoint_sample_rate)
                pp_trace_sample(&ctx, sample);
#endif
            if (pp_nearest(&ctx, &ctx.indices[0], sample, &nearest) != 0 ||
                pp_steer(&ctx, pp_point(&ctx.nodes, nearest), sample, candidate, &edge_cost) != 0 ||
                pp_nearest(&ctx, &ctx.indices[0], candidate, &duplicate) != 0) {
                pp_set_error(&result->plan, "nearest-node query or steering failed in dynamic RRT");
                goto done;
            }
#ifdef PP_ENABLE_TRACE
            pp_trace_native_event(ctx.trace, PP_TRACE_EXPAND, nearest, PP_C_NONE,
                                  (double)ctx.iterations, 0);
#endif
            if (pp_distance2(pp_point(&ctx.nodes, duplicate), candidate, dimension) <= 1e-24) {
                ++ctx.iterations;
                continue;
            }
            valid = pp_valid_motion(&ctx, pp_point(&ctx.nodes, nearest), candidate);
            if (valid < 0) {
                pp_set_error(&result->plan, "motion-validity callback failed in dynamic RRT");
                goto done;
            }
            if (!valid) {
                ++ctx.iterations;
                continue;
            }
            edge_cost = sqrt(pp_distance2(pp_point(&ctx.nodes, nearest), candidate, dimension));
            id = pp_node_add(&ctx.nodes, candidate, nearest,
                             ctx.nodes.cost[nearest] + edge_cost, edge_cost, 0);
            if (id == PP_C_NONE || pp_add_index(&ctx, &ctx.indices[0], id) != 0) {
                pp_set_error(&result->plan, "out of memory growing dynamic RRT tree");
                goto done;
            }
            goal_distance = sqrt(pp_distance2(candidate, goal, dimension));
            if (goal_distance <= options->step_size) {
                valid = pp_valid_motion(&ctx, candidate, goal);
                if (valid < 0) {
                    pp_set_error(&result->plan, "motion-validity callback failed near dynamic RRT goal");
                    goto done;
                }
                if (valid) {
                    if (goal_distance <= 1e-12) goal_id = id;
                    else {
                        goal_id = pp_node_add(&ctx.nodes, goal, id,
                            ctx.nodes.cost[id] + goal_distance, goal_distance, 0);
                        if (goal_id == PP_C_NONE ||
                            pp_add_index(&ctx, &ctx.indices[0], goal_id) != 0) {
                            pp_set_error(&result->plan, "out of memory connecting dynamic RRT goal");
                            goto done;
                        }
                    }
                    ++ctx.iterations;
                    break;
                }
            }
            ++ctx.iterations;
        }
        if (goal_id != PP_C_NONE &&
            pp_dynamic_return_path(&ctx, goal_id, &result->plan) != 0) {
            pp_set_error(&result->plan, "failed to reconstruct dynamic RRT path");
            goto done;
        }
    }
    if (!result->plan.success) {
        result->plan.stop_reason = prune_only ? 3 : 2;
        result->plan.iters = ctx.iterations;
        result->plan.nodes = ctx.nodes.count;
        result->plan.sample_count = ctx.samples;
        result->plan.motion_checks = ctx.motion_checks;
        result->plan.elapsed_s = pp_now() - ctx.started;
    }
    if (pp_dynamic_return_tree(&ctx.nodes, result) != 0) {
        pp_set_error(&result->plan, "out of memory returning dynamic RRT tree");
        goto done;
    }
    result->invalid_nodes = invalid_nodes;
    invalid_nodes = NULL;
    status = 0;

done:
    pp_kd_free(&ctx.indices[0]);
    pp_kd_free(&ctx.indices[1]);
    pp_nodes_free(&ctx.nodes);
    pp_collision_index_free(ctx.collision_index);
    free(remap);
    free(keep);
    free(invalid_nodes);
    free(sample);
    free(candidate);
    if (status != 0) {
        free(result->tree_points);
        free(result->tree_parents);
        free(result->invalid_nodes);
        result->tree_points = NULL;
        result->tree_parents = NULL;
        result->invalid_nodes = NULL;
        result->tree_count = 0;
        result->invalid_count = 0;
    }
    return status;
}

int pp_dynamic_rrt_plan(const pp_continuous_callbacks *callbacks,
                        const double *start, const double *goal,
                        const double *initial_points,
                        const uint32_t *initial_parents,
                        size_t initial_count, size_t dimension,
                        const pp_continuous_options *options,
                        double waypoint_sample_rate,
                        int prune_only,
                        pp_dynamic_rrt_result *result) {
    return pp_dynamic_rrt_plan_impl(callbacks, start, goal, initial_points,
                                     initial_parents, initial_count, dimension,
                                     options, waypoint_sample_rate, prune_only, result
#ifdef PP_ENABLE_TRACE
                                     , NULL
#endif
                                     );
}

#ifdef PP_ENABLE_TRACE
int pp_dynamic_rrt_plan_traced(const pp_continuous_callbacks *callbacks,
                               const double *start, const double *goal,
                               const double *initial_points,
                               const uint32_t *initial_parents,
                               size_t initial_count, size_t dimension,
                               const pp_continuous_options *options,
                               double waypoint_sample_rate, int prune_only,
                               pp_dynamic_rrt_result *result,
                               uint64_t max_bytes, pp_trace_result *trace) {
    pp_trace_buffer buffer;
    int status;
    if (trace == NULL) return -1;
    pp_trace_init(&buffer, trace, dimension, max_bytes);
    status = pp_dynamic_rrt_plan_impl(callbacks, start, goal, initial_points,
                                      initial_parents, initial_count, dimension,
                                      options, waypoint_sample_rate, prune_only,
                                      result, &buffer);
    pp_trace_finish(&buffer);
    return status;
}

uint32_t pp_continuous_trace_abi_version(void) { return PP_CONTINUOUS_TRACE_ABI_VERSION; }

void pp_continuous_trace_free_result(pp_trace_result *trace) {
    if (trace == NULL) return;
    free(trace->events);
    free(trace->points);
    memset(trace, 0, sizeof(*trace));
}
#endif
