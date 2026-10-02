#if !defined(_WIN32)
#define _POSIX_C_SOURCE 200809L
#endif

#include "continuous_engine.h"

#include <math.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>

#define PP_C_INF INFINITY
#define PP_C_NONE UINT32_MAX
#define PP_C_VERSION "1.0.0"
#define PP_C_KD_LEVELS 32

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
} pp_nodes;

typedef struct {
    uint32_t *items;
    size_t count;
    size_t capacity;
} pp_ids;

typedef struct {
    uint32_t source;
    uint32_t target;
    double key;
    double source_cost;
    double tentative;
    uint64_t order;
} pp_edge_entry;

typedef struct {
    pp_edge_entry *items;
    size_t count;
    size_t capacity;
    uint64_t next_order;
} pp_edge_heap;

typedef struct {
    const pp_continuous_callbacks *callbacks;
    const pp_continuous_options *options;
    size_t dimension;
    int has_goal_point;
    const double *goal;
    pp_rng rng;
    pp_nodes nodes;
    pp_kd_index indices[2];
    uint64_t iterations;
    uint64_t samples;
    uint64_t batches;
    uint64_t motion_checks;
    uint64_t rewires;
    double started;
} pp_context;

static double pp_now(void) {
    struct timespec value;
#if defined(_WIN32)
    if (timespec_get(&value, TIME_UTC) != TIME_UTC) return (double)clock() / CLOCKS_PER_SEC;
#else
    if (clock_gettime(CLOCK_MONOTONIC, &value) != 0) return (double)clock() / CLOCKS_PER_SEC;
#endif
    return (double)value.tv_sec + (double)value.tv_nsec * 1e-9;
}

static void pp_set_error(pp_continuous_result *result, const char *message) {
    size_t length;
    if (result == NULL || message == NULL) return;
    free(result->error_message);
    length = strlen(message);
    result->error_message = (char *)malloc(length + 1);
    if (result->error_message != NULL) memcpy(result->error_message, message, length + 1);
    result->success = 0;
    result->stop_reason = 3;
}

static uint32_t pp_rng_u32(pp_rng *rng) {
    uint64_t old = rng->state;
    uint32_t xorshifted;
    uint32_t rotation;
    rng->state = old * UINT64_C(6364136223846793005) + rng->increment;
    xorshifted = (uint32_t)(((old >> 18u) ^ old) >> 27u);
    rotation = (uint32_t)(old >> 59u);
    return (xorshifted >> rotation) | (xorshifted << ((-(int32_t)rotation) & 31));
}

static void pp_rng_seed(pp_rng *rng, uint64_t seed) {
    rng->state = 0;
    rng->increment = (seed << 1u) | 1u;
    (void)pp_rng_u32(rng);
    rng->state += seed ^ UINT64_C(0x9e3779b97f4a7c15);
    (void)pp_rng_u32(rng);
}

static double pp_uniform(pp_rng *rng) {
    return ((double)pp_rng_u32(rng) + 0.5) / 4294967296.0;
}

static double pp_distance2(const double *a, const double *b, size_t dimension) {
    double total = 0.0;
    size_t i;
    for (i = 0; i < dimension; ++i) {
        double delta = a[i] - b[i];
        total += delta * delta;
    }
    return total;
}

static int pp_native_state_valid(const pp_continuous_space_model *model,
                                 const double *state, size_t dimension) {
    size_t i, obstacle;
    if (model == NULL || model->lower_bounds == NULL || model->upper_bounds == NULL) return 0;
    if ((model->box_count != 0 && (model->box_minima == NULL || model->box_maxima == NULL)) ||
        (model->sphere_count != 0 && (model->sphere_centers == NULL || model->sphere_radii == NULL)) ||
        (model->obb_count != 0 && (model->obb_centers == NULL || model->obb_extents == NULL ||
                                   model->obb_orientations == NULL))) return 0;
    for (i = 0; i < dimension; ++i)
        if (!isfinite(state[i]) || state[i] < model->lower_bounds[i] ||
            state[i] > model->upper_bounds[i]) return 0;
    for (obstacle = 0; obstacle < model->box_count; ++obstacle) {
        int inside = 1;
        for (i = 0; i < dimension; ++i) {
            size_t at = obstacle * dimension + i;
            if (state[i] < model->box_minima[at] || state[i] > model->box_maxima[at]) { inside = 0; break; }
        }
        if (inside) return 0;
    }
    for (obstacle = 0; obstacle < model->sphere_count; ++obstacle) {
        double radius = model->sphere_radii[obstacle];
        double distance2 = 0.0;
        for (i = 0; i < dimension; ++i) {
            double delta = state[i] - model->sphere_centers[obstacle * dimension + i];
            distance2 += delta * delta;
        }
        if (distance2 <= radius * radius) return 0;
    }
    if (model->obb_count != 0 && dimension != 3) return 0;
    for (obstacle = 0; obstacle < model->obb_count; ++obstacle) {
        size_t axis, row;
        int inside = 1;
        for (axis = 0; axis < 3; ++axis) {
            double local = 0.0;
            for (row = 0; row < 3; ++row) {
                size_t center_at = obstacle * 3 + row;
                size_t orientation_at = obstacle * 9 + row * 3 + axis;
                local += model->obb_orientations[orientation_at] * (state[row] - model->obb_centers[center_at]);
            }
            if (fabs(local) > model->obb_extents[obstacle * 3 + axis]) { inside = 0; break; }
        }
        if (inside) return 0;
    }
    return 1;
}

static int pp_id_less(uint32_t a, uint32_t b, const double *points,
                      size_t dimension, size_t axis) {
    double av = points[(size_t)a * dimension + axis];
    double bv = points[(size_t)b * dimension + axis];
    return av < bv || (av == bv && a < b);
}

static void pp_select_median(uint32_t *ids, ptrdiff_t low, ptrdiff_t high,
                             ptrdiff_t nth, const double *points,
                             size_t dimension, size_t axis) {
    while (low < high) {
        uint32_t pivot = ids[low + (high - low) / 2];
        ptrdiff_t left = low, right = high;
        while (left <= right) {
            while (pp_id_less(ids[left], pivot, points, dimension, axis)) ++left;
            while (pp_id_less(pivot, ids[right], points, dimension, axis)) --right;
            if (left <= right) {
                uint32_t temp = ids[left]; ids[left] = ids[right]; ids[right] = temp;
                ++left; --right;
            }
        }
        if (nth <= right) high = right;
        else if (nth >= left) low = left;
        else return;
    }
}

static void pp_kd_build(uint32_t *ids, ptrdiff_t low, ptrdiff_t high,
                        const double *points, size_t dimension, size_t depth) {
    ptrdiff_t middle;
    size_t axis;
    if (low >= high) return;
    middle = low + (high - low) / 2;
    axis = depth % dimension;
    pp_select_median(ids, low, high, middle, points, dimension, axis);
    pp_kd_build(ids, low, middle - 1, points, dimension, depth + 1);
    pp_kd_build(ids, middle + 1, high, points, dimension, depth + 1);
}

static void pp_kd_collect(const pp_kd_block *block, ptrdiff_t low,
                          ptrdiff_t high, uint32_t *out, size_t *cursor) {
    ptrdiff_t middle;
    if (low > high) return;
    middle = low + (high - low) / 2;
    out[(*cursor)++] = block->ids[middle];
    pp_kd_collect(block, low, middle - 1, out, cursor);
    pp_kd_collect(block, middle + 1, high, out, cursor);
}

static int pp_kd_insert(pp_kd_index *index, uint32_t id, const double *points) {
    size_t level = 0;
    uint32_t carry = id;
    while (level < PP_C_KD_LEVELS) {
        pp_kd_block *block = &index->blocks[level];
        if (block->count == 0) {
            block->ids = (uint32_t *)malloc(sizeof(uint32_t));
            if (block->ids == NULL) return -1;
            block->ids[0] = carry; block->count = 1;
            pp_kd_build(block->ids, 0, 0, points, index->dimension, 0);
            return 0;
        }
        if (block->count > SIZE_MAX / (2 * sizeof(uint32_t))) return -1;
        {
            size_t merged_count = block->count * 2;
            uint32_t *merged = (uint32_t *)malloc(merged_count * sizeof(uint32_t));
            size_t cursor = 0;
            if (merged == NULL) return -1;
            pp_kd_collect(block, 0, (ptrdiff_t)block->count - 1, merged, &cursor);
            merged[cursor++] = carry;
            free(block->ids); block->ids = NULL; block->count = 0;
            if (cursor != merged_count) { free(merged); return -1; }
            pp_kd_build(merged, 0, (ptrdiff_t)merged_count - 1, points, index->dimension, 0);
            carry = (uint32_t)merged_count;
            ++level;
            while (level < PP_C_KD_LEVELS && index->blocks[level].count != 0) {
                pp_kd_block *next_block = &index->blocks[level];
                size_t count = (size_t)carry + next_block->count;
                uint32_t *joined;
                cursor = 0;
                if (count > SIZE_MAX / sizeof(uint32_t)) { free(merged); return -1; }
                joined = (uint32_t *)malloc(count * sizeof(uint32_t));
                if (joined == NULL) { free(merged); return -1; }
                memcpy(joined, merged, (size_t)carry * sizeof(uint32_t));
                cursor = (size_t)carry;
                pp_kd_collect(next_block, 0, (ptrdiff_t)next_block->count - 1, joined, &cursor);
                free(next_block->ids); next_block->ids = NULL; next_block->count = 0;
                free(merged); merged = joined; carry = (uint32_t)count;
                pp_kd_build(merged, 0, (ptrdiff_t)count - 1, points, index->dimension, 0);
                ++level;
            }
            if (level >= PP_C_KD_LEVELS) { free(merged); return -1; }
            index->blocks[level].ids = merged;
            index->blocks[level].count = carry;
            return 0;
        }
    }
    return -1;
}

static void pp_kd_free(pp_kd_index *index) {
    size_t i;
    for (i = 0; i < PP_C_KD_LEVELS; ++i) { free(index->blocks[i].ids); index->blocks[i].ids = NULL; index->blocks[i].count = 0; }
}

static void pp_kd_nearest_block(const uint32_t *ids, ptrdiff_t low, ptrdiff_t high,
                                size_t depth, const double *query, const double *points,
                                size_t dimension, uint32_t *best_id, double *best_distance) {
    ptrdiff_t middle;
    size_t axis;
    uint32_t id;
    double distance, delta;
    ptrdiff_t near_low, near_high, far_low, far_high;
    if (low > high) return;
    middle = low + (high - low) / 2; axis = depth % dimension; id = ids[middle];
    distance = pp_distance2(query, points + (size_t)id * dimension, dimension);
    if (distance < *best_distance || (distance == *best_distance && id < *best_id)) { *best_id = id; *best_distance = distance; }
    delta = query[axis] - points[(size_t)id * dimension + axis];
    if (delta <= 0.0) { near_low = low; near_high = middle - 1; far_low = middle + 1; far_high = high; }
    else { near_low = middle + 1; near_high = high; far_low = low; far_high = middle - 1; }
    pp_kd_nearest_block(ids, near_low, near_high, depth + 1, query, points, dimension, best_id, best_distance);
    if (delta * delta <= *best_distance) pp_kd_nearest_block(ids, far_low, far_high, depth + 1, query, points, dimension, best_id, best_distance);
}

static int pp_kd_nearest(const pp_kd_index *index, const double *query,
                         const double *points, uint32_t *out_id) {
    double best = PP_C_INF;
    uint32_t best_id = PP_C_NONE;
    size_t i;
    for (i = 0; i < PP_C_KD_LEVELS; ++i) {
        const pp_kd_block *block = &index->blocks[i];
        if (block->count) pp_kd_nearest_block(block->ids, 0, (ptrdiff_t)block->count - 1,
                                             0, query, points, index->dimension, &best_id, &best);
    }
    if (best_id == PP_C_NONE) return -1;
    *out_id = best_id;
    return 0;
}

static int pp_ids_append(pp_ids *ids, uint32_t value) {
    if (ids->count == ids->capacity) {
        size_t capacity = ids->capacity == 0 ? 16 : ids->capacity * 2;
        uint32_t *grown;
        if (capacity < ids->capacity || capacity > SIZE_MAX / sizeof(uint32_t)) return -1;
        grown = (uint32_t *)realloc(ids->items, capacity * sizeof(uint32_t));
        if (grown == NULL) return -1;
        ids->items = grown; ids->capacity = capacity;
    }
    ids->items[ids->count++] = value;
    return 0;
}

static int pp_kd_radius_block(const uint32_t *ids, ptrdiff_t low, ptrdiff_t high,
                              size_t depth, const double *query, double radius2,
                              const double *points, size_t dimension, pp_ids *found) {
    ptrdiff_t middle;
    size_t axis;
    uint32_t id;
    double delta;
    if (low > high) return 0;
    middle = low + (high - low) / 2; axis = depth % dimension; id = ids[middle];
    if (pp_distance2(query, points + (size_t)id * dimension, dimension) <= radius2 &&
        pp_ids_append(found, id) != 0) return -1;
    delta = query[axis] - points[(size_t)id * dimension + axis];
    if (delta <= 0.0) {
        if (pp_kd_radius_block(ids, low, middle - 1, depth + 1, query, radius2, points, dimension, found) != 0) return -1;
        if (delta * delta <= radius2 && pp_kd_radius_block(ids, middle + 1, high, depth + 1, query, radius2, points, dimension, found) != 0) return -1;
    } else {
        if (pp_kd_radius_block(ids, middle + 1, high, depth + 1, query, radius2, points, dimension, found) != 0) return -1;
        if (delta * delta <= radius2 && pp_kd_radius_block(ids, low, middle - 1, depth + 1, query, radius2, points, dimension, found) != 0) return -1;
    }
    return 0;
}

static int pp_kd_radius(const pp_kd_index *index, const double *query, double radius,
                        const double *points, pp_ids *found) {
    size_t i;
    double radius2 = radius * radius;
    for (i = 0; i < PP_C_KD_LEVELS; ++i) {
        const pp_kd_block *block = &index->blocks[i];
        if (block->count && pp_kd_radius_block(block->ids, 0, (ptrdiff_t)block->count - 1,
                                              0, query, radius2, points, index->dimension, found) != 0) return -1;
    }
    return 0;
}

static int pp_nodes_reserve(pp_nodes *nodes, uint32_t requested) {
    uint32_t capacity;
    double *points, *cost, *edge_cost;
    uint32_t *parent, *first_child, *next_sibling;
    uint8_t *flags;
    if (requested <= nodes->capacity) return 0;
    if (requested > UINT32_MAX - 1) return -1;
    capacity = nodes->capacity == 0 ? 64 : nodes->capacity;
    while (capacity < requested) {
        if (capacity > (UINT32_MAX - 1) / 2) { capacity = requested; break; }
        capacity *= 2;
    }
    if ((size_t)capacity > SIZE_MAX / nodes->dimension / sizeof(double)) return -1;
    points = (double *)realloc(nodes->points, (size_t)capacity * nodes->dimension * sizeof(double));
    if (points == NULL) return -1; nodes->points = points;
    cost = (double *)realloc(nodes->cost, (size_t)capacity * sizeof(double));
    if (cost == NULL) return -1; nodes->cost = cost;
    edge_cost = (double *)realloc(nodes->edge_cost, (size_t)capacity * sizeof(double));
    if (edge_cost == NULL) return -1; nodes->edge_cost = edge_cost;
    parent = (uint32_t *)realloc(nodes->parent, (size_t)capacity * sizeof(uint32_t));
    if (parent == NULL) return -1; nodes->parent = parent;
    first_child = (uint32_t *)realloc(nodes->first_child, (size_t)capacity * sizeof(uint32_t));
    if (first_child == NULL) return -1; nodes->first_child = first_child;
    next_sibling = (uint32_t *)realloc(nodes->next_sibling, (size_t)capacity * sizeof(uint32_t));
    if (next_sibling == NULL) return -1; nodes->next_sibling = next_sibling;
    flags = (uint8_t *)realloc(nodes->flags, (size_t)capacity * sizeof(uint8_t));
    if (flags == NULL) return -1; nodes->flags = flags;
    nodes->capacity = capacity;
    return 0;
}

static uint32_t pp_node_add(pp_nodes *nodes, const double *state, uint32_t parent,
                            double cost, double edge_cost, uint8_t flags) {
    uint32_t id;
    if (nodes->count == UINT32_MAX || pp_nodes_reserve(nodes, nodes->count + 1) != 0) return PP_C_NONE;
    id = nodes->count++;
    memcpy(nodes->points + (size_t)id * nodes->dimension, state, nodes->dimension * sizeof(double));
    nodes->cost[id] = cost; nodes->edge_cost[id] = edge_cost;
    nodes->parent[id] = parent; nodes->first_child[id] = PP_C_NONE;
    nodes->next_sibling[id] = PP_C_NONE; nodes->flags[id] = flags;
    if (parent != PP_C_NONE) {
        nodes->next_sibling[id] = nodes->first_child[parent];
        nodes->first_child[parent] = id;
    }
    return id;
}

static void pp_node_detach(pp_nodes *nodes, uint32_t id) {
    uint32_t parent = nodes->parent[id], current, previous = PP_C_NONE;
    if (parent == PP_C_NONE) return;
    current = nodes->first_child[parent];
    while (current != PP_C_NONE) {
        if (current == id) {
            if (previous == PP_C_NONE) nodes->first_child[parent] = nodes->next_sibling[current];
            else nodes->next_sibling[previous] = nodes->next_sibling[current];
            break;
        }
        previous = current; current = nodes->next_sibling[current];
    }
    nodes->parent[id] = PP_C_NONE; nodes->next_sibling[id] = PP_C_NONE;
}

static void pp_node_attach(pp_nodes *nodes, uint32_t id, uint32_t parent,
                           double edge_cost, double cost) {
    pp_node_detach(nodes, id);
    nodes->parent[id] = parent; nodes->edge_cost[id] = edge_cost; nodes->cost[id] = cost;
    nodes->next_sibling[id] = nodes->first_child[parent]; nodes->first_child[parent] = id;
}

static void pp_nodes_free(pp_nodes *nodes) {
    free(nodes->points); free(nodes->cost); free(nodes->edge_cost); free(nodes->parent);
    free(nodes->first_child); free(nodes->next_sibling); free(nodes->flags); memset(nodes, 0, sizeof(*nodes));
}

static const double *pp_point(const pp_nodes *nodes, uint32_t id) {
    return nodes->points + (size_t)id * nodes->dimension;
}

static int pp_distance(pp_context *ctx, const double *a, const double *b, double *out) {
    double value = ctx->callbacks->native_space != NULL
        ? sqrt(pp_distance2(a, b, ctx->dimension))
        : ctx->callbacks->distance(ctx->callbacks->user_data, a, b, ctx->dimension);
    if (!isfinite(value) || value < 0.0) return -1;
    *out = value; return 0;
}

static int pp_valid_state(pp_context *ctx, const double *state) {
    int valid = ctx->callbacks->native_space != NULL
        ? pp_native_state_valid(ctx->callbacks->native_space, state, ctx->dimension)
        : ctx->callbacks->state_valid(ctx->callbacks->user_data, state, ctx->dimension);
    return valid == 0 || valid == 1 ? valid : -1;
}

static int pp_valid_motion(pp_context *ctx, const double *a, const double *b) {
    if (ctx->callbacks->native_space != NULL) {
        double distance = sqrt(pp_distance2(a, b, ctx->dimension));
        uint64_t steps, i;
        double sample[1024];
        ++ctx->motion_checks;
        if (distance == 0.0) return pp_valid_state(ctx, a);
        steps = (uint64_t)ceil(distance / ctx->options->collision_step);
        if (steps == 0) steps = 1;
        if (ctx->dimension > sizeof(sample) / sizeof(sample[0])) return -1;
        for (i = 0; i <= steps; ++i) {
            size_t axis;
            double alpha = (double)i / (double)steps;
            for (axis = 0; axis < ctx->dimension; ++axis) sample[axis] = a[axis] + alpha * (b[axis] - a[axis]);
            int valid = pp_valid_state(ctx, sample);
            if (valid < 0) return -1;
            if (!valid) return 0;
        }
        return 1;
    }
    ++ctx->motion_checks;
    {
        int valid = ctx->callbacks->motion_valid(ctx->callbacks->user_data, a, b,
                                                ctx->dimension,
                                                ctx->options->collision_step);
        return valid == 0 || valid == 1 ? valid : -1;
    }
}

static int pp_sample(pp_context *ctx, double *out) {
    uint64_t attempt;
    for (attempt = 0; attempt < ctx->options->max_sample_tries; ++attempt) {
        int status;
        if (ctx->callbacks->native_space != NULL) {
            size_t i;
            const pp_continuous_space_model *model = ctx->callbacks->native_space;
            for (i = 0; i < ctx->dimension; ++i)
                out[i] = model->lower_bounds[i] + pp_uniform(&ctx->rng) * (model->upper_bounds[i] - model->lower_bounds[i]);
            status = pp_native_state_valid(model, out, ctx->dimension);
        } else {
            status = ctx->callbacks->sample_free(ctx->callbacks->user_data, out, ctx->dimension);
            if (status != 0) return -1;
            status = pp_valid_state(ctx, out);
        }
        if (status < 0) return -1;
        if (status > 0) { ++ctx->samples; return 0; }
    }
    return -2;
}

static int pp_steer(pp_context *ctx, const double *from, const double *to, double *out,
                    double *edge_distance) {
    double distance;
    if (pp_distance(ctx, from, to, &distance) != 0) return -1;
    *edge_distance = distance;
    if (ctx->callbacks->native_space != NULL) {
        size_t i;
        double scale = distance <= ctx->options->step_size || distance == 0.0
            ? 1.0 : ctx->options->step_size / distance;
        for (i = 0; i < ctx->dimension; ++i) out[i] = from[i] + (to[i] - from[i]) * scale;
    } else if (ctx->callbacks->steer != NULL) {
        if (ctx->callbacks->steer(ctx->callbacks->user_data, from, to, ctx->dimension,
                                  ctx->options->step_size, out) != 0) return -1;
    } else if (distance <= ctx->options->step_size || distance == 0.0) {
        memcpy(out, to, ctx->dimension * sizeof(double));
    } else {
        size_t i;
        double scale = ctx->options->step_size / distance;
        for (i = 0; i < ctx->dimension; ++i) out[i] = from[i] + (to[i] - from[i]) * scale;
    }
    return pp_distance(ctx, from, out, edge_distance);
}

static int pp_is_goal(pp_context *ctx, const double *state) {
    if (ctx->callbacks->native_goal)
        return pp_distance2(state, ctx->goal, ctx->dimension) <= ctx->callbacks->goal_radius * ctx->callbacks->goal_radius;
    {
        int is_goal = ctx->callbacks->is_goal(ctx->callbacks->user_data, state,
                                             ctx->dimension);
        return is_goal == 0 || is_goal == 1 ? is_goal : -1;
    }
}

static int pp_goal_distance(pp_context *ctx, const double *state, double *out) {
    double distance;
    if (ctx->callbacks->native_goal) {
        *out = fmax(0.0, sqrt(pp_distance2(state, ctx->goal, ctx->dimension)) - ctx->callbacks->goal_radius);
        return 0;
    }
    if (ctx->callbacks->goal_distance != NULL) {
        distance = ctx->callbacks->goal_distance(ctx->callbacks->user_data, state, ctx->dimension);
        if (isnan(distance) || distance < 0.0) return -1;
        *out = isfinite(distance) ? distance : PP_C_INF;
        return 0;
    }
    if (!ctx->has_goal_point) {
        *out = PP_C_INF;
        return 0;
    }
    return pp_distance(ctx, state, ctx->goal, out);
}

static int pp_timed_out(pp_context *ctx) {
    double budget = ctx->options->time_budget_s;
    return budget > 0.0 && pp_now() - ctx->started >= budget;
}

static double pp_radius(const pp_context *ctx, uint64_t count) {
    double radius;
    if (count <= 1) return ctx->options->step_size;
    radius = ctx->options->rrt_star_radius_gamma *
        pow(log((double)count) / (double)count, 1.0 / (double)ctx->dimension);
    return fmin(ctx->options->step_size * ctx->options->rrt_star_radius_max_factor, radius);
}

static int pp_nearest(pp_context *ctx, const pp_kd_index *index, const double *target,
                      uint32_t *out) {
    size_t i;
    double best = PP_C_INF;
    uint32_t best_id = PP_C_NONE;
    if (index != NULL) return pp_kd_nearest(index, target, ctx->nodes.points, out);
    for (i = 0; i < ctx->nodes.count; ++i) {
        double distance;
        if (pp_distance(ctx, pp_point(&ctx->nodes, (uint32_t)i), target, &distance) != 0) return -1;
        if (distance < best) { best = distance; best_id = (uint32_t)i; }
    }
    if (best_id == PP_C_NONE) return -1;
    *out = best_id; return 0;
}

static int pp_nearest_tree(pp_context *ctx, const pp_kd_index *index, const double *target,
                           int tree_id, uint32_t *out) {
    size_t i;
    double best = PP_C_INF;
    uint32_t best_id = PP_C_NONE;
    if (index != NULL) return pp_kd_nearest(index, target, ctx->nodes.points, out);
    for (i = 0; i < ctx->nodes.count; ++i) {
        double distance;
        if ((int)ctx->nodes.flags[i] != tree_id) continue;
        if (pp_distance(ctx, pp_point(&ctx->nodes, (uint32_t)i), target, &distance) != 0) return -1;
        if (distance < best) { best = distance; best_id = (uint32_t)i; }
    }
    if (best_id == PP_C_NONE) return -1;
    *out = best_id; return 0;
}

static int pp_radius_ids(pp_context *ctx, const pp_kd_index *index, const double *target,
                         double radius, pp_ids *found) {
    size_t i;
    if (index != NULL) return pp_kd_radius(index, target, radius, ctx->nodes.points, found);
    for (i = 0; i < ctx->nodes.count; ++i) {
        double distance;
        if (pp_distance(ctx, pp_point(&ctx->nodes, (uint32_t)i), target, &distance) != 0) return -1;
        if (distance <= radius && pp_ids_append(found, (uint32_t)i) != 0) return -1;
    }
    return 0;
}

static int pp_add_index(pp_context *ctx, pp_kd_index *index, uint32_t id) {
    if (index == NULL) return 0;
    return pp_kd_insert(index, id, ctx->nodes.points);
}

static int pp_path_to_result(pp_context *ctx, const uint32_t *ids, size_t length,
                             double path_cost, int stop_reason,
                             pp_continuous_result *result) {
    size_t i;
    double *path;
    if (length == 0 || length > SIZE_MAX / ctx->dimension / sizeof(double)) return -1;
    path = (double *)malloc(length * ctx->dimension * sizeof(double));
    if (path == NULL) return -1;
    for (i = 0; i < length; ++i) memcpy(path + i * ctx->dimension, pp_point(&ctx->nodes, ids[i]), ctx->dimension * sizeof(double));
    result->success = 1; result->stop_reason = stop_reason;
    result->iters = ctx->iterations; result->nodes = ctx->nodes.count;
    result->sample_count = ctx->samples; result->batches = ctx->batches;
    result->motion_checks = ctx->motion_checks; result->rewires = ctx->rewires;
    result->path_cost = path_cost; result->elapsed_s = pp_now() - ctx->started;
    result->path = path; result->path_length = length; result->dimension = ctx->dimension;
    return 0;
}

static size_t pp_reconstruct(const pp_nodes *nodes, uint32_t end, uint32_t *path) {
    size_t length = 0, i;
    uint32_t current = end;
    while (current != PP_C_NONE && length < nodes->count) {
        path[length++] = current; current = nodes->parent[current];
    }
    if (length == 0 || nodes->parent[path[length - 1]] != PP_C_NONE) return 0;
    for (i = 0; i < length / 2; ++i) { uint32_t temp = path[i]; path[i] = path[length - 1 - i]; path[length - 1 - i] = temp; }
    return length;
}

static int pp_objective_for_node(pp_context *ctx, uint32_t node, double *out) {
    size_t length, i;
    uint32_t *ids;
    double *path, value;
    if (ctx->callbacks->path_objective == NULL) {
        uint32_t parent = ctx->nodes.parent[node];
        *out = parent == PP_C_NONE ? 0.0 : ctx->nodes.cost[parent] + ctx->nodes.edge_cost[node];
        return 0;
    }
    ids = (uint32_t *)malloc((size_t)ctx->nodes.count * sizeof(uint32_t));
    if (ids == NULL) return -1;
    length = pp_reconstruct(&ctx->nodes, node, ids);
    if (length == 0 || length > SIZE_MAX / ctx->dimension / sizeof(double)) { free(ids); return -1; }
    path = (double *)malloc(length * ctx->dimension * sizeof(double));
    if (path == NULL) { free(ids); return -1; }
    for (i = 0; i < length; ++i) memcpy(path + i * ctx->dimension, pp_point(&ctx->nodes, ids[i]), ctx->dimension * sizeof(double));
    value = ctx->callbacks->path_objective(ctx->callbacks->user_data, path, length, ctx->dimension);
    free(path); free(ids);
    if (!isfinite(value)) return -2;
    *out = value; return 0;
}

static int pp_objective_for_parent(pp_context *ctx, uint32_t parent,
                                   const double *candidate, double *out) {
    size_t length, i;
    uint32_t *ids;
    double *path, value;
    if (ctx->callbacks->path_objective == NULL) {
        double distance;
        if (pp_distance(ctx, pp_point(&ctx->nodes, parent), candidate, &distance) != 0) return -2;
        *out = ctx->nodes.cost[parent] + distance;
        return 0;
    }
    ids = (uint32_t *)malloc((size_t)ctx->nodes.count * sizeof(uint32_t));
    if (ids == NULL) return -1;
    length = pp_reconstruct(&ctx->nodes, parent, ids);
    if (length == 0 || length + 1 > SIZE_MAX / ctx->dimension / sizeof(double)) { free(ids); return -1; }
    path = (double *)malloc((length + 1) * ctx->dimension * sizeof(double));
    if (path == NULL) { free(ids); return -1; }
    for (i = 0; i < length; ++i) memcpy(path + i * ctx->dimension, pp_point(&ctx->nodes, ids[i]), ctx->dimension * sizeof(double));
    memcpy(path + length * ctx->dimension, candidate, ctx->dimension * sizeof(double));
    value = ctx->callbacks->path_objective(ctx->callbacks->user_data, path, length + 1, ctx->dimension);
    free(path); free(ids);
    if (!isfinite(value)) return -2;
    *out = value; return 0;
}

static int pp_update_descendant_costs(pp_context *ctx, uint32_t root) {
    pp_nodes *nodes = &ctx->nodes;
    uint32_t *stack = (uint32_t *)malloc((size_t)nodes->count * sizeof(uint32_t));
    size_t top = 0;
    if (stack == NULL) return -1;
    stack[top++] = root;
    while (top) {
        uint32_t parent = stack[--top], child;
        for (child = nodes->first_child[parent]; child != PP_C_NONE; child = nodes->next_sibling[child]) {
            int cost_status = pp_objective_for_node(ctx, child, &nodes->cost[child]);
            if (cost_status != 0) { free(stack); return cost_status; }
            if (top < nodes->count) stack[top++] = child;
        }
    }
    free(stack);
    return 0;
}

static int pp_sample_informed(pp_context *ctx, double best_cost, double *out) {
    double c_min, a, b, norm, radius;
    double *normal, *unit, *v;
    size_t i;
    double sum = 0.0;
    if (!ctx->has_goal_point || ctx->dimension < 2) return pp_sample(ctx, out);
    c_min = sqrt(pp_distance2(ctx->nodes.points, ctx->goal, ctx->dimension));
    if (best_cost <= c_min) { memcpy(out, ctx->nodes.points, ctx->dimension * sizeof(double)); return 0; }
    a = best_cost / 2.0;
    b = sqrt(fmax(0.0, best_cost * best_cost - c_min * c_min)) / 2.0;
    normal = (double *)malloc(ctx->dimension * sizeof(double));
    unit = (double *)malloc(ctx->dimension * sizeof(double));
    v = (double *)malloc(ctx->dimension * sizeof(double));
    if (normal == NULL || unit == NULL || v == NULL) { free(normal); free(unit); free(v); return -1; }
    for (i = 0; i < ctx->dimension; ++i) {
        double u = pp_uniform(&ctx->rng), w = pp_uniform(&ctx->rng);
        normal[i] = sqrt(-2.0 * log(u)) * cos(6.283185307179586 * w);
    }
    for (i = 0; i < ctx->dimension; ++i) sum += normal[i] * normal[i];
    norm = sqrt(sum);
    if (!(norm > 0.0) || !isfinite(norm)) norm = 1.0;
    radius = pow(pp_uniform(&ctx->rng), 1.0 / (double)ctx->dimension);
    for (i = 0; i < ctx->dimension; ++i) unit[i] = normal[i] * radius / norm;
    {
        double direction_norm = c_min;
        double first = (ctx->goal[0] - ctx->nodes.points[0]) / direction_norm;
        double v_norm = 0.0;
        v[0] = 1.0 - first;
        for (i = 1; i < ctx->dimension; ++i) {
            double direction = (ctx->goal[i] - ctx->nodes.points[i]) / direction_norm;
            v[i] = -direction;
        }
        for (i = 0; i < ctx->dimension; ++i) v_norm += v[i] * v[i];
        if (v_norm < 1e-20) memcpy(normal, unit, ctx->dimension * sizeof(double));
        else {
            double dot = 0.0;
            for (i = 0; i < ctx->dimension; ++i) dot += v[i] * unit[i];
            for (i = 0; i < ctx->dimension; ++i) normal[i] = unit[i] - 2.0 * v[i] * dot / v_norm;
        }
    }
    for (i = 0; i < ctx->dimension; ++i) {
        double scale = i == 0 ? a : b;
        out[i] = 0.5 * (ctx->nodes.points[i] + ctx->goal[i]) + normal[i] * scale;
    }
    free(normal); free(unit); free(v);
    {
        int valid = pp_valid_state(ctx, out);
        if (valid < 0) return -1;
        if (valid == 0) return -2;
    }
    ++ctx->samples;
    return 0;
}

static int pp_edge_less(const pp_edge_entry *a, const pp_edge_entry *b) {
    if (a->key != b->key) return a->key < b->key;
    return a->order < b->order;
}

static int pp_edge_push(pp_edge_heap *heap, uint32_t source, uint32_t target,
                        double key, double source_cost, double tentative) {
    pp_edge_entry entry;
    size_t index;
    if (heap->count == heap->capacity) {
        size_t capacity = heap->capacity == 0 ? 64 : heap->capacity * 2;
        pp_edge_entry *grown;
        if (capacity < heap->capacity || capacity > SIZE_MAX / sizeof(*grown)) return -1;
        grown = (pp_edge_entry *)realloc(heap->items, capacity * sizeof(*grown));
        if (grown == NULL) return -1;
        heap->items = grown; heap->capacity = capacity;
    }
    entry.source = source; entry.target = target; entry.key = key;
    entry.source_cost = source_cost; entry.tentative = tentative; entry.order = heap->next_order++;
    index = heap->count++;
    while (index > 0) {
        size_t parent = (index - 1) / 2;
        if (!pp_edge_less(&entry, &heap->items[parent])) break;
        heap->items[index] = heap->items[parent]; index = parent;
    }
    heap->items[index] = entry; return 0;
}

static int pp_edge_pop(pp_edge_heap *heap, pp_edge_entry *out) {
    pp_edge_entry tail;
    size_t index = 0;
    if (heap->count == 0) return 0;
    *out = heap->items[0]; tail = heap->items[--heap->count];
    while (index * 2 + 1 < heap->count) {
        size_t child = index * 2 + 1;
        if (child + 1 < heap->count && pp_edge_less(&heap->items[child + 1], &heap->items[child])) ++child;
        if (!pp_edge_less(&heap->items[child], &tail)) break;
        heap->items[index] = heap->items[child]; index = child;
    }
    if (heap->count) heap->items[index] = tail;
    return 1;
}

typedef struct {
    uint32_t node;
    double key;
    double queued_cost;
    uint64_t order;
} pp_vertex_entry;

typedef struct {
    pp_vertex_entry *items;
    size_t count;
    size_t capacity;
    uint64_t next_order;
} pp_vertex_heap;

static int pp_vertex_less(const pp_vertex_entry *a, const pp_vertex_entry *b) {
    if (a->key != b->key) return a->key < b->key;
    return a->order < b->order;
}

static int pp_vertex_push(pp_vertex_heap *heap, uint32_t node, double key, double cost) {
    pp_vertex_entry entry;
    size_t index;
    if (heap->count == heap->capacity) {
        size_t capacity = heap->capacity == 0 ? 64 : heap->capacity * 2;
        pp_vertex_entry *grown;
        if (capacity < heap->capacity || capacity > SIZE_MAX / sizeof(*grown)) return -1;
        grown = (pp_vertex_entry *)realloc(heap->items, capacity * sizeof(*grown));
        if (grown == NULL) return -1;
        heap->items = grown; heap->capacity = capacity;
    }
    entry.node = node; entry.key = key; entry.queued_cost = cost; entry.order = heap->next_order++;
    index = heap->count++;
    while (index > 0) {
        size_t parent = (index - 1) / 2;
        if (!pp_vertex_less(&entry, &heap->items[parent])) break;
        heap->items[index] = heap->items[parent]; index = parent;
    }
    heap->items[index] = entry; return 0;
}

static int pp_vertex_pop(pp_vertex_heap *heap, pp_vertex_entry *out) {
    pp_vertex_entry tail;
    size_t index = 0;
    if (heap->count == 0) return 0;
    *out = heap->items[0]; tail = heap->items[--heap->count];
    while (index * 2 + 1 < heap->count) {
        size_t child = index * 2 + 1;
        if (child + 1 < heap->count && pp_vertex_less(&heap->items[child + 1], &heap->items[child])) ++child;
        if (!pp_vertex_less(&heap->items[child], &tail)) break;
        heap->items[index] = heap->items[child]; index = child;
    }
    if (heap->count) heap->items[index] = tail;
    return 1;
}

static double pp_vertex_peek(const pp_vertex_heap *heap) {
    return heap->count == 0 ? PP_C_INF : heap->items[0].key;
}

static double pp_edge_peek(const pp_edge_heap *heap) {
    return heap->count == 0 ? PP_C_INF : heap->items[0].key;
}

static int pp_is_ancestor(const pp_nodes *nodes, uint32_t possible_ancestor, uint32_t node) {
    uint32_t current = node;
    while (current != PP_C_NONE) {
        if (current == possible_ancestor) return 1;
        current = nodes->parent[current];
    }
    return 0;
}

static int pp_push_sample(pp_context *ctx, double *out, double best_cost, int informed) {
    uint64_t attempt;
    for (attempt = 0; attempt < ctx->options->max_sample_tries; ++attempt) {
        int status;
        if (informed && isfinite(best_cost)) status = pp_sample_informed(ctx, best_cost, out);
        else status = pp_sample(ctx, out);
        if (status == 0) return 0;
        if (status == -1) return -1;
        if (status == -2) continue;
        return -1;
    }
    return -2;
}

static int pp_record_path(pp_context *ctx, uint32_t goal_id,
                          pp_continuous_result *result) {
    uint32_t *ids;
    size_t length;
    int status;
    ids = (uint32_t *)malloc((size_t)ctx->nodes.count * sizeof(uint32_t));
    if (ids == NULL) return -1;
    length = pp_reconstruct(&ctx->nodes, goal_id, ids);
    status = length == 0 ? -1 : pp_path_to_result(ctx, ids, length,
                                                   ctx->nodes.cost[goal_id], 0, result);
    free(ids);
    return status;
}

static int pp_run_rrt(pp_context *ctx, int optimize, int informed,
                      pp_continuous_result *result) {
    double *sample = NULL, *candidate = NULL;
    pp_ids neighbors = {0};
    uint32_t root, best_goal = PP_C_NONE;
    double best_cost = PP_C_INF;
    uint64_t iteration;
    int stop_reason = 3, status = -1, root_goal;
    sample = (double *)malloc(ctx->dimension * sizeof(double));
    candidate = (double *)malloc(ctx->dimension * sizeof(double));
    if (sample == NULL || candidate == NULL) { pp_set_error(result, "out of memory allocating sample state"); goto done; }
    ctx->indices[0].dimension = ctx->dimension;
    ctx->indices[1].dimension = ctx->dimension;
    root = pp_node_add(&ctx->nodes, ctx->nodes.points, PP_C_NONE, 0.0, 0.0, 0);
    if (root == PP_C_NONE || pp_add_index(ctx, ctx->options->use_euclidean_index ? &ctx->indices[0] : NULL, root) != 0) {
        pp_set_error(result, "out of memory creating start node index"); goto done;
    }
    root_goal = pp_is_goal(ctx, pp_point(&ctx->nodes, root));
    if (root_goal < 0) { pp_set_error(result, "goal callback failed"); goto done; }
    if (ctx->callbacks->path_objective != NULL && pp_objective_for_node(ctx, root, &ctx->nodes.cost[root]) != 0) {
        pp_set_error(result, "objective callback failed for start state"); goto done;
    }
    if (root_goal > 0) { status = pp_record_path(ctx, root, result); goto done; }
    for (iteration = 0; iteration < ctx->options->max_iters; ++iteration) {
        uint32_t nearest, new_id, parent_id;
        double edge_length, best_parent_cost;
        int to_goal = ctx->has_goal_point && pp_uniform(&ctx->rng) < ctx->options->goal_sample_rate;
        int valid, found_goal;
        if (pp_timed_out(ctx)) { stop_reason = 1; break; }
        if (to_goal) memcpy(sample, ctx->goal, ctx->dimension * sizeof(double));
        else if (pp_push_sample(ctx, sample, best_cost, informed) != 0) { pp_set_error(result, "failed to sample a valid state"); goto done; }
        if (pp_nearest(ctx, ctx->options->use_euclidean_index ? &ctx->indices[0] : NULL,
                       sample, &nearest) != 0) { pp_set_error(result, "nearest-node query failed"); goto done; }
        if (pp_steer(ctx, pp_point(&ctx->nodes, nearest), sample, candidate, &edge_length) != 0) {
            pp_set_error(result, "distance or steering callback failed"); goto done;
        }
        if (edge_length <= ctx->options->goal_reach_tolerance) {
            found_goal = pp_is_goal(ctx, pp_point(&ctx->nodes, nearest));
            if (found_goal < 0) { pp_set_error(result, "goal callback failed"); goto done; }
            if (found_goal > 0) {
                if (!optimize) { ++ctx->iterations; status = pp_record_path(ctx, nearest, result); goto done; }
                if (ctx->nodes.cost[nearest] < best_cost) { best_goal = nearest; best_cost = ctx->nodes.cost[nearest]; }
            }
            ++ctx->iterations;
            continue;
        }
        valid = pp_valid_state(ctx, candidate);
        if (valid < 0) { pp_set_error(result, "state-validity callback failed"); goto done; }
        if (!valid) { ++ctx->iterations; continue; }

        parent_id = nearest;
        best_parent_cost = ctx->nodes.cost[nearest] + edge_length;
        if (optimize) {
            double radius = pp_radius(ctx, (uint64_t)ctx->nodes.count + 1);
            size_t j;
            valid = pp_valid_motion(ctx, pp_point(&ctx->nodes, nearest), candidate);
            if (valid < 0) { pp_set_error(result, "motion-validity callback failed"); goto done; }
            if (!valid) { parent_id = PP_C_NONE; best_parent_cost = PP_C_INF; }
            else if (ctx->callbacks->path_objective != NULL &&
                     pp_objective_for_parent(ctx, nearest, candidate, &best_parent_cost) != 0) {
                pp_set_error(result, "objective callback failed while choosing a parent"); goto done;
            }
            neighbors.count = 0;
            if (pp_radius_ids(ctx, ctx->options->use_euclidean_index ? &ctx->indices[0] : NULL,
                              candidate, radius, &neighbors) != 0) { pp_set_error(result, "near-node query failed"); goto done; }
            for (j = 0; j < neighbors.count; ++j) {
                uint32_t near_id = neighbors.items[j];
                double distance, proposed;
                if (pp_distance(ctx, pp_point(&ctx->nodes, near_id), candidate, &distance) != 0) { pp_set_error(result, "distance callback failed"); goto done; }
                if (ctx->callbacks->path_objective != NULL) {
                    if (pp_objective_for_parent(ctx, near_id, candidate, &proposed) != 0) { pp_set_error(result, "objective callback failed while choosing a parent"); goto done; }
                } else proposed = ctx->nodes.cost[near_id] + distance;
                if (proposed + 1e-12 >= best_parent_cost) continue;
                valid = pp_valid_motion(ctx, pp_point(&ctx->nodes, near_id), candidate);
                if (valid < 0) { pp_set_error(result, "motion-validity callback failed"); goto done; }
                if (valid) { parent_id = near_id; best_parent_cost = proposed; edge_length = distance; }
            }
            if (parent_id == PP_C_NONE) { ++ctx->iterations; continue; }
        } else {
            valid = pp_valid_motion(ctx, pp_point(&ctx->nodes, nearest), candidate);
            if (valid < 0) { pp_set_error(result, "motion-validity callback failed"); goto done; }
            if (!valid) { ++ctx->iterations; continue; }
        }
        new_id = pp_node_add(&ctx->nodes, candidate, parent_id, best_parent_cost, edge_length, 0);
        if (new_id == PP_C_NONE || pp_add_index(ctx, ctx->options->use_euclidean_index ? &ctx->indices[0] : NULL, new_id) != 0) {
            pp_set_error(result, "out of memory growing planner tree"); goto done;
        }
        if (optimize) {
            size_t j;
            for (j = 0; j < neighbors.count; ++j) {
                uint32_t near_id = neighbors.items[j];
                double distance, proposed;
                if (near_id == new_id || near_id == parent_id || pp_is_ancestor(&ctx->nodes, near_id, new_id)) continue;
                if (pp_distance(ctx, candidate, pp_point(&ctx->nodes, near_id), &distance) != 0) { pp_set_error(result, "distance callback failed"); goto done; }
                if (ctx->callbacks->path_objective != NULL) {
                    if (pp_objective_for_parent(ctx, new_id, pp_point(&ctx->nodes, near_id), &proposed) != 0) { pp_set_error(result, "objective callback failed while rewiring"); goto done; }
                } else proposed = ctx->nodes.cost[new_id] + distance;
                if (proposed + 1e-12 >= ctx->nodes.cost[near_id]) continue;
                valid = pp_valid_motion(ctx, candidate, pp_point(&ctx->nodes, near_id));
                if (valid < 0) { pp_set_error(result, "motion-validity callback failed"); goto done; }
                if (!valid) continue;
                pp_node_attach(&ctx->nodes, near_id, new_id, distance, proposed);
                if (pp_update_descendant_costs(ctx, near_id) != 0) { pp_set_error(result, "failed to update rewired subtree objective"); goto done; }
                ++ctx->rewires;
            }
        }
        found_goal = pp_is_goal(ctx, pp_point(&ctx->nodes, new_id));
        if (found_goal < 0) { pp_set_error(result, "goal callback failed"); goto done; }
        if (found_goal) {
            ctx->nodes.flags[new_id] |= 2;
            if (!optimize) { ++ctx->iterations; status = pp_record_path(ctx, new_id, result); goto done; }
        }
        if (optimize) {
            uint32_t node;
            for (node = 0; node < ctx->nodes.count; ++node) {
                if ((ctx->nodes.flags[node] & 2) && ctx->nodes.cost[node] < best_cost) {
                    best_cost = ctx->nodes.cost[node]; best_goal = node;
                }
            }
        }
        ++ctx->iterations;
    }
    if (stop_reason != 1 && ctx->iterations >= ctx->options->max_iters) stop_reason = 2;
    if (best_goal != PP_C_NONE) { status = pp_record_path(ctx, best_goal, result); goto done; }
    result->success = 0; result->stop_reason = stop_reason; result->iters = ctx->iterations;
    result->nodes = ctx->nodes.count; result->sample_count = ctx->samples;
    result->batches = ctx->batches; result->motion_checks = ctx->motion_checks;
    result->rewires = ctx->rewires; result->elapsed_s = pp_now() - ctx->started;
    status = 0;
done:
    free(sample); free(candidate); free(neighbors.items);
    return status;
}

static int pp_run_fmt(pp_context *ctx, pp_continuous_result *result) {
    pp_vertex_heap open = {0};
    pp_ids near = {0}, parents = {0};
    double *sample = NULL;
    uint8_t *status = NULL;
    uint32_t *proposal_target = NULL, *proposal_parent = NULL, *parent_temp = NULL;
    double *proposal_cost = NULL, *proposal_edge = NULL, *parent_cost_temp = NULL, *parent_edge_temp = NULL;
    uint32_t root, goal_id, node_count, node;
    double radius;
    int stop = 3, out = -1;
    sample = (double *)malloc(ctx->dimension * sizeof(double));
    if (sample == NULL) { pp_set_error(result, "out of memory allocating FMT* sample"); goto done; }
    ctx->indices[0].dimension = ctx->dimension;
    root = pp_node_add(&ctx->nodes, ctx->nodes.points, PP_C_NONE, 0.0, 0.0, 0);
    goal_id = pp_node_add(&ctx->nodes, ctx->goal, PP_C_NONE, PP_C_INF, 0.0, 0);
    if (root == PP_C_NONE || goal_id == PP_C_NONE || pp_add_index(ctx, &ctx->indices[0], root) != 0 || pp_add_index(ctx, &ctx->indices[0], goal_id) != 0) {
        pp_set_error(result, "out of memory initializing FMT* tree"); goto done;
    }
    for (node = 0; node < ctx->options->sample_count; ++node) {
        if (pp_timed_out(ctx)) { stop = 1; break; }
        if (pp_push_sample(ctx, sample, PP_C_INF, 0) != 0) { pp_set_error(result, "failed to generate FMT* samples"); goto done; }
        if (pp_node_add(&ctx->nodes, sample, PP_C_NONE, PP_C_INF, 0.0, 0) == PP_C_NONE || pp_add_index(ctx, &ctx->indices[0], ctx->nodes.count - 1) != 0) {
            pp_set_error(result, "out of memory initializing FMT* samples"); goto done;
        }
    }
    if (stop == 1) {
        result->success = 0; result->stop_reason = stop; result->iters = ctx->iterations;
        result->nodes = ctx->nodes.count; result->sample_count = ctx->samples;
        result->motion_checks = ctx->motion_checks; result->elapsed_s = pp_now() - ctx->started;
        out = 0; goto done;
    }
    node_count = ctx->nodes.count;
    radius = pp_radius(ctx, node_count);
    status = (uint8_t *)calloc(node_count, sizeof(uint8_t));
    proposal_target = (uint32_t *)malloc((size_t)node_count * sizeof(uint32_t));
    proposal_parent = (uint32_t *)malloc((size_t)node_count * sizeof(uint32_t));
    proposal_cost = (double *)malloc((size_t)node_count * sizeof(double));
    proposal_edge = (double *)malloc((size_t)node_count * sizeof(double));
    parent_temp = (uint32_t *)malloc((size_t)node_count * sizeof(uint32_t));
    parent_cost_temp = (double *)malloc((size_t)node_count * sizeof(double));
    parent_edge_temp = (double *)malloc((size_t)node_count * sizeof(double));
    if (status == NULL || proposal_target == NULL || proposal_parent == NULL || proposal_cost == NULL || proposal_edge == NULL ||
        parent_temp == NULL || parent_cost_temp == NULL || parent_edge_temp == NULL) {
        pp_set_error(result, "out of memory allocating FMT* search state"); goto done;
    }
    status[root] = 1;
    if (pp_vertex_push(&open, root, 0.0, 0.0) != 0) { pp_set_error(result, "out of memory creating FMT* queue"); goto done; }
    while (open.count && ctx->iterations < ctx->options->max_iters) {
        pp_vertex_entry current_entry;
        size_t proposal_count = 0, i;
        uint32_t current;
        if (pp_timed_out(ctx)) { stop = 1; break; }
        pp_vertex_pop(&open, &current_entry); current = current_entry.node;
        if (status[current] != 1 || current_entry.queued_cost > ctx->nodes.cost[current] + 1e-12) continue;
        status[current] = 2; ++ctx->iterations;
        if (current == goal_id) { out = pp_record_path(ctx, goal_id, result); goto done; }
        near.count = 0;
        if (pp_radius_ids(ctx, &ctx->indices[0], pp_point(&ctx->nodes, current), radius, &near) != 0) { pp_set_error(result, "FMT* neighbor query failed"); goto done; }
        for (i = 0; i < near.count; ++i) {
            uint32_t target = near.items[i];
            size_t parent_count = 0, j, k;
            uint32_t best_parent = PP_C_NONE;
            double best_cost = PP_C_INF, best_edge = 0.0;
            if (status[target] != 0) continue;
            parents.count = 0;
            if (pp_radius_ids(ctx, &ctx->indices[0], pp_point(&ctx->nodes, target), radius, &parents) != 0) { pp_set_error(result, "FMT* parent query failed"); goto done; }
            for (j = 0; j < parents.count; ++j) {
                uint32_t parent = parents.items[j];
                double edge_cost, cost;
                if (!(status[parent] == 1 || parent == current)) continue;
                if (pp_distance(ctx, pp_point(&ctx->nodes, parent), pp_point(&ctx->nodes, target), &edge_cost) != 0) { pp_set_error(result, "FMT* distance callback failed"); goto done; }
                cost = ctx->nodes.cost[parent] + edge_cost;
                for (k = parent_count; k > 0 && cost < parent_cost_temp[k - 1]; --k) {
                    parent_cost_temp[k] = parent_cost_temp[k - 1]; parent_edge_temp[k] = parent_edge_temp[k - 1];
                    parent_temp[k] = parent_temp[k - 1];
                }
                parent_cost_temp[k] = cost; parent_edge_temp[k] = edge_cost; parent_temp[k] = parent; ++parent_count;
            }
            for (j = 0; j < parent_count; ++j) {
                int valid = pp_valid_motion(ctx, pp_point(&ctx->nodes, parent_temp[j]), pp_point(&ctx->nodes, target));
                if (valid < 0) { pp_set_error(result, "FMT* motion-validity callback failed"); goto done; }
                if (valid) { best_parent = parent_temp[j]; best_cost = parent_cost_temp[j]; best_edge = parent_edge_temp[j]; break; }
            }
            if (best_parent != PP_C_NONE) {
                proposal_target[proposal_count] = target; proposal_parent[proposal_count] = best_parent;
                proposal_cost[proposal_count] = best_cost; proposal_edge[proposal_count] = best_edge; ++proposal_count;
            }
        }
        for (i = 0; i < proposal_count; ++i) {
            uint32_t target = proposal_target[i], parent = proposal_parent[i];
            if (status[target] != 0) continue;
            status[target] = 1; ctx->nodes.parent[target] = PP_C_NONE;
            ctx->nodes.cost[target] = proposal_cost[i];
            pp_node_attach(&ctx->nodes, target, parent, proposal_edge[i], proposal_cost[i]);
            if (pp_vertex_push(&open, target, proposal_cost[i], proposal_cost[i]) != 0) { pp_set_error(result, "out of memory growing FMT* queue"); goto done; }
        }
    }
    if (stop != 1 && ctx->iterations >= ctx->options->max_iters) stop = 2;
    result->success = 0; result->stop_reason = stop; result->iters = ctx->iterations;
    result->nodes = node_count; result->sample_count = ctx->samples; result->motion_checks = ctx->motion_checks;
    result->elapsed_s = pp_now() - ctx->started; out = 0;
done:
    free(sample); free(open.items); free(near.items); free(parents.items); free(status);
    free(proposal_target); free(proposal_parent); free(proposal_cost); free(proposal_edge);
    free(parent_temp); free(parent_cost_temp); free(parent_edge_temp);
    return out;
}

static int pp_run_bit_batch(pp_context *ctx, uint32_t node_count, double best_cost,
                            double inflation, double truncation, uint64_t max_expansions,
                            pp_continuous_result *result, int *timed_out) {
    pp_vertex_heap vertices = {0};
    pp_edge_heap edges = {0};
    pp_ids near = {0};
    uint8_t *closed = NULL;
    double *expanded_cost = NULL, *open_cost = NULL;
    uint32_t node;
    uint64_t batch_expanded = 0;
    int status = -1;
    closed = (uint8_t *)calloc(node_count, sizeof(uint8_t));
    expanded_cost = (double *)malloc((size_t)node_count * sizeof(double));
    open_cost = (double *)malloc((size_t)node_count * sizeof(double));
    if (closed == NULL || expanded_cost == NULL || open_cost == NULL) { pp_set_error(result, "out of memory allocating BIT* batch state"); goto done; }
    for (node = 0; node < node_count; ++node) {
        expanded_cost[node] = PP_C_INF; open_cost[node] = PP_C_INF;
        if (isfinite(ctx->nodes.cost[node])) {
            double h;
            if (pp_goal_distance(ctx, pp_point(&ctx->nodes, node), &h) != 0) {
                pp_set_error(result, "goal-distance callback failed"); goto done;
            }
            if (pp_vertex_push(&vertices, node, ctx->nodes.cost[node] + inflation * h, ctx->nodes.cost[node]) != 0) { pp_set_error(result, "out of memory creating BIT* vertex queue"); goto done; }
            open_cost[node] = ctx->nodes.cost[node];
        }
    }
    while (vertices.count || edges.count) {
        pp_vertex_entry ventry;
        pp_edge_entry eentry;
        double vkey, ekey;
        while (vertices.count) {
            ventry = vertices.items[0];
            if (ventry.queued_cost != ctx->nodes.cost[ventry.node] || ventry.queued_cost != open_cost[ventry.node] ||
                (closed[ventry.node] && expanded_cost[ventry.node] <= ventry.queued_cost)) {
                pp_vertex_pop(&vertices, &ventry);
                if (closed[ventry.node] && expanded_cost[ventry.node] <= ventry.queued_cost) open_cost[ventry.node] = PP_C_INF;
                continue;
            }
            break;
        }
        while (edges.count) {
            eentry = edges.items[0];
            if (eentry.source_cost != ctx->nodes.cost[eentry.source]) { pp_edge_pop(&edges, &eentry); continue; }
            break;
        }
        vkey = pp_vertex_peek(&vertices); ekey = pp_edge_peek(&edges);
        if (isfinite(best_cost) && fmin(vkey, ekey) >= best_cost) break;
        if (vkey <= ekey) {
            uint32_t source;
            size_t i;
            if (ctx->iterations >= ctx->options->max_iters || batch_expanded >= max_expansions) break;
            pp_vertex_pop(&vertices, &ventry); source = ventry.node;
            if (ventry.queued_cost != ctx->nodes.cost[source]) continue;
            open_cost[source] = PP_C_INF;
            if (closed[source] && expanded_cost[source] <= ventry.queued_cost) continue;
            closed[source] = 1; expanded_cost[source] = ventry.queued_cost; ++ctx->iterations; ++batch_expanded;
            near.count = 0;
            if (pp_radius_ids(ctx, &ctx->indices[0], pp_point(&ctx->nodes, source),
                              pp_radius(ctx, node_count), &near) != 0) { pp_set_error(result, "BIT* neighbor query failed"); goto done; }
            for (i = 0; i < near.count; ++i) {
                uint32_t target = near.items[i];
                double edge_cost, tentative, h;
                if (source == target) continue;
                if (pp_distance(ctx, pp_point(&ctx->nodes, source), pp_point(&ctx->nodes, target), &edge_cost) != 0) { pp_set_error(result, "BIT* distance callback failed"); goto done; }
                tentative = ventry.queued_cost + edge_cost;
                if (pp_goal_distance(ctx, pp_point(&ctx->nodes, target), &h) != 0) {
                    pp_set_error(result, "goal-distance callback failed"); goto done;
                }
                if (isfinite(best_cost) && tentative + h > truncation * best_cost) continue;
                if (pp_edge_push(&edges, source, target, tentative + inflation * h, ventry.queued_cost, tentative) != 0) { pp_set_error(result, "out of memory growing BIT* edge queue"); goto done; }
            }
        } else {
            uint32_t source, target;
            double h;
            int valid;
            pp_edge_pop(&edges, &eentry); source = eentry.source; target = eentry.target;
            if (eentry.source_cost != ctx->nodes.cost[source] || eentry.tentative >= ctx->nodes.cost[target]) continue;
            if (pp_goal_distance(ctx, pp_point(&ctx->nodes, target), &h) != 0) {
                pp_set_error(result, "goal-distance callback failed"); goto done;
            }
            if (isfinite(best_cost) && eentry.tentative + h > truncation * best_cost) continue;
            valid = pp_valid_motion(ctx, pp_point(&ctx->nodes, source), pp_point(&ctx->nodes, target));
            if (valid < 0) { pp_set_error(result, "BIT* motion-validity callback failed"); goto done; }
            if (!valid) continue;
            pp_node_attach(&ctx->nodes, target, source,
                           eentry.tentative - ctx->nodes.cost[source], eentry.tentative);
            if (pp_update_descendant_costs(ctx, target) != 0) { pp_set_error(result, "failed to update BIT* subtree costs"); goto done; }
            closed[target] = 0; open_cost[target] = eentry.tentative;
            if (pp_vertex_push(&vertices, target, eentry.tentative + inflation * h, eentry.tentative) != 0) { pp_set_error(result, "out of memory growing BIT* vertex queue"); goto done; }
            if (target == 1 && eentry.tentative < best_cost) best_cost = eentry.tentative;
        }
        if (pp_timed_out(ctx)) { *timed_out = 1; break; }
    }
    status = 0;
done:
    free(vertices.items); free(edges.items); free(near.items); free(closed); free(expanded_cost); free(open_cost);
    return status;
}

static int pp_run_bit(pp_context *ctx, int anytime, pp_continuous_result *result) {
    double *sample = NULL;
    uint32_t root, goal_id;
    double best_cost = PP_C_INF;
    uint64_t sample_count = 0;
    int stop = 3, timed_out = 0, status = -1;
    sample = (double *)malloc(ctx->dimension * sizeof(double));
    if (sample == NULL) { pp_set_error(result, "out of memory allocating BIT* sample"); goto done; }
    ctx->indices[0].dimension = ctx->dimension;
    root = pp_node_add(&ctx->nodes, ctx->nodes.points, PP_C_NONE, 0.0, 0.0, 0);
    goal_id = pp_node_add(&ctx->nodes, ctx->goal, PP_C_NONE, PP_C_INF, 0.0, 0);
    if (root == PP_C_NONE || goal_id == PP_C_NONE || pp_add_index(ctx, &ctx->indices[0], root) != 0 || pp_add_index(ctx, &ctx->indices[0], goal_id) != 0) {
        pp_set_error(result, "out of memory initializing BIT* tree"); goto done;
    }
    while (sample_count < ctx->options->sample_count && ctx->iterations < ctx->options->max_iters) {
        uint64_t batch_count = ctx->options->batch_size;
        uint64_t j;
        uint32_t node_count;
        double inflation = anytime ? 1.0 + ctx->options->abit_inflation_parameter / (double)(ctx->batches + 1) : 1.0;
        double truncation = anytime ? 1.0 + ctx->options->abit_truncation_parameter / (double)(ctx->batches + 1) : 1.0;
        if (pp_timed_out(ctx)) { stop = 1; break; }
        if (batch_count > ctx->options->sample_count - sample_count) batch_count = ctx->options->sample_count - sample_count;
        for (j = 0; j < batch_count; ++j) {
            if (pp_timed_out(ctx)) { stop = 1; timed_out = 1; break; }
            if (pp_push_sample(ctx, sample, best_cost, 1) != 0) { pp_set_error(result, "failed to generate BIT* samples"); goto done; }
            if (pp_node_add(&ctx->nodes, sample, PP_C_NONE, PP_C_INF, 0.0, 0) == PP_C_NONE || pp_add_index(ctx, &ctx->indices[0], ctx->nodes.count - 1) != 0) {
                pp_set_error(result, "out of memory extending BIT* samples"); goto done;
            }
        }
        if (timed_out) break;
        sample_count += batch_count; node_count = ctx->nodes.count; ++ctx->batches;
        if (pp_run_bit_batch(ctx, node_count, best_cost, inflation, truncation,
                             ctx->options->max_iters - ctx->iterations, result, &timed_out) != 0) goto done;
        if (isfinite(ctx->nodes.cost[goal_id]) && ctx->nodes.cost[goal_id] < best_cost) best_cost = ctx->nodes.cost[goal_id];
        if (timed_out) { stop = 1; break; }
        if (ctx->iterations >= ctx->options->max_iters) { stop = 2; break; }
        if (sample_count >= ctx->options->sample_count) { stop = isfinite(best_cost) ? 0 : 3; break; }
        if (isfinite(best_cost)) {
            double lower_bound;
            if (pp_distance(ctx, ctx->nodes.points, ctx->goal, &lower_bound) != 0) { pp_set_error(result, "goal-distance callback failed"); goto done; }
            if (fabs(best_cost - lower_bound) <= fmax(1e-12, fabs(lower_bound) * 1e-10)) { stop = 0; break; }
        }
    }
    result->success = 0; result->stop_reason = stop; result->iters = ctx->iterations;
    result->nodes = ctx->nodes.count; result->sample_count = ctx->samples;
    result->batches = ctx->batches; result->motion_checks = ctx->motion_checks;
    result->elapsed_s = pp_now() - ctx->started;
    if (isfinite(best_cost) && pp_record_path(ctx, goal_id, result) == 0) status = 0;
    else status = 0;
done:
    free(sample);
    return status;
}

static int pp_run_connect(pp_context *ctx, pp_continuous_result *result) {
    double *sample = NULL, *candidate = NULL;
    uint32_t root_start, root_goal, start_join = PP_C_NONE, goal_join = PP_C_NONE;
    uint64_t iteration;
    int stop = 3, status = -1, active_tree = 0;
    sample = (double *)malloc(ctx->dimension * sizeof(double));
    candidate = (double *)malloc(ctx->dimension * sizeof(double));
    if (sample == NULL || candidate == NULL) { pp_set_error(result, "out of memory allocating RRT-Connect state"); goto done; }
    ctx->indices[0].dimension = ctx->dimension; ctx->indices[1].dimension = ctx->dimension;
    root_start = pp_node_add(&ctx->nodes, ctx->nodes.points, PP_C_NONE, 0.0, 0.0, 0);
    root_goal = pp_node_add(&ctx->nodes, ctx->goal, PP_C_NONE, 0.0, 0.0, 1);
    if (root_start == PP_C_NONE || root_goal == PP_C_NONE ||
        pp_add_index(ctx, ctx->options->use_euclidean_index ? &ctx->indices[0] : NULL, root_start) != 0 ||
        pp_add_index(ctx, ctx->options->use_euclidean_index ? &ctx->indices[1] : NULL, root_goal) != 0) {
        pp_set_error(result, "out of memory initializing RRT-Connect trees"); goto done;
    }
    if (sqrt(pp_distance2(pp_point(&ctx->nodes, root_start), pp_point(&ctx->nodes, root_goal),
                          ctx->dimension)) <= ctx->options->goal_reach_tolerance) {
        status = pp_record_path(ctx, root_start, result); goto done;
    }
    for (iteration = 0; iteration < ctx->options->max_iters; ++iteration) {
        uint32_t near_id, new_id, other_id, tree_id = (uint32_t)active_tree;
        int sample_status, valid, connected = 0;
        double edge_length, distance;
        uint64_t connect_step;
        pp_kd_index *active_index = ctx->options->use_euclidean_index ? &ctx->indices[tree_id] : NULL;
        pp_kd_index *other_index = ctx->options->use_euclidean_index ? &ctx->indices[1 - tree_id] : NULL;
        if (pp_timed_out(ctx)) { stop = 1; break; }
        if (pp_uniform(&ctx->rng) < ctx->options->goal_sample_rate) memcpy(sample, ctx->goal, ctx->dimension * sizeof(double));
        else if ((sample_status = pp_push_sample(ctx, sample, PP_C_INF, 0)) != 0) { pp_set_error(result, "failed to sample RRT-Connect target"); goto done; }
        if (pp_nearest_tree(ctx, active_index, sample, (int)tree_id, &near_id) != 0 ||
            pp_steer(ctx, pp_point(&ctx->nodes, near_id), sample, candidate, &edge_length) != 0) { pp_set_error(result, "RRT-Connect nearest or steering operation failed"); goto done; }
        if (edge_length <= ctx->options->goal_reach_tolerance) { active_tree = 1 - active_tree; ++ctx->iterations; continue; }
        valid = pp_valid_state(ctx, candidate);
        if (valid < 0) { pp_set_error(result, "state-validity callback failed"); goto done; }
        if (!valid) { active_tree = 1 - active_tree; ++ctx->iterations; continue; }
        valid = pp_valid_motion(ctx, pp_point(&ctx->nodes, near_id), candidate);
        if (valid < 0) { pp_set_error(result, "motion-validity callback failed"); goto done; }
        if (!valid) { active_tree = 1 - active_tree; ++ctx->iterations; continue; }
        new_id = pp_node_add(&ctx->nodes, candidate, near_id,
                             ctx->nodes.cost[near_id] + edge_length, edge_length, (uint8_t)tree_id);
        if (new_id == PP_C_NONE || pp_add_index(ctx, active_index, new_id) != 0) { pp_set_error(result, "out of memory growing RRT-Connect tree"); goto done; }
        if (pp_nearest_tree(ctx, other_index, pp_point(&ctx->nodes, new_id), (int)(1 - tree_id), &other_id) != 0) { pp_set_error(result, "RRT-Connect opposite-tree query failed"); goto done; }
        for (connect_step = 0; connect_step < ctx->options->max_sample_tries; ++connect_step) {
            const double *other_point = pp_point(&ctx->nodes, other_id);
            if (pp_distance(ctx, other_point, pp_point(&ctx->nodes, new_id), &distance) != 0) { pp_set_error(result, "distance callback failed"); goto done; }
            if (distance <= ctx->options->goal_reach_tolerance) {
                valid = pp_valid_motion(ctx, other_point, pp_point(&ctx->nodes, new_id));
                if (valid < 0) { pp_set_error(result, "motion-validity callback failed"); goto done; }
                connected = valid != 0;
                break;
            }
            if (distance <= ctx->options->step_size) {
                valid = pp_valid_motion(ctx, other_point, pp_point(&ctx->nodes, new_id));
                if (valid < 0) { pp_set_error(result, "motion-validity callback failed"); goto done; }
                if (valid) connected = 1;
                else other_id = PP_C_NONE;
                break;
            }
            if (pp_steer(ctx, other_point, pp_point(&ctx->nodes, new_id), candidate, &edge_length) != 0) { pp_set_error(result, "RRT-Connect steering callback failed"); goto done; }
            valid = pp_valid_motion(ctx, other_point, candidate);
            if (valid < 0) { pp_set_error(result, "motion-validity callback failed"); goto done; }
            if (!valid) { other_id = PP_C_NONE; break; }
            other_id = pp_node_add(&ctx->nodes, candidate, other_id,
                                   ctx->nodes.cost[other_id] + edge_length, edge_length, (uint8_t)(1 - tree_id));
            if (other_id == PP_C_NONE || pp_add_index(ctx, other_index, other_id) != 0) { pp_set_error(result, "out of memory growing RRT-Connect tree"); goto done; }
        }
        ++ctx->iterations;
        if (connected && other_id != PP_C_NONE) {
            if (tree_id == 0) { start_join = new_id; goal_join = other_id; }
            else { start_join = other_id; goal_join = new_id; }
            break;
        }
        active_tree = 1 - active_tree;
    }
    if (start_join != PP_C_NONE && goal_join != PP_C_NONE) {
        uint32_t *first = (uint32_t *)malloc((size_t)ctx->nodes.count * sizeof(uint32_t));
        uint32_t *path = (uint32_t *)malloc(((size_t)ctx->nodes.count + 1) * sizeof(uint32_t));
        size_t n = 0, first_len;
        double bridge;
        if (first == NULL || path == NULL || pp_distance(ctx, pp_point(&ctx->nodes, start_join), pp_point(&ctx->nodes, goal_join), &bridge) != 0) {
            free(first); free(path); pp_set_error(result, "out of memory reconstructing RRT-Connect path"); goto done;
        }
        first_len = pp_reconstruct(&ctx->nodes, start_join, first);
        if (first_len == 0) { free(first); free(path); pp_set_error(result, "invalid RRT-Connect start tree"); goto done; }
        memcpy(path, first, first_len * sizeof(uint32_t)); n = first_len;
        if (bridge > ctx->options->goal_reach_tolerance) path[n++] = goal_join;
        {
            uint32_t current = ctx->nodes.parent[goal_join];
            while (current != PP_C_NONE && n <= ctx->nodes.count) {
                path[n++] = current; current = ctx->nodes.parent[current];
            }
        }
        {
            double total = ctx->nodes.cost[start_join] + bridge + ctx->nodes.cost[goal_join];
            status = pp_path_to_result(ctx, path, n, total, 0, result);
        }
        free(first); free(path); goto done;
    }
    if (stop != 1 && ctx->iterations >= ctx->options->max_iters) stop = 2;
    result->success = 0; result->stop_reason = stop; result->iters = ctx->iterations;
    result->nodes = ctx->nodes.count; result->sample_count = ctx->samples;
    result->motion_checks = ctx->motion_checks; result->elapsed_s = pp_now() - ctx->started;
    status = 0;
done:
    free(sample); free(candidate);
    return status;
}

int pp_continuous_plan(const pp_continuous_callbacks *callbacks, const double *start,
                       const double *goal, size_t dimension, int has_goal_point,
                       const pp_continuous_options *options, pp_continuous_result *result) {
    pp_context ctx;
    int valid, status = -1;
    if (result == NULL) return -1;
    memset(result, 0, sizeof(*result)); result->stop_reason = 4;
    if (callbacks == NULL || options == NULL || start == NULL || dimension == 0 ||
        (callbacks->native_space == NULL && (callbacks->sample_free == NULL || callbacks->state_valid == NULL ||
         callbacks->motion_valid == NULL || callbacks->distance == NULL || callbacks->steer == NULL)) ||
        (!callbacks->native_goal && callbacks->is_goal == NULL) ||
        (has_goal_point && goal == NULL)) {
        pp_set_error(result, "continuous planner callbacks, states, or options are invalid"); return -1;
    }
    if (callbacks->native_goal && (!has_goal_point || !isfinite(callbacks->goal_radius) || callbacks->goal_radius < 0.0)) {
        pp_set_error(result, "native goal requires a point goal and a finite non-negative radius"); return -1;
    }
    if (options->algorithm < PP_CONTINUOUS_RRT || options->algorithm > PP_CONTINUOUS_RRT_CONNECT ||
        options->max_iters == 0 || options->max_sample_tries == 0 ||
        !(options->step_size > 0.0) || !(options->collision_step > 0.0) ||
        !(options->goal_sample_rate >= 0.0 && options->goal_sample_rate <= 1.0) ||
        dimension > 1024) {
        pp_set_error(result, "continuous planner options are invalid"); return -1;
    }
    if ((options->algorithm == PP_CONTINUOUS_FMT_STAR || options->algorithm == PP_CONTINUOUS_BIT_STAR ||
         options->algorithm == PP_CONTINUOUS_ABIT_STAR || options->algorithm == PP_CONTINUOUS_INFORMED_RRT_STAR) &&
        (!has_goal_point || options->sample_count == 0 || options->batch_size == 0)) {
        pp_set_error(result, "informed sampling planners require an exact goal and positive samples"); return -1;
    }
    if (options->algorithm == PP_CONTINUOUS_RRT_CONNECT && !has_goal_point) {
        pp_set_error(result, "RRT-Connect requires an exact goal state"); return -1;
    }
    memset(&ctx, 0, sizeof(ctx));
    ctx.callbacks = callbacks; ctx.options = options; ctx.dimension = dimension;
    ctx.has_goal_point = has_goal_point; ctx.goal = goal; ctx.started = pp_now();
    ctx.nodes.dimension = dimension;
    pp_rng_seed(&ctx.rng, options->seed);
    ctx.indices[0].dimension = dimension; ctx.indices[1].dimension = dimension;
    valid = pp_valid_state(&ctx, start);
    if (valid < 0) { pp_set_error(result, "state-validity callback failed for start"); goto done; }
    if (!valid) { pp_set_error(result, "start must be collision free"); goto done; }
    if (has_goal_point) {
        valid = pp_valid_state(&ctx, goal);
        if (valid < 0) { pp_set_error(result, "state-validity callback failed for goal"); goto done; }
        if (!valid) { pp_set_error(result, "goal must be collision free"); goto done; }
    }
    if (pp_nodes_reserve(&ctx.nodes, 2) != 0) { pp_set_error(result, "out of memory creating planner state"); goto done; }
    memcpy(ctx.nodes.points, start, dimension * sizeof(double));
    if (has_goal_point && options->algorithm != PP_CONTINUOUS_RRT &&
        options->algorithm != PP_CONTINUOUS_RRT_STAR && options->algorithm != PP_CONTINUOUS_INFORMED_RRT_STAR &&
        options->algorithm != PP_CONTINUOUS_RRT_CONNECT) {
        double distance;
        if (pp_distance(&ctx, start, goal, &distance) != 0) { pp_set_error(result, "distance callback failed"); goto done; }
        if (distance <= options->goal_reach_tolerance) {
            uint32_t only = pp_node_add(&ctx.nodes, start, PP_C_NONE, 0.0, 0.0, 0);
            status = pp_record_path(&ctx, only, result); goto done;
        }
    }
    switch (options->algorithm) {
        case PP_CONTINUOUS_RRT: status = pp_run_rrt(&ctx, 0, 0, result); break;
        case PP_CONTINUOUS_RRT_STAR: status = pp_run_rrt(&ctx, 1, 0, result); break;
        case PP_CONTINUOUS_INFORMED_RRT_STAR: status = pp_run_rrt(&ctx, 1, 1, result); break;
        case PP_CONTINUOUS_FMT_STAR: status = pp_run_fmt(&ctx, result); break;
        case PP_CONTINUOUS_BIT_STAR: status = pp_run_bit(&ctx, 0, result); break;
        case PP_CONTINUOUS_ABIT_STAR: status = pp_run_bit(&ctx, 1, result); break;
        case PP_CONTINUOUS_RRT_CONNECT: status = pp_run_connect(&ctx, result); break;
        default: pp_set_error(result, "unknown continuous algorithm"); status = -1; break;
    }
    if (status < 0 && result->error_message == NULL) pp_set_error(result, "native continuous planner failed");
    if (result->success) result->elapsed_s = pp_now() - ctx.started;
done:
    pp_kd_free(&ctx.indices[0]); pp_kd_free(&ctx.indices[1]); pp_nodes_free(&ctx.nodes);
    return status;
}

static int pp_dynamic_return_path(pp_context *ctx, uint32_t goal_id,
                                  pp_dynamic_rrt_result *result) {
    uint32_t *ids = (uint32_t *)malloc((size_t)ctx->nodes.count * sizeof(uint32_t));
    size_t length;
    int status;
    if (ids == NULL) return -1;
    length = pp_reconstruct(&ctx->nodes, goal_id, ids);
    status = length == 0 ? -1 : pp_path_to_result(ctx, ids, length,
                                                   ctx->nodes.cost[goal_id], 0,
                                                   &result->plan);
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

int pp_dynamic_rrt_plan(const pp_continuous_callbacks *callbacks,
                        const double *start, const double *goal,
                        const double *initial_points,
                        const uint32_t *initial_parents,
                        size_t initial_count, size_t dimension,
                        const pp_continuous_options *options,
                        double waypoint_sample_rate,
                        int prune_only,
                        pp_dynamic_rrt_result *result) {
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
    ctx.indices[0].dimension = dimension;
    pp_rng_seed(&ctx.rng, options->seed);
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
        if (!valid) continue;
        keep[i] = 1;
        id = pp_node_add(&ctx.nodes, initial_points + i * dimension, parent,
                         i == 0 ? 0.0 : ctx.nodes.cost[parent] + edge_cost,
                         edge_cost, 0);
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
        if (pp_dynamic_return_path(&ctx, goal_id, result) != 0) {
            pp_set_error(&result->plan, "failed to reconstruct existing dynamic RRT path");
            goto done;
        }
    } else if (!prune_only) {
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
            if (pp_nearest(&ctx, &ctx.indices[0], sample, &nearest) != 0 ||
                pp_steer(&ctx, pp_point(&ctx.nodes, nearest), sample, candidate, &edge_cost) != 0 ||
                pp_nearest(&ctx, &ctx.indices[0], candidate, &duplicate) != 0) {
                pp_set_error(&result->plan, "nearest-node query or steering failed in dynamic RRT");
                goto done;
            }
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
        if (goal_id != PP_C_NONE && pp_dynamic_return_path(&ctx, goal_id, result) != 0) {
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

void pp_continuous_free_result(pp_continuous_result *result) {
    if (result == NULL) return;
    free(result->path); free(result->error_message); memset(result, 0, sizeof(*result));
}

void pp_dynamic_rrt_free_result(pp_dynamic_rrt_result *result) {
    if (result == NULL) return;
    pp_continuous_free_result(&result->plan);
    free(result->tree_points);
    free(result->tree_parents);
    free(result->invalid_nodes);
    memset(result, 0, sizeof(*result));
}

const char *pp_continuous_engine_version(void) { return PP_C_VERSION; }

uint32_t pp_continuous_abi_version(void) { return PP_CONTINUOUS_ABI_VERSION; }
