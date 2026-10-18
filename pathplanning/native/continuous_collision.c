#include "continuous_collision.h"

#include <math.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>

#define PP_COLLISION_LEAF_SIZE 4

typedef struct {
    double *minimum;
    double *maximum;
    size_t first;
    size_t count;
    size_t left;
    size_t right;
} pp_collision_node;

struct pp_collision_index {
    size_t dimension;
    size_t obstacle_count;
    size_t node_count;
    size_t *obstacle_ids;
    double *obstacle_minima;
    double *obstacle_maxima;
    pp_collision_node *nodes;
    double *node_bounds;
};

static int pp_model_arrays_valid(const pp_continuous_space_model *model,
                                 size_t dimension) {
    return model != NULL && model->lower_bounds != NULL &&
           model->upper_bounds != NULL &&
           (model->box_count == 0 ||
            (model->box_minima != NULL && model->box_maxima != NULL)) &&
           (model->sphere_count == 0 ||
            (model->sphere_centers != NULL && model->sphere_radii != NULL)) &&
           (model->obb_count == 0 ||
            (dimension == 3 && model->obb_centers != NULL &&
             model->obb_extents != NULL && model->obb_orientations != NULL));
}

static int pp_size_product(size_t a, size_t b, size_t *out) {
    if (a != 0 && b > SIZE_MAX / a) return -1;
    *out = a * b;
    return 0;
}

static size_t pp_obstacle_bound_offset(const pp_collision_index *index,
                                      size_t obstacle_id) {
    return obstacle_id * index->dimension;
}

static double pp_obstacle_center(const pp_collision_index *index,
                                 size_t obstacle_id, size_t axis) {
    size_t offset = pp_obstacle_bound_offset(index, obstacle_id) + axis;
    return (index->obstacle_minima[offset] + index->obstacle_maxima[offset]) * 0.5;
}

static int pp_obstacle_less(const pp_collision_index *index, size_t first,
                            size_t second, size_t axis) {
    double a = pp_obstacle_center(index, first, axis);
    double b = pp_obstacle_center(index, second, axis);
    return a < b || (a == b && first < second);
}

static void pp_select_median(pp_collision_index *index, ptrdiff_t low,
                             ptrdiff_t high, ptrdiff_t nth, size_t axis) {
    while (low < high) {
        size_t pivot = index->obstacle_ids[low + (high - low) / 2];
        ptrdiff_t left = low;
        ptrdiff_t right = high;
        while (left <= right) {
            while (pp_obstacle_less(index, index->obstacle_ids[left], pivot, axis))
                ++left;
            while (pp_obstacle_less(index, pivot, index->obstacle_ids[right], axis))
                --right;
            if (left <= right) {
                size_t temporary = index->obstacle_ids[left];
                index->obstacle_ids[left] = index->obstacle_ids[right];
                index->obstacle_ids[right] = temporary;
                ++left;
                --right;
            }
        }
        if (nth <= right) high = right;
        else if (nth >= left) low = left;
        else return;
    }
}

static int pp_overlap(const double *first_minimum, const double *first_maximum,
                      const double *second_minimum, const double *second_maximum,
                      size_t dimension) {
    size_t axis;
    for (axis = 0; axis < dimension; ++axis) {
        if (first_maximum[axis] < second_minimum[axis] ||
            second_maximum[axis] < first_minimum[axis]) return 0;
    }
    return 1;
}

static size_t pp_build_node(pp_collision_index *index, size_t first,
                            size_t count) {
    size_t node_id = index->node_count++;
    pp_collision_node *node = &index->nodes[node_id];
    size_t axis, i;

    node->first = first;
    node->count = count;
    node->left = SIZE_MAX;
    node->right = SIZE_MAX;
    node->minimum = index->node_bounds + node_id * 2 * index->dimension;
    node->maximum = node->minimum + index->dimension;
    for (axis = 0; axis < index->dimension; ++axis) {
        node->minimum[axis] = INFINITY;
        node->maximum[axis] = -INFINITY;
        for (i = first; i < first + count; ++i) {
            size_t obstacle_id = index->obstacle_ids[i];
            size_t offset = pp_obstacle_bound_offset(index, obstacle_id) + axis;
            node->minimum[axis] = fmin(node->minimum[axis], index->obstacle_minima[offset]);
            node->maximum[axis] = fmax(node->maximum[axis], index->obstacle_maxima[offset]);
        }
    }
    if (count <= PP_COLLISION_LEAF_SIZE) return node_id;

    axis = 0;
    for (i = 1; i < index->dimension; ++i) {
        if (node->maximum[i] - node->minimum[i] >
            node->maximum[axis] - node->minimum[axis]) axis = i;
    }
    {
        size_t left_count = count / 2;
        ptrdiff_t low = (ptrdiff_t)first;
        ptrdiff_t high = (ptrdiff_t)(first + count - 1);
        ptrdiff_t middle = low + (ptrdiff_t)left_count - 1;
        pp_select_median(index, low, high, middle, axis);
        node->count = 0;
        node->left = pp_build_node(index, first, left_count);
        node->right = pp_build_node(index, first + left_count, count - left_count);
    }
    return node_id;
}

static void pp_set_obstacle_bounds(pp_collision_index *index,
                                   const pp_continuous_space_model *model,
                                   size_t obstacle_id) {
    size_t dimension = index->dimension;
    size_t offset = pp_obstacle_bound_offset(index, obstacle_id);
    size_t axis;
    if (obstacle_id < model->box_count) {
        for (axis = 0; axis < dimension; ++axis) {
            index->obstacle_minima[offset + axis] =
                model->box_minima[obstacle_id * dimension + axis];
            index->obstacle_maxima[offset + axis] =
                model->box_maxima[obstacle_id * dimension + axis];
        }
        return;
    }
    if (obstacle_id < model->box_count + model->sphere_count) {
        size_t sphere_id = obstacle_id - model->box_count;
        double radius = model->sphere_radii[sphere_id];
        for (axis = 0; axis < dimension; ++axis) {
            double center = model->sphere_centers[sphere_id * dimension + axis];
            index->obstacle_minima[offset + axis] = center - radius;
            index->obstacle_maxima[offset + axis] = center + radius;
        }
        return;
    }
    {
        size_t obb_id = obstacle_id - model->box_count - model->sphere_count;
        for (axis = 0; axis < 3; ++axis) {
            size_t local_axis;
            double radius = 0.0;
            double center = model->obb_centers[obb_id * 3 + axis];
            for (local_axis = 0; local_axis < 3; ++local_axis) {
                radius += fabs(model->obb_orientations[obb_id * 9 + axis * 3 + local_axis]) *
                          model->obb_extents[obb_id * 3 + local_axis];
            }
            index->obstacle_minima[offset + axis] = center - radius;
            index->obstacle_maxima[offset + axis] = center + radius;
        }
    }
}

int pp_collision_index_create(const pp_continuous_space_model *model,
                              size_t dimension,
                              pp_collision_index **out_index) {
    pp_collision_index *index = NULL;
    size_t count, bound_values, node_capacity, node_bound_values, i;
    if (out_index == NULL) return -1;
    *out_index = NULL;
    if (!pp_model_arrays_valid(model, dimension) || dimension == 0 || dimension > 1024 ||
        model->box_count > SIZE_MAX - model->sphere_count ||
        model->box_count + model->sphere_count > SIZE_MAX - model->obb_count) return -1;
    count = model->box_count + model->sphere_count + model->obb_count;
    index = (pp_collision_index *)calloc(1, sizeof(*index));
    if (index == NULL) return -1;
    index->dimension = dimension;
    index->obstacle_count = count;
    if (count == 0) {
        *out_index = index;
        return 0;
    }
    if (pp_size_product(count, dimension, &bound_values) != 0 ||
        bound_values > SIZE_MAX / sizeof(double) ||
        count > SIZE_MAX / 2 || count > SIZE_MAX / sizeof(size_t) ||
        count * 2 > SIZE_MAX / sizeof(pp_collision_node) ||
        count * 2 > (size_t)PTRDIFF_MAX ||
        pp_size_product(count * 2, dimension, &node_bound_values) != 0 ||
        node_bound_values > SIZE_MAX / sizeof(double)) goto fail;
    node_capacity = count * 2;
    index->obstacle_ids = (size_t *)malloc(count * sizeof(size_t));
    index->obstacle_minima = (double *)malloc(bound_values * sizeof(double));
    index->obstacle_maxima = (double *)malloc(bound_values * sizeof(double));
    index->nodes = (pp_collision_node *)calloc(node_capacity, sizeof(pp_collision_node));
    index->node_bounds = (double *)malloc(node_bound_values * sizeof(double));
    if (index->obstacle_ids == NULL || index->obstacle_minima == NULL ||
        index->obstacle_maxima == NULL || index->nodes == NULL ||
        index->node_bounds == NULL) goto fail;
    for (i = 0; i < count; ++i) {
        index->obstacle_ids[i] = i;
        pp_set_obstacle_bounds(index, model, i);
    }
    (void)pp_build_node(index, 0, count);
    *out_index = index;
    return 0;

fail:
    pp_collision_index_free(index);
    return -1;
}

void pp_collision_index_free(pp_collision_index *index) {
    if (index == NULL) return;
    free(index->obstacle_ids);
    free(index->obstacle_minima);
    free(index->obstacle_maxima);
    free(index->nodes);
    free(index->node_bounds);
    free(index);
}

static int pp_state_in_bounds(const pp_continuous_space_model *model,
                              const double *state, size_t dimension) {
    size_t axis;
    for (axis = 0; axis < dimension; ++axis) {
        if (!isfinite(state[axis]) || state[axis] < model->lower_bounds[axis] ||
            state[axis] > model->upper_bounds[axis]) return 0;
    }
    return 1;
}

static int pp_state_hits_obstacle(const pp_collision_index *index,
                                  const pp_continuous_space_model *model,
                                  size_t obstacle_id, const double *state) {
    size_t dimension = index->dimension;
    size_t axis;
    if (obstacle_id < model->box_count) {
        for (axis = 0; axis < dimension; ++axis) {
            size_t at = obstacle_id * dimension + axis;
            if (state[axis] < model->box_minima[at] ||
                state[axis] > model->box_maxima[at]) return 0;
        }
        return 1;
    }
    if (obstacle_id < model->box_count + model->sphere_count) {
        size_t sphere_id = obstacle_id - model->box_count;
        double distance2 = 0.0;
        double radius = model->sphere_radii[sphere_id];
        for (axis = 0; axis < dimension; ++axis) {
            double delta = state[axis] - model->sphere_centers[sphere_id * dimension + axis];
            distance2 += delta * delta;
        }
        return distance2 <= radius * radius;
    }
    {
        size_t obb_id = obstacle_id - model->box_count - model->sphere_count;
        size_t local_axis, row;
        for (local_axis = 0; local_axis < 3; ++local_axis) {
            double local = 0.0;
            for (row = 0; row < 3; ++row) {
                size_t center_at = obb_id * 3 + row;
                size_t orientation_at = obb_id * 9 + row * 3 + local_axis;
                local += model->obb_orientations[orientation_at] *
                         (state[row] - model->obb_centers[center_at]);
            }
            if (fabs(local) > model->obb_extents[obb_id * 3 + local_axis]) return 0;
        }
        return 1;
    }
}

static int pp_state_hits_node(const pp_collision_index *index,
                              const pp_continuous_space_model *model,
                              size_t node_id, const double *state) {
    const pp_collision_node *node = &index->nodes[node_id];
    size_t i;
    if (!pp_overlap(state, state, node->minimum, node->maximum, index->dimension))
        return 0;
    if (node->count != 0) {
        for (i = node->first; i < node->first + node->count; ++i) {
            if (pp_state_hits_obstacle(index, model, index->obstacle_ids[i], state))
                return 1;
        }
        return 0;
    }
    return pp_state_hits_node(index, model, node->left, state) ||
           pp_state_hits_node(index, model, node->right, state);
}

int pp_collision_state_valid(const pp_collision_index *index,
                             const pp_continuous_space_model *model,
                             const double *state, size_t dimension) {
    if (index == NULL || state == NULL || index->dimension != dimension ||
        !pp_model_arrays_valid(model, dimension)) return -1;
    if (!pp_state_in_bounds(model, state, dimension)) return 0;
    if (index->obstacle_count == 0) return 1;
    return !pp_state_hits_node(index, model, 0, state);
}

static int pp_segment_intersects_aabb(const double *start, const double *end,
                                      const double *minimum, const double *maximum,
                                      size_t dimension) {
    double first = 0.0;
    double last = 1.0;
    size_t axis;
    for (axis = 0; axis < dimension; ++axis) {
        double delta = end[axis] - start[axis];
        double enter, leave;
        if (delta == 0.0) {
            if (start[axis] < minimum[axis] || start[axis] > maximum[axis]) return 0;
            continue;
        }
        enter = (minimum[axis] - start[axis]) / delta;
        leave = (maximum[axis] - start[axis]) / delta;
        if (enter > leave) {
            double temporary = enter;
            enter = leave;
            leave = temporary;
        }
        first = fmax(first, enter);
        last = fmin(last, leave);
        if (first > last) return 0;
    }
    return 1;
}

static int pp_segment_hits_obstacle(const pp_collision_index *index,
                                    const pp_continuous_space_model *model,
                                    size_t obstacle_id, const double *start,
                                    const double *end) {
    size_t dimension = index->dimension;
    size_t axis;
    if (obstacle_id < model->box_count) {
        size_t offset = obstacle_id * dimension;
        return pp_segment_intersects_aabb(start, end,
            model->box_minima + offset, model->box_maxima + offset, dimension);
    }
    if (obstacle_id < model->box_count + model->sphere_count) {
        size_t sphere_id = obstacle_id - model->box_count;
        const double *center = model->sphere_centers + sphere_id * dimension;
        double radius = model->sphere_radii[sphere_id];
        double length2 = 0.0, projection = 0.0, distance2 = 0.0;
        for (axis = 0; axis < dimension; ++axis) {
            double delta = end[axis] - start[axis];
            length2 += delta * delta;
            projection += (center[axis] - start[axis]) * delta;
        }
        projection = length2 == 0.0 ? 0.0 : projection / length2;
        projection = fmax(0.0, fmin(1.0, projection));
        for (axis = 0; axis < dimension; ++axis) {
            double closest = start[axis] + projection * (end[axis] - start[axis]);
            double delta = closest - center[axis];
            distance2 += delta * delta;
        }
        return distance2 <= radius * radius;
    }
    {
        size_t obb_id = obstacle_id - model->box_count - model->sphere_count;
        double local_start[3], local_end[3], minimum[3], maximum[3];
        size_t local_axis, row;
        for (local_axis = 0; local_axis < 3; ++local_axis) {
            double start_value = 0.0, end_value = 0.0;
            for (row = 0; row < 3; ++row) {
                size_t center_at = obb_id * 3 + row;
                size_t orientation_at = obb_id * 9 + row * 3 + local_axis;
                double orientation = model->obb_orientations[orientation_at];
                start_value += orientation * (start[row] - model->obb_centers[center_at]);
                end_value += orientation * (end[row] - model->obb_centers[center_at]);
            }
            local_start[local_axis] = start_value;
            local_end[local_axis] = end_value;
            minimum[local_axis] = -model->obb_extents[obb_id * 3 + local_axis];
            maximum[local_axis] = model->obb_extents[obb_id * 3 + local_axis];
        }
        return pp_segment_intersects_aabb(local_start, local_end, minimum, maximum, 3);
    }
}

static int pp_segment_hits_node(const pp_collision_index *index,
                                const pp_continuous_space_model *model,
                                size_t node_id, const double *start,
                                const double *end, const double *minimum,
                                const double *maximum) {
    const pp_collision_node *node = &index->nodes[node_id];
    size_t i;
    if (!pp_overlap(minimum, maximum, node->minimum, node->maximum, index->dimension))
        return 0;
    if (node->count != 0) {
        for (i = node->first; i < node->first + node->count; ++i) {
            if (pp_segment_hits_obstacle(index, model, index->obstacle_ids[i], start, end))
                return 1;
        }
        return 0;
    }
    return pp_segment_hits_node(index, model, node->left, start, end, minimum, maximum) ||
           pp_segment_hits_node(index, model, node->right, start, end, minimum, maximum);
}

int pp_collision_motion_valid(const pp_collision_index *index,
                              const pp_continuous_space_model *model,
                              const double *start, const double *end,
                              size_t dimension) {
    size_t axis;
    if (index == NULL || start == NULL || end == NULL || index->dimension != dimension ||
        !pp_model_arrays_valid(model, dimension)) return -1;
    if (!pp_state_in_bounds(model, start, dimension) ||
        !pp_state_in_bounds(model, end, dimension)) return 0;
    if (index->obstacle_count == 0) return 1;
    if (dimension <= 3) {
        double minimum[3], maximum[3];
        for (axis = 0; axis < dimension; ++axis) {
            minimum[axis] = fmin(start[axis], end[axis]);
            maximum[axis] = fmax(start[axis], end[axis]);
        }
        return !pp_segment_hits_node(index, model, 0, start, end, minimum, maximum);
    }
    {
        double minimum[dimension], maximum[dimension];
        for (axis = 0; axis < dimension; ++axis) {
            minimum[axis] = fmin(start[axis], end[axis]);
            maximum[axis] = fmax(start[axis], end[axis]);
        }
        return !pp_segment_hits_node(index, model, 0, start, end,
                                     minimum, maximum);
    }
}
