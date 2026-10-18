#ifndef PATHPLANNING_NATIVE_CONTINUOUS_COLLISION_H_
#define PATHPLANNING_NATIVE_CONTINUOUS_COLLISION_H_

#include <stddef.h>

#include "continuous_engine.h"

typedef struct pp_collision_index pp_collision_index;

int pp_collision_index_create(const pp_continuous_space_model *model,
                              size_t dimension,
                              pp_collision_index **out_index);
void pp_collision_index_free(pp_collision_index *index);
int pp_collision_state_valid(const pp_collision_index *index,
                             const pp_continuous_space_model *model,
                             const double *state, size_t dimension);
int pp_collision_motion_valid(const pp_collision_index *index,
                              const pp_continuous_space_model *model,
                              const double *start, const double *end,
                              size_t dimension);

#endif
