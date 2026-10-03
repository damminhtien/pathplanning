#ifndef PATHPLANNING_NATIVE_TRACE_ENGINE_H_
#define PATHPLANNING_NATIVE_TRACE_ENGINE_H_

#include <stddef.h>
#include <stdint.h>

// Shared diagnostic ABI. Production planning functions never use these types.
typedef enum pp_trace_kind {
    PP_TRACE_DISCOVER = 1,
    PP_TRACE_EXPAND = 2,
    PP_TRACE_PARENT = 3,
    PP_TRACE_SAMPLE = 4,
    PP_TRACE_PHASE = 5,
    PP_TRACE_SOLUTION = 6,
    PP_TRACE_PRUNE = 7
} pp_trace_kind;

typedef struct pp_trace_event {
    uint64_t node;
    uint64_t parent;
    double value;
    uint32_t kind;
    uint32_t side;
} pp_trace_event;

typedef struct pp_trace_result {
    pp_trace_event *events;
    size_t event_count;
    double *points;
    size_t point_count;
    size_t dimension;
    int truncated;
} pp_trace_result;

#endif  // PATHPLANNING_NATIVE_TRACE_ENGINE_H_
