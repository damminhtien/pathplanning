# Native ABI, ownership, and errors

This page is the contract for the native search and sampling engines. The
package version and each engine's implementation version are independent of
its C ABI version.

## ABI versions and loading

| Library | ABI constant | Probe function | Supported ABI |
| --- | --- | --- | --- |
| C++ graph search | `PP_SEARCH_ABI_VERSION` | `pp_search_abi_version()` | 5 |
| C sampling planners | `PP_CONTINUOUS_ABI_VERSION` | `pp_continuous_abi_version()` | 1 |
| C++ diagnostic search | `PP_SEARCH_TRACE_ABI_VERSION` | `pp_search_trace_abi_version()` | 1 |
| C++ diagnostic search metrics | `PP_SEARCH_METRICS_ABI_VERSION` | `pp_search_metrics_abi_version()` | 2 |
| C diagnostic sampling | `PP_CONTINUOUS_TRACE_ABI_VERSION` | `pp_continuous_trace_abi_version()` | 1 |

The constants are declared in `pathplanning/native/abi_version.h`. Increment
an engine's ABI version when an exported function, struct layout, enum value,
or callback contract changes. ABI versions require an exact match; PathPlanning
does not load an older ABI or provide a compatibility shim. The `*_engine_version()`
functions report implementation versions and do not establish ABI compatibility.

The Python loader checks the ABI probe before configuring or calling the rest
of the library. A missing probe, version mismatch, or missing required export
raises `NativeLibraryCompatibilityError` with the library path and rebuild
instruction. A shared-library load failure raises `NativeLibraryLoadError`.
Both errors fail closed; rebuild or reinstall the package with `make build-ext`.
The diagnostic libraries are loaded only for trace or explicit search-metrics
collection. Search ABI 3 removed the callback-based `pp_search_plan()` and
`pp_astar_plan()` entrypoints. Search ABI 4 adds graph storage inspection and
idempotent reverse-CSR preparation, and exposes a borrowed CSR view for cloning
a graph into diagnostic libraries. Continuous production ABI did not change.
Search ABI 5 adds the `reexp_astar` search algorithm, its conditional reopen
options and runtime budget, and a reopen count in `pp_search_result`. Metrics
ABI 2 adds the same reopen count and reports whether the selected algorithm
supports re-expansion.

The production `_search_engine` is built without `PP_ENABLE_METRICS`; metric
macros compile to no-ops and search containers use the normal
`std::vector`/`std::deque` allocators. `_search_metrics_engine` is a separate
extension built from its own object directory with `PP_ENABLE_METRICS=1` and
exports the metrics ABI only. The benchmark collects work counters from that
extension in a separate pass and measures latency through `_search_engine`.
This isolates counter and allocation-tracking overhead from production timing.

## Ownership and lifetime

| Resource | Owner and lifetime | Release rule |
| --- | --- | --- |
| `pp_native_graph*` | Caller owns the opaque handle returned by a successful graph-create call. The engine copies the CSR/grid input arrays before returning. | Call `pp_graph_free()` once. `NULL` is accepted. |
| Search input arrays and graph | Borrowed for the duration of `pp_native_search_plan()`. The graph remains caller-owned. | Keep all inputs alive until the call returns. |
| `pp_search_result` buffers | Result storage belongs to the caller; `path_ids` and `error_message` are allocated by the native library on success or failure. | Call `pp_search_free_result()` after every call and before reusing the result storage. It frees buffers and resets the struct. |
| Continuous inputs, model arrays, callbacks, and `user_data` | Borrowed for the duration of `pp_continuous_plan()` or `pp_dynamic_rrt_plan()`. The planner retains no callback or input pointer after return. | Keep them alive until the call returns. |
| `pp_continuous_result` buffers | Result storage belongs to the caller; `path` and `error_message` belong to the native library. | Call `pp_continuous_free_result()` after every call and before reuse. |
| `pp_dynamic_rrt_result` buffers | The nested plan buffers and tree/invalid-node arrays belong to the native library. | Call `pp_dynamic_rrt_free_result()` after every call and before reuse; do not free nested fields separately. |
| `pp_graph_csr_view` | Borrowed pointers to a production graph's CSR arrays and grid metadata. | Keep the production graph alive until `pp_graph_create_csr_view()` copies them into the diagnostic library; never share opaque graph handles between libraries. |
| `pp_trace_result` buffers | Diagnostic library owns event and coordinate arrays. The recording cap includes their allocated capacities and, for continuous planners, the node-ID map. | Copy into Python-owned arrays, then call `pp_search_trace_free_result()` or `pp_continuous_trace_free_result()` from the producing library. |

Free functions accept `NULL` and reset a valid result struct after freeing it.
Callers must use the matching free function from the same engine library and
must not free individual result fields. Search and continuous outputs are
value structs, not opaque result handles.

Diagnostic recording stops at its byte limit or allocation failure and marks
`truncated`; planning continues. The extra diagnostic graph copy is reported
separately from the trace cap as the graph struct plus allocated CSR-vector
capacities; allocator bookkeeping is not included. The production libraries
contain no trace event recording in their planner loops.

The C++ search implementation uses RAII for internal graph/search state and
catches exceptions at every exported C boundary. No C++ exception crosses the
C ABI. PathPlanning has no separate public C++ SDK today; the C++ engine is an
implementation behind the C interface.

## Callback failures

Discrete search has no callback-based C entrypoint. The Python adapter converts
custom graph protocols to native CSR before calling `pp_native_search_plan()`;
neighbors, edge costs, goals, and heuristics are prepared before native search
starts. No Python call occurs while the C++ search loop expands nodes.

Continuous callbacks and their `user_data` are borrowed for the native call.
Input pointers are valid only during each callback invocation.

Continuous callbacks use these rules:

| Callback | Return rule |
| --- | --- |
| `sample_free`, `steer` | `0` means success; nonzero means callback failure. Output state is read only on success. |
| `state_valid`, `motion_valid`, `is_goal` | `0` means false, `1` means true, and a negative value or a value greater than `1` means callback failure. |
| `distance` | Must return a finite, non-negative value. A negative or non-finite value fails the plan. |
| `goal_distance` | Must return a non-negative value. Positive infinity means that no usable estimate is available; NaN or a negative value fails the plan. |
| `path_objective` | Must return a finite value. A non-finite value fails the plan. |

The native result reports a callback failure through nonzero status and,
when allocation succeeds, `error_message`. Planning outcomes such as timeout
or no path are represented by `success` and `stop_reason`; they are not ABI
call errors. Python callback adapters catch the original exception because
`ctypes` cannot unwind it through C, save the first exception, return the
failure sentinel above, and skip later Python callback invocations. After the
native call returns, Python frees the native result and re-raises that original
exception. Direct C callers receive the status/result error and can preserve
their own exception-like detail through `user_data`.

## Language bindings

Python uses `ctypes` declarations matching these C structs. `NativeGraph` owns
its graph handle and supports explicit close/context-manager cleanup, with a
finalizer as a fallback. Planner adapters copy result arrays into Python-owned
memory before calling the matching native result-free function.

PathPlanning currently has no Java/JNI binding. Any future Java binding must
verify the exact ABI before calls, wrap native graph handles in `AutoCloseable`
owners, copy result data into Java-owned values, map C status/result errors to
Java exceptions, and never depend on private C++ object layouts.
