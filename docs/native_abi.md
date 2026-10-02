# Native ABI, ownership, and errors

This page is the contract for the native search and sampling engines. The
package version and each engine's implementation version are independent of
its C ABI version.

## ABI versions and loading

| Library | ABI constant | Probe function | Supported ABI |
| --- | --- | --- | --- |
| C++ graph search | `PP_SEARCH_ABI_VERSION` | `pp_search_abi_version()` | 1 |
| C sampling planners | `PP_CONTINUOUS_ABI_VERSION` | `pp_continuous_abi_version()` | 1 |

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

## Ownership and lifetime

| Resource | Owner and lifetime | Release rule |
| --- | --- | --- |
| `pp_native_graph*` | Caller owns the opaque handle returned by a successful graph-create call. The engine copies the CSR/grid input arrays before returning. | Call `pp_graph_free()` once. `NULL` is accepted. |
| Search input arrays and graph | Borrowed for the duration of `pp_search_plan()` or `pp_native_search_plan()`. The graph remains caller-owned. | Keep all inputs alive until the call returns. |
| `pp_search_result` buffers | Result storage belongs to the caller; `path_ids` and `error_message` are allocated by the native library on success or failure. | Call `pp_search_free_result()` after every call and before reusing the result storage. It frees buffers and resets the struct. |
| Continuous inputs, model arrays, callbacks, and `user_data` | Borrowed for the duration of `pp_continuous_plan()` or `pp_dynamic_rrt_plan()`. The planner retains no callback or input pointer after return. | Keep them alive until the call returns. |
| `pp_continuous_result` buffers | Result storage belongs to the caller; `path` and `error_message` belong to the native library. | Call `pp_continuous_free_result()` after every call and before reuse. |
| `pp_dynamic_rrt_result` buffers | The nested plan buffers and tree/invalid-node arrays belong to the native library. | Call `pp_dynamic_rrt_free_result()` after every call and before reuse; do not free nested fields separately. |

Free functions accept `NULL` and reset a valid result struct after freeing it.
Callers must use the matching free function from the same engine library and
must not free individual result fields. Search and continuous outputs are
value structs, not opaque result handles.

The C++ search implementation uses RAII for internal graph/search state and
catches exceptions at every exported C boundary. No C++ exception crosses the
C ABI. PathPlanning has no separate public C++ SDK today; the C++ engine is an
implementation behind the C interface.

## Callback failures

Callbacks and their `user_data` are borrowed for the native call. The
graph-neighbors callback's returned arrays must stay valid until the next
callback invocation; the engine copies them into the graph snapshot before
then. Other callback input pointers are valid only during that callback.

Graph-search callbacks use one status convention: return `0` for success and a
nonzero value for failure. The goal callback writes exactly `0` or `1` to its
output. Other output arguments are read only after success. A
callback failure aborts graph snapshotting; the native call returns nonzero
and, when allocation succeeds, the result contains an `error_message`. The C
ABI does not carry a callback's private diagnostic string; a C caller can keep
richer detail in its own `user_data`.

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
