# Kế hoạch đánh giá single-agent shortest path

Ngày review: 2026-10-04. Phiên bản kế hoạch: 2.

Trạng thái: đề xuất triển khai. Tài liệu này thay thế kế hoạch trong cuộc trao đổi, chốt định nghĩa phép đo, đầu việc và tiêu chí nghiệm thu. Các file/lệnh được đánh dấu “dự kiến” chưa được cài đặt. Tài liệu không chứa kết quả benchmark mới.

## 1. Kết quả review và các quyết định cần sửa

| Điểm trong kế hoạch trước | Quyết định sau review |
| --- | --- |
| Dùng grid hiện tại cho MovingAI | Tạo CSR benchmark đúng luật no-corner-cutting từ occupancy. Grid hiện tại chỉ kiểm tra ô nguồn/đích và có thể cho optimum khác scenario. |
| Dùng Dijkstra native làm oracle | Dùng reference Dijkstra độc lập trên occupancy gốc. Dijkstra native là một thuật toán được đánh giá. |
| graph_init_s là thời gian tạo graph | Trường hiện tại gồm adapter, ánh xạ query và chuẩn bị heuristic; graph có thể được tái sử dụng. Đo các ranh giới riêng. |
| Suy memory từ expanded | SearchStates cấp phát mảng toàn bộ node slots. Ghi slots, capacity và allocations đang sống. |
| Counter reopen/re-expansion áp dụng cho mọi kernel | Kernel hiện bỏ qua CLOSED và chưa hỗ trợ reopen. Ghi capability và null cho metric không áp dụng. Tách mở rộng lặp qua anytime passes. |
| Cache cạnh ngược nằm trong query time | Reverse CSR được tạo lười rồi giữ trong graph. Đo prepare/first query/reuse và ownership của mỗi variant. |
| Seed + repeats đủ cho fairness | Chốt scope timer, graph state, h-mode, lịch ghép cặp, đơn vị thống kê và timeout handling. |
| Trừ hai ru_maxrss để lấy query peak | ru_maxrss là lifetime high-water. Báo process peak; native memory và cửa sổ RSS có phép đo riêng. |
| max_expansions là tổng budget của anytime | Hiện giới hạn áp dụng cho từng pass. Thêm contract tổng riêng trước khi dùng quality-vs-budget. |
| Đổi SCHEMA_VERSION toàn cục thành v2 | Thêm builder v2 riêng; runner v1 và báo cáo lịch sử tiếp tục dùng cấu trúc v1. |

Source được khảo sát tại worktree ngày review; HEAD quan sát là c07cef0cb53ad9bbe0ad8cc13672c3fb1869a8e9. Trước triển khai, kiểm tra lại HEAD/diff/ABI vì repo có công việc khác đang diễn ra. Đây là nhận xét từ source, chưa xác minh bằng chạy binary.

Các điểm nối đã đọc:

- [Grid2DSearchSpace](../pathplanning/spaces/grid2d.py): movement, heuristic và to_native_graph.
- [NativeGraph](../pathplanning/native/graph.py): CSR reuse, labels và ownership.
- [Search adapter](../pathplanning/planners/search/_internal/native.py): query preparation, timers, conversion về Python.
- [Search kernel](../pathplanning/native/search_engine.cpp): states, heap/deque, reverse CSR, consistency validation và anytime passes.
- [Search ABI](../pathplanning/native/search_engine.h), [ABI version](../pathplanning/native/abi_version.h), [FFI loader](../pathplanning/native/_ffi.py).
- [Benchmark contract v1](benchmark_contract.md), [runner mẫu](../scripts/benchmark_planners.py), [trace benchmark](../scripts/benchmark_trace_overhead.py).

## 2. Phạm vi và cấu hình thuật toán

MVP: grid 2D tĩnh, exact start/goal, positive costs. Bộ chính gồm Dijkstra, A*, bidirectional Dijkstra và bidirectional A*. Nhóm đánh đổi quality thêm weighted A* weights 1.25, 1.5, 2.0 và greedy best-first trên cùng input. Weight 1.0 phải khớp A*.

BFS có suite 4-connected unit-cost riêng và oracle tương ứng; optimum octile không dùng cho suite đó. DFS chỉ là baseline tìm được đường. Anytime vào milestone sau khi có pass observations và tổng budget đúng nghĩa. Các track sampling/3D/dynamic/JPS/any-angle giữ trong mục 12.

Mỗi observation ghi measurement scope:

- **prepared_kernel**: graph và query arrays đã chuẩn bị; bracket đúng lời gọi native.
- **public_api**: bracket plan_discrete, tính cả adapter, chuẩn bị heuristic, native và chuyển/free kết quả.

Mỗi scope có **fresh_graph** hoặc **reused_graph**. Các tên này mô tả graph ownership/cache; không bảo đảm CPU/file cache lạnh hay nóng. Headline mặc định là public_api + reused_graph; prepared_kernel giải thích cơ chế.

## 3. Dataset, representation và manifest

### 3.1. Profile MovingAI

Chọn collection 2D version 2. Parser scenario ban đầu hỗ trợ header version 1 hoặc 1.0; collection version và scenario-format version là hai khái niệm khác nhau. Theo [định dạng chính thức](https://movingai.com/benchmarks/formats.html), scenario có 9 trường, gốc tọa độ ở góc trên trái, optimum dùng diagonal sqrt(2) và cấm cắt góc.

Profile **land_octile_v1**:

- Cardinal cost 1; diagonal sqrt(2), chỉ hợp lệ khi cả hai ô cardinal liên quan đều đi được.
- Lưu ký hiệu ASCII gốc. Land agent đi được trên . / G / S; @ / O / T / W bị chặn. W có ngữ nghĩa water-agent riêng trong định dạng và nằm ngoài profile này.
- Unknown symbol gây dataset_error; không tự đổi thành đất/vật cản.
- Kiểm tra header, số dòng/cột, map path, bounds, endpoint passability và optimum hữu hạn không âm.
- MVP từ chối scenario có dimension khác map với unsupported_scaled_scenario; định dạng gốc cho phép scaling nên giới hạn này phải được ghi rõ.
- Resolve map theo từng dòng scenario; không giả định cả file chỉ có một map.

CSR dùng ID ổn định **id=y*width+x**, **N=width*height** slots gồm row rỗng của blocked cells; **V_free** là ô đi được; **E** là số cạnh hợp lệ có hướng. Báo cả N và V_free. CSR được dựng đúng profile; không dùng factory grid hiện tại để suy optimum MovingAI.

Thứ tự 8 motions cố định theo runner hiện tại. Ghi tie policy của từng variant: best-first hiện dùng f tăng, h tăng, insertion-order tăng. Không bắt các thuật toán khác nhau trả đúng cùng một path khi có nhiều đường tối ưu.

Heuristic cohort chính: octile, vectorized float64 array theo goal cho toàn N slots; Dijkstra h=0. Giữ **precomputed_array** ở mọi kích thước. Adapter grid hiện đổi cách chuẩn bị h quanh ngưỡng 65,536 slots, nên thay đổi h-mode phải là một ablation riêng. prepared_kernel đặt h preparation ngoài call timer nhưng vẫn báo thời gian/bytes; public_api tính nó vào query cost. Cohort đầu không cache h-array giữa queries; mọi caching bổ sung là variant riêng.

workloads.py dự kiến chứa MovingAIGrid với to_native_graph trả đúng CSR đã dựng và native_heuristic_values trả mảng octile vectorized. Mỗi variant có instance/handle riêng. Public API dùng adapter này; prepared_kernel dùng cùng CSR/start/goal/h-arrays qua FFI đã chuẩn bị. Node materialization limit được đặt rõ theo N trong problem params khi đi qua factory adapter, thay vì tình cờ từ chối map 1024x1024 vì default 1,000,000.

### 3.2. Profile dữ liệu dự kiến

| Profile | Lựa chọn cụ thể |
| --- | --- |
| fixtures | Map 2x2..16x16: chặn góc một/hai phía, hành lang, disconnected, start=goal, tie paths; thêm directed graph và heap entry cũ. Có chi phí/counters đếm tay. |
| pilot | 6 họ DAO, Starcraft, room, maze, random, street. Chọn 3 map/họ theo V_free nhỏ/trung vị/lớn, tie theo tên; mỗi map tối đa 100 scenario: 5 phân vị C* x tối đa 20 dòng, chọn bằng hash với workload_seed=7. Tối đa 1,800 query. |
| full | Toàn bộ map/scenario trong 6 họ đã chốt với danh sách file/hash đóng băng. Parser exclusions có lý do và counts. |
| scaling | Các generator và sweep ở mục 9; seed và version cố định. |

Các họ có trong [catalog 2D MovingAI](https://movingai.com/benchmarks/grids.html). Thiếu map/bin được ghi trong manifest, không bù theo success của candidate. Nếu ba vị trí map trùng nhau, lấy vị trí distinct gần nhất theo thứ tự đã chốt. Độ khó baseline-expanded chỉ gắn sau khi lựa chọn query, ngoài measurement window.

Pilot correctness/work dùng toàn bộ tối đa 1,800 queries. Pilot latency/RSS dùng subset cố định tối đa 20 queries/map: lấy 4 trong mỗi bin đã chọn, bằng cùng quy tắc hash, tối đa 360 queries. Primary latency ban đầu chỉ chạy public_api + reused_graph; các scope/graph-state khác chạy campaign ablation riêng trên subset này. Với 8 MVP variants, một latency campaign tối đa 20,160 measured observations + 5,760 warmups; work tối đa 14,400 observations; RSS tối đa 2,880 workers. Đây là số lượng dự kiến theo protocol, không phải số thí nghiệm đã chạy. Full latency/memory cohort được chốt theo pilot cost và lưu manifest trước khi chạy; không tự sinh toàn bộ tích Cartesian của scopes, variants và modes.

Manifest/query fields: family, map/scenario URL và SHA-256, scenario row, movement profile, dimensions, N/V_free/E, density, start/goal, optimum string gốc, oracle C*, h(start), solution depth và baseline A* expansions. Ghi generator version, bin boundaries và seeds.

**workload_id** phụ thuộc dữ liệu và ngữ nghĩa bài toán; **variant_id** phụ thuộc thuật toán/tham số/h-mode/tie policy; **experiment_id** thêm source/binary/host/protocol. Ghép cặp bằng workload_id và protocol; experiment_id khác build không phải pairing key.

Dataset payload ở cache local; metadata, attribution và profile được lưu có phiên bản. Kết quả ở benchmark-results/, đã được contract hiện tại loại khỏi source fingerprint.

## 4. Oracle và correctness

Reference Dijkstra dùng Python heapq trên occupancy gốc, tự duyệt chuyển động; không gọi NativeGraph, grid.neighbors hoặc search kernel. Cache theo map hash + movement profile + start/goal + reference version. Native Dijkstra là candidate/baseline; reference chạy ngoài timer/RSS window.

Validator kiểm tra endpoint, adjacency, obstacle, corner rule rồi tự tính cost bằng cardinal/diagonal counts. Kiểm tra declared_cost so với cost tự tính, rồi so với oracle. Candidate-vs-oracle tolerance: atol=1e-8, rtol=1e-10. Scenario-vs-oracle trước hết cho phép sai số floating và một đơn vị chữ số cuối. MovingAI scenario optima trong bộ dữ liệu kiểm tra tích lũy đường chéo theo `1.414213562`; nếu biểu diễn đó khác `sqrt(2)`, validator suy ra duy nhất số bước cardinal/diagonal từ C* của oracle rồi đối chiếu chuỗi optimum với cùng hằng số và nửa đơn vị chữ số cuối. Sai số biểu diễn này không áp dụng cho candidate-vs-oracle.

Scenario mismatch ngoài tolerance là dataset/reference discrepancy: điều tra trước khi benchmark, không tự gán lỗi candidate. So sánh thuật toán khác nhau bằng path validity/cost. Release-vs-metrics cùng thuật toán/input phải khớp stop reason, path hash, cost, iters và nodes.

Lưu riêng execution_status, planner_stop_reason, path_present, path_valid, declared_cost_matches và optimal_cost_matches. Timeout/crash/invalid_path/valid_suboptimal/proved_unreachable là outcome khác nhau. start=goal có C*=0: kiểm tra tuyệt đối; ratio/gap là null. Unreachable query oracle xác nhận không được tính là thất bại khi candidate trả đúng “không có đường”.

## 5. Work metrics và điểm đo

Counters uint64 ở C/C++, int trong JSON. Không đi qua PlanResult.stats hiện là Mapping[str,float]. Observation có input/outcome/work/memory/timing/capabilities/provenance. **0** là đã đo và không xảy ra; **null** là không áp dụng/chưa đo, kèm reason.

| Field dự kiến | Định nghĩa và điểm tăng |
| --- | --- |
| expanded | Node được xử lý sau khi loại entry CLOSED, gồm goal khi kernel thực sự xử lý goal; giữ nghĩa hiện tại của iters. |
| discovered_first | g đổi từ infinity sang finite, gồm start. Bidirectional ghi mỗi side; tổng side không phải node union. |
| edges_examined | Cạnh đọc trong vòng adjacency của search, trước khi kiểm tra/skip. |
| relaxation_attempts | Lần tính tentative trên cạnh hợp lệ. |
| relaxation_successes | Cập nhật g/parent, tách first discovery và cải thiện nhãn đã biết. |
| closed_neighbor_skips | Cạnh tới CLOSED bị bỏ qua; không gọi chung là dominance pruning. |
| nonimproving_skips | Candidate không cải thiện; greedy có skip reason riêng theo tiêu chí của nó. |
| frontier_pushes / frontier_pops | Mọi thao tác heap/queue/deque thật, gồm initial pushes và stale pops. |
| stale_pops | Entry bị bỏ qua do node CLOSED; denominator là frontier_pops. |
| frontier_peak_entries | Max entry đang sống, khác unique OPEN nodes; ghi side peaks và peak tổng đồng thời. |
| heuristic_array_values_prepared | Values được chuẩn bị trước search, thuộc query preparation. |
| heuristic_lookups / computations | Tách đọc h-array và tính h thật; validation có phase riêng. |
| validation_edge_checks | Full CSR consistency scan của bidirectional A*; không trộn vào search-loop edges_examined. |
| goal_tests | Kiểm tra goal trong kernel; goal-mask preparation ở Python là field riêng. |
| reopen_count / reexpanded_same_pass | null với supports_reopen=false trong implementation hiện tại. |
| anytime pass fields | Counters từng pass, tổng work qua pass và max live memory; nodes hiện tại của anytime là sum discoveries qua passes. |

Tỷ lệ dẫn xuất: expanded/V_free, edges_examined/E, successful/attempted relaxations, stale_pops/frontier_pops, pushes/expanded. Denominator 0 cho null. Không cộng các loại thành một “tổng operation” vì chi phí edge check/heap push/h-computation khác nhau. heap_comparisons là mở rộng sau MVP.

## 6. Timing và memory boundaries

### 6.1. Timing

| Field | Ranh giới |
| --- | --- |
| input_load_s | Đọc/parse input, ở cấp dataset/map. |
| graph_build_s | Tạo CSR và copy/adopt native handle. |
| algorithm_prepare_s | Reverse CSR hoặc dữ liệu retained của variant. |
| query_prepare_s | Ánh xạ query, h-array, goal flags/options. |
| native_call_s | Lời gọi C/C++ trên prepared inputs; gồm validation cần thiết, state init, search và dựng path native. |
| result_decode_free_s | Chuyển/copy path ra Python và free native output. |
| api_total_s | Bracket public plan_discrete trên graph state được khai báo. |

Scope cạnh nhau dùng timestamp chung để kiểm tra tổng và residual; scope lồng không được cộng hai lần. Clone CSR giữa library là setup trước prepared_kernel timer, có cost riêng. Chi phí process startup/harness được lưu ngoài algorithm timers.

Nếu cần state_init_s/search_loop_s/path_reconstruct_s, dùng native profiling và gắn diagnostic_profile. Thời gian phase này giải thích cơ chế; latency headline từ release, có khai báo timer overhead.

### 6.2. Memory

| Field | Phép đo |
| --- | --- |
| input_occupancy_bytes | ndarray.nbytes; labels/maps/parse buffers còn sống có field riêng. |
| base_csr_capacity_bytes | Capacity offsets/indices/costs nhân sizeof; includes blocked slots. |
| prepared_retained_bytes | Reverse CSR và precomputed tables còn sống sau prepare. |
| query_input_bytes | h-array, goal flags/options. |
| state_slots_allocated / parent_id_bytes | Slots thực của từng SearchStates và width parent 32/64-bit. |
| state_capacity_bytes | Capacities g/parent/flags nhân sizeof. |
| frontier_capacity_bytes_peak | Heap vector capacity; deque phải dùng allocator tracking. |
| native_requested_bytes_peak | Counting allocator cho các structures/buffers đã khai báo; bao gồm old/new overlap khi reallocate; ghi allocation coverage. |
| query_workspace_peak_bytes | Max tổng workspace bytes cùng sống, riêng với retained/input/output. |
| result_path_bytes | Native path và copy Python có ownership/lifetime riêng. |
| process_peak_rss_bytes | ru_maxrss chuẩn hóa OS/unit trong fresh worker; includes import/load/build/query. |
| query_peak_rss_sampled_bytes | Tùy chọn sampler ngoài worker trong READY→DONE; ghi interval/resolution. Query ngắn có thể bỏ lỡ spike. |

Tổng peak là **max_t(sum live component bytes tại t)**, không là sum các component peaks. Capacity/requested bytes khác resident RSS và chưa bao gồm mọi allocator overhead. Coverage chưa đầy đủ phải được gắn nhãn.

RSS campaign dùng release worker mới cho map+variant+query, giữ một graph candidate. Báo lifetime peak; current RSS trước query nếu backend hỗ trợ. Không trừ hai high-water marks để suy query peak. RSS sampling tách khỏi latency campaign.

Báo absolute bytes, bytes/V_free và bytes/N. Payload state một side với parent 32-bit xấp xỉ (8+4+1)*N trước headers/capacity; bidirectional có hai bộ. Native tracking phải xác minh lifetime/allocations, không suy memory từ expanded.

## 7. Protocol chạy và thống kê

Pilot mặc định 2 warmups/query/variant, 7 measured repeats, schedule_seed=7. Full campaign mặc định 15 measured repeats; nếu pilot cho thấy timing bất ổn, chốt số repeats mới trước full run. Đây là cấu hình ban đầu, không bảo đảm một độ chính xác thống kê cụ thể.

Latency worker theo map với graph handle riêng mỗi variant; mỗi thời điểm chỉ một query tính toán. Block gồm query và repeat; xáo thứ tự variant bằng schedule_seed, lưu schedule để replay. Khi nhiều graph cùng resident, ghi tổng retained footprint của latency worker; memory headline lấy từ worker riêng.

fresh_graph tạo handle mới mỗi invocation và báo build/first-query. reused_graph prepare graph và reverse CSR riêng mỗi variant trước warmup, giữ graph qua queries; search states hiện tại vẫn tạo lại mỗi call. Không dùng cache của một variant cho variant khác.

Controller bắt đầu query watchdog sau READY khi setup hoàn tất: pilot mặc định 5 s/query, 60 s/setup, timeout stage riêng. Record elapsed lower bound; timeout là censored khi tính speedup. Headline MVP để max_expansions unset; resource-budget sweep có protocol riêng. Deterministic search dùng cùng input/params ở mỗi repeat; schedule_seed khác workload_seed.

Work collection một lần/query/variant; lặp kiểm tra trên fixtures và mẫu pilot. Release iters/nodes đối chiếu metrics build. Tất cả warmups retained nhưng không vào summary.

Với query i, t_i là median release measured repeats. Báo median/p95 của t_i giữa queries và IQR/dispersion giữa repeats. P95 giữa queries không phải p95 noise của 7 lần lặp một query.

Speedup s_i=t_baseline_i/t_candidate_i trên common-valid-solved cùng protocol; báo median, geometric mean và CI. Bootstrap theo map cluster rồi query trong map, giữ cặp: mặc định 2,000 draws, bootstrap_seed=7. Ít map thì ghi giới hạn CI và hiển thị per-map. Repeats/query cùng map không được coi là mẫu hoàn toàn độc lập.

Báo micro theo queries và macro theo map/họ với trọng số rõ. Coverage = valid solved unique queries / oracle-solvable queries thuộc cohort đã chốt, riêng với execution-repeat counts. start=goal thuộc solvable cohort; reference-unreachable có decision-correctness denominator riêng trên tất cả input hợp lệ. Eligibility do input/oracle xác định trước candidate run. Báo optimal/invalid/crash/timeout counts, quality ratio mean và ratio của sums cùng denominator. Common-solved quality/speedup luôn đặt cạnh coverage toàn manifest. Repeats có outcome không nhất quán được đánh dấu và điều tra trước công bố.

Provenance gồm source/binary/input hashes, OS/CPU/RAM, compiler thực/flags, timer/resolution, h-mode, graph state, seeds và actual order. Build logs xác nhận flags; compiler metadata cấu hình chưa đủ. Chạy tuần tự, ghi khả năng affinity/frequency control và workload hệ thống quan sát được.

## 8. Native instrumentation và anytime

### 8.1. Native metrics

Thêm extension dự kiến **_search_metrics_engine** từ cùng search_engine.cpp, C++17/-O3, PP_ENABLE_METRICS=1 và object directory riêng theo [setup.py](../setup.py). Release compile-out metric hooks; trace không dùng để suy counters.

Header dự kiến search_metrics.h: counters uint64, memory/capabilities, struct_size và metrics ABI riêng. Entry pp_native_search_plan_measured nhận SearchResult và metrics output độc lập. Context lifetime theo call/pass; heap adapter giữ comparator và insertion order. Counting allocator chỉ gắn ở build metrics. Đổi heap/reserve policy là optimization variant riêng có ablation.

Thêm load_search_metrics_library trong _ffi.py với ABI/export/layout checks. Production SearchResult/PlanResult giữ layout nếu measurement API riêng. Increment search ABI theo version thực ở thời điểm thay exported declarations; không hardcode ABI kế tiếp từ bản review.

Metrics library tạo/free handle và result của chính nó. Export CSR view rồi copy vào library trước timer; owner sống lúc copy. Không truyền opaque handle/free buffer qua library khác.

Thêm API dự kiến **pp_graph_prepare_reverse** idempotent và **pp_graph_get_storage_info** cho base/reverse components, ở release và metrics. Cập nhật header, ABI, ctypes và docs; second prepare không tăng retained bytes.

### 8.2. Anytime và resource budgets

Anytime hiện restart weighted A* từng weight, cộng iters/nodes, trả best final path. time_to_first_path không có trong output hiện tại. Ticket riêng thêm pass index/weight/start/end/counters, incumbent cost/improvement events và first-valid-path timestamp. Schedule thử nghiệm [2.0,1.5,1.25,1.0] nằm trong variant_id.

Quality-vs-expanded cần total query expansion budget chung, truyền phần còn lại vào mỗi pass; max_expansions cũ là per-pass. Có parameter/contract tổng riêng và kiểm tra không reset. Path availability khác termination cause: có incumbent vẫn có thể dừng vì budget.

Quality-vs-wall-time cần cooperative deadline trong native để trả incumbent. Watchdog kill chỉ tạo timeout, không lấy được incumbent; curve này chỉ công bố sau khi cooperative return hoàn tất. Một expansion của A* và một scan của JPS không có cùng unit cost.

## 9. Complexity, scaling và ablation

Đặt N=allocated slots, M=edges_examined, P=frontier_pushes, D=solution steps. Phân tích cài đặt hiện tại, h lookup O(1), không reopen:

- BFS/DFS query: O(N+M+D), state/frontier O(N+frontier_peak).
- Best-first lazy heap: O(N+M+P*log(max(2,P))+D); mỗi pass P<=E+1, cho upper bound O(N+E*log(max(2,E))) dưới các giả định này.
- State O(N); heap theo peak/capacity có duplicates; input CSR O(N+E) riêng.
- Bidirectional thêm hai state/frontier; reverse prep O(N+E). Bidirectional A* còn full consistency validation mỗi query O(N+E).
- Anytime K passes cộng initialization/search work từng pass; peak memory theo live lifetime, không nhân K lần peak state.

Mỗi thay đổi reopen/heap/heuristic/representation/workspace reuse cần cập nhật bound và giả định.

| Sweep ban đầu | Thiết kế |
| --- | --- |
| Size | L=64,128,256,512,1024; random density=0.20; 5 generator seeds; 20 queries/map ở fixed normalized-displacement bins. Giữ movement, CSR layout và h-mode. |
| Density | L=512, density=0.10,0.20,0.30,0.40; 5 seeds; ghi connectivity và unreachable cases. |
| Heuristic | Cùng map/query, h=alpha*octile với alpha=0,0.25,0.5,0.75,1; consistent. Weighted-A* weight sweep có tên khác. |
| Topology | Room/maze corridor/opening width=1,2,4,8 với cùng L; ghi density/connectivity thay đổi cùng parameter. |

Plot work/memory theo N/V_free/E; log-log slopes có fit range và CI là xu hướng thực nghiệm, không chứng minh Big-O. [Sturtevant 2012](https://www.cs.du.edu/~sturtevant/papers/benchmarks.pdf) chỉ ra scaling map làm đổi thuộc tính không gian; ghi topology covariates thay vì giả định resize giữ nguyên mọi thứ.

Ablation đầu: fresh/reused graph; precomputed/lazy h nếu backend hỗ trợ; release/metrics overhead; forward/bidirectional. Mỗi ablation đổi một yếu tố và báo work/time/memory/quality. ns/expansion là tỷ lệ tổng hợp, không là chi phí nhân quả một expansion vì chứa init/validation và công việc khác.

Hòa vốn A/B giải từ P_B+Q*q_B <= P_A+Q*q_A. Khi P_B>P_A và q_B<q_A: Q*=ceil((P_B-P_A)/(q_A-q_B)); trường hợp khác có classification riêng. q lấy cùng query distribution, baseline prep/load cũng phải tính; storage budget riêng.

## 10. Report v2, outputs và CLI

Builder v2 nằm trong `scripts/shortest_path_benchmark/contract.py`, dùng lại
các helper provenance/environment của `benchmark_contract.py` và bổ sung hashes
cho metrics binary. `create_report` và `SCHEMA_VERSION` v1 hiện tại giữ hợp
đồng của chúng. Outputs:

~~~text
benchmark-results/<campaign>/
  manifest.json
  schedule_<pass>_<scope>_<graph_state>.json
  <pass>_<scope>_<graph_state>.json
  oracle.jsonl
  runs.jsonl
  run_summary_<pass>_<scope>_<graph_state>.json
  summary.json
  report.md
  plots/*.svg
~~~

Row chứa run/workload/variant/campaign IDs, phase/repeat/order/scope/graph_state/budget và input/outcome/work/memory/timing/capabilities/provenance. Giữ exact int >2^53; unavailable/nonfinite là null + reason, không NaN/Infinity.

JSONL records hoàn chỉnh, checkpoint/flush; resume kiểm tra manifest/binary/protocol hashes. Dòng cuối dang dở có crash-recovery marker; chỉ chạy key chưa hoàn tất. Terminal timeout/error được giữ. Primary coverage không được đổi âm thầm bằng chỉ giữ retry thành công. Summary snapshot viết atomically.

CLI nonzero khi correctness/ABI/manifest gate lỗi, crash hoặc thiếu expected observations. Approximate path hợp lệ có gap dương là outcome đúng nếu được khai báo. Missing required metric gây incomplete_measurements.

Full/scaling campaign cần báo per-map/family/difficulty, paired work ratios,
coverage-vs-budget và Pareto time-memory-quality cùng workload/protocol; coverage
và quality phải được annotate. MVP report hiện có aggregate latency/work/memory
và paired statistics; strata đầy đủ, budget frontier và Pareto report còn thuộc
SP-09/SP-10. EBF, transit-node count, estimated diameter/map-dimension proxy là
descriptors tùy chọn, không là MVP required metrics.

CLI được triển khai:

~~~text
python scripts/benchmark_shortest_path.py prepare --profile pilot --manifest <path>
python scripts/benchmark_shortest_path.py validate --manifest <path>
python scripts/benchmark_shortest_path.py run --manifest <path> --pass latency --scope public_api --graph-state reused_graph
python scripts/benchmark_shortest_path.py run --manifest <path> --pass work
python scripts/benchmark_shortest_path.py run --manifest <path> --pass memory
python scripts/benchmark_shortest_path.py analyze --campaign <directory>
~~~

Latency/work/memory passes có provenance riêng. Analyzer kiểm tra compatibility
keys; ghép ba pass không biến chúng thành số đo đồng thời. Report Markdown hiển
thị latency median/P95 và paired speedup; work counter và allocator-memory
median/P95; fresh-worker RSS cùng occupancy/CSR/retained-graph memory. `summary.json`
giữ vector đầy đủ với count, median, P95, min, max, IQR, paired ratios và
bootstrap intervals. RSS có phạm vi vòng đời tiến trình, bao gồm Python, NumPy,
đọc map và tạo graph; không được diễn giải là memory chỉ riêng search query.

## 11. Backlog, dependencies và acceptance gates

Các acceptance gates được giữ lại để phân biệt MVP với các track mở rộng. Code
change chạy Graphify update; generated output giữ ngoài Git.

| Ticket | Deliverable / file dự kiến | Phụ thuộc | Acceptance |
| --- | --- | --- | --- |
| SP-01 | scripts/shortest_path_benchmark/contract.py, profiles/pilot.json, profiles/scaling.json; schema dictionary và contract docs | Kế hoạch | V1 không đổi cấu trúc; v2 round-trip int >2^53; missing/zero/unsupported và denominators đúng. |
| SP-02 | workloads.py: parser, no-corner CSR, vectorized heuristic adapter, manifest | SP-01 | Version/symbol/dimension/path/bounds được kiểm tra; corner fixtures có cạnh đúng; pilot IDs/hash tái tạo. |
| SP-03 | reference.py: độc lập oracle, validator và cache | SP-02 | Known costs/start=goal/disconnected/tie paths/declared-cost mismatch; scenario discrepancies được giữ. |
| SP-04 | search_metrics.h, search_engine.cpp/.h, abi_version.h, _ffi.py, setup.py, docs/native_abi.md; metric hooks/build/loader và graph APIs | SP-03 | Release/metrics parity trên mọi MVP variant ở fixtures/subset; counters đếm tay; ABI/free/ownership đúng; reverse prepare idempotent và cache riêng. |
| SP-05 | Allocation tracking + memory observations | SP-04 | Slots/dtype/capacity đúng; heap duplicates/reallocation overlap có phép đo; total peak theo live sum; failed/unreachable call vẫn có counters. |
| SP-06 | runner.py + benchmark_shortest_path.py: workers, scopes, schedule, watchdog, JSONL/resume | SP-01..05 | Replay schedule; query tuần tự; timeout đúng stage; graph state đúng; crash/missing records retained; unique-query khác repeat denominator. |
| SP-07 | analysis.py: paired stats, cluster bootstrap, weights, report/plots | SP-06 | Fixture kiểm tra median/ratio/censor/unmatched/zero-cost; CI seed tái tạo; plots có units/counts. |
| SP-08 | Pilot end-to-end, build/environment evidence và reproducibility docs | SP-07 | Work/correctness tối đa 1,800 queries, latency/RSS subset tối đa 360; không unexplained validity/optimality mismatch; metrics parity; đủ required fields trên từng measurement cohort. Gate trước full campaign. |
| SP-09 | Anytime pass observation, global budget, cooperative incumbent return | SP-08 | Sum work đúng; tổng budget không reset; first/last incumbent kiểm chứng; termination và path availability đúng. |
| SP-10 | Full/scaling/ablation campaigns và frontier reports | SP-08; SP-09 cho anytime | Cohort/config đóng băng; đủ expected records; bounds và slopes có evidence riêng; mọi failure/limitation xuất hiện trong report. |

### Trạng thái triển khai, 2026-10-06

| Ticket | Trạng thái |
| --- | --- |
| SP-01–SP-06 | Đã triển khai; contract, MovingAI parser/oracle, native metrics/allocation hooks và runner có test. |
| SP-07 | MVP report/plot đã triển khai: latency, work counters, allocator memory, process RSS, quality, paired ratios và bootstrap. Full phân tầng per-map/difficulty, coverage-vs-budget và Pareto frontier thuộc SP-09/SP-10. |
| SP-08 | Pilot end-to-end đã qua: 1,750 workload × 8 biến thể ở work (14,000 observations); 360 workload × 8 biến thể ở latency (20,160 measured + 5,760 warmups) và memory (2,880 observations). Tổng 42,800 observations đều `ok`; không có đường đi sai, thiếu metric bắt buộc hay sai khác oracle chưa giải thích. Output ở `benchmark-results/` bị gitignore và cần tạo lại theo reproduction guide. |
| SP-09 | Chưa triển khai: anytime pass observations, tổng resource budget và incumbent events. |
| SP-10 | Một phần: scaling profile/generators, log-log fit và ablation identity helpers đã có; full/scaling/ablation campaigns và Pareto-frontier evidence chưa chạy/chưa hoàn tất. |

SP-01→SP-08 là MVP và pilot đã vượt correctness gates. SP-09/SP-10 vẫn là
follow-up, không được suy ra đã hoàn thành từ việc các profile hoặc sweep helpers
tồn tại. Tests hiện có gồm `test_movingai_workloads.py`,
`test_shortest_path_reference.py`, `test_native_metrics.py`,
`test_shortest_path_benchmark_contract.py`,
`test_shortest_path_benchmark_runner.py`,
`test_shortest_path_benchmark_analysis.py`, và
`test_shortest_path_scaling.py`. Tận dụng [native graph tests](../tests/test_native_graph_search.py)
và [trace parity tests](../tests/test_native_trace.py) cho regression.

Native/API validation cần build release/metrics/trace, focused tests, ruff/pyright
và non-slow suite theo Makefile khi phạm vi yêu cầu; full/slow mở rộng theo
failure/risk. Pilot này đã chạy focused suite và Ruff; test files ở trên là phần
triển khai hiện có, không còn là test dự kiến.

## 12. Các track mở rộng đã giữ

| Track | Metrics bổ sung | Điều kiện |
| --- | --- | --- |
| JPS/grid pruning | Scanned cells/blocks, repeated scans, jump points, forced-neighbor checks, prune reason | Có planner thực và same movement/cost; expansion không thay scan work. |
| Any-angle | LOS calls, primitive cells/segments checked, geometric cost/validity | Oracle đúng objective; octile optimum không là any-angle optimum. |
| Sampling | Attempted/accepted/rejected samples; NN queries/distance evaluations; collision calls/steps; rewire attempts/success; tree/index peak; quality/coverage theo budget và RNG seeds | Workload continuous riêng; reuse sample_count/motion_checks/rewires hiện có; không gộp nodes với expansions. |
| Incremental/real-time | First prefix, max segment latency, repair/update work, retained state, amortized sequence cost | API trả prefix/update thật và fixed change trace. |
| Hardware profiling | Instructions/cycles/cache/branch misses/allocations | Platform hỗ trợ, run riêng; unavailable=null và machine dependence được ghi. |

[GPPC 2014](https://webdocs.cs.ualberta.ca/~nathanst/papers/GPPC-2014.pdf) đánh giá preprocessing/memory/quality/latency và Pareto trade-offs. Kế hoạch này thêm work definitions, memory lifetime và protocol ở mức implementation để giải thích cơ chế.

## 13. Tiêu chí hoàn tất

MVP hoàn tất khi có validated parser/profile, oracle độc lập, work metrics có semantics/coverage, release scopes đúng, native memory và RSS đúng nhãn, report v2 tái tạo được và pilot vượt correctness gates.

Kết quả nghiên cứu cần thêm full/scaling cohorts đóng băng, paired distributions/sample counts, coverage/failures, dữ liệu/build provenance và ablation cho cơ chế được tuyên bố. Mỗi phần báo rõ proposed/implemented/measured; số runtime hoặc dashboard riêng chưa hoàn tất các gates này.
