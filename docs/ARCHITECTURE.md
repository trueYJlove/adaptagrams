# Local Orthogonal Router 架构设计与 API 接入说明

本文档对应仓库精简后的纯 C++17 正交布线引擎（命名空间 `localroute`），遵循 Google C++ Style Guide（仓库根目录 `.clang-format` 采用 `BasedOnStyle: Google`），涵盖整体分层架构、核心算法原理、线程模型、配置参数说明及 C++ / GUI 接入范例。

## 1. 设计目标、Simulink 核心原则与工程契约

本引擎面向类 Simulink 电气拓扑与框图建模软件，针对 **1,000 ~ 10,000+ 模块、2,000 ~ 20,000+ 连线** 的交互式编辑场景设计，以 MathWorks Simulink 编辑器的四大核心交互/布线原则作为长期架构准绳：

1. **最小扰动原则（Minimum Disturbance）**：智能布线的首要目标不是每次都重算全局最短路，而是**尽可能少地打扰已有布局**——拖动模块时非关联连线保持绝对静止；框选多块移动时内部拓扑刚性平移，仅外部连线重算；用户手动调整过的线段（`Route::user_modified = true`）在后续移动与分轨中应尽量保留，且模块层级遵循所见即所得（拖拽不临时置顶）。
2. **交互态与整理态严格分层**：高频交互拖拽追求亚毫秒级响应与低扰动（保持 `occupied_wires = nullptr`）；只有在用户显式触发 `AutoRouteConnectors`（`整理选中线 (Auto-route)`）或全图整理时，才执行更彻底的避障、拥挤分流（`wire_overlap_penalty`）与通道分轨。
3. **同源分线与动态焊点（Branching & Solder Joints）**：同一源引脚 `(src, sp)` 扇出的多条连线在视觉上共享主干并在分叉处以实心焊点（`ComputeSolderJoints`）明确标识连通；不同源连线的正交交叉则通过跳线弧（Line Hops）清晰区分“跨过不连通”。
4. **可预测的避障与直线保持（Orthogonal Avoidance & Preserve Straightness）**：常规连线保持严格正交避障与引脚出线方向约束；当拖动单模块使端口近似对齐时提供直线吸附（Smart Guides），保持无拐直线；当模块重叠无法避障时提供不反转引脚方向的确定性外环兜底。

在此原则之上，工程实现严格恪守以下四条底层契约：

1. **零外部依赖、纯 C++17**：核心静态库 `local_router` 仅依赖 C++17 标准库与 `Threads::Threads`，无 `libavoid` 或任何第三方库依赖，跨平台支持 Windows（MSVC / Clang）与 Linux（GCC / Clang）。
2. **默认“移模块压线，线不动”**：在默认配置（`Config::reroute_third_party_blocked = false`）下，拖动模块时**仅重算与移动模块直接相连的连线**，其余非关联连线保持绝对静止（允许临时穿模），避免无关连线跳变与拖动帧率抖动。
3. **严格全局几何复验**：所有候选路径（包括快路径复用、L/Z 候选与 A* 输出）均需通过基于完整 `SpatialGridIndex` 的全局碰撞、正交性、缓冲净空与引脚出线方向复验，绝不将漏障或退化直连误报为 `Status::kValid`。
4. **UI 线程与计算解耦**：同时提供同步批量接口（`LocalOrthogonalRouter`）与非阻塞异步会话接口（`AsyncLocalRouter`），后者保证 UI 主线程单帧提交与取回处于亚毫秒级。

## 2. 模块分层与文件职责

```text
include/localroute/
├── local_route_solver.h       # 几何基元、Config、Route 与求解器抽象 LocalRouteSolver
├── spatial_grid_index.h       # 泛型二维均匀空间网格 SpatialGrid<Item> 与 SpatialGridIndex
├── local_orthogonal_router.h  # 同步模型管理、快路径、局部窗口调度、全局复验与并发冷启动
└── async_local_router.h       # 异步后台工作线程会话、视口优先调度与过期结果丢弃

src/
├── geometry_utils.h             # 内部共享窄缝缓冲距离计算工具（RectPointGap / EffectiveObstacleBuffer）
├── lazy_hanan_a_star_solver.cc  # 默认求解器：非均匀正交网格 + 二维阻塞位图 + 两阶段正交 A*
├── local_orthogonal_router.cc   # 同步路由器实现、退化前置检查与 SVG 调试导出
├── channel_nudger.h / .cc       # 轻量局部通道分轨（按 NetKey 等距无交叉分轨）
└── async_local_router.cc        # 异步双副本隔离与代际版本管理实现

qt_demo/
├── library_widget.h / .cc       # 可拖拽 Simulink 风格图元库（7 种预设模块模板）
├── canvas_widget.h / .cc        # 自定义高帧率裁剪画布、手动连线、预设拓扑与 HUD
├── main_window.h / .cc          # 图元库/端口检查器 Dock、预设与运行模式开关
└── main.cc                      # 交互入口与 `--smoke-bench` 回归校验
```

## 3. 核心算法与机制详解

### 3.1 二维空间网格索引（`SpatialGrid<Item>`）

* **网格映射**：按 `Config::cell`（默认 `256.0`）使用 `std::floor(x / cell_)` 将二维坐标映射到整数格坐标，将 `(int32_t x, int32_t y)` 打包为 `uint64_t` 键，正确支持负坐标。
* **增量更新**：`Update(id, new_rect)` 始终刷新精确包围盒 `boxes_[id]`；当新旧矩形覆盖的网格区间不变时，直接跳过网格桶搬移。
* **内部单元免哈希优化与联合窄条查询**：`Query(area)` 遍历查询矩形覆盖的网格时，仅对边界格元素执行精确相交判断 `boxes_.at(id).Intersects(area)`；完全落入查询框内部的网格直接批量拷贝，最后排序去重。补充 `QueryUnion(areas)` 对多窄条矩形跨格键做单次去重合并，供分轨阶段规避长线二维外包框膨胀。
* **并发只读安全**：`Query()` 与 `QueryUnion()` 均为严格无副作用的 `const` 方法（不在实例内维护可变 `queryStamp`），支持 `RouteAllParallel()` 多线程并发只读查询。

### 3.2 多级路由管线（`LocalOrthogonalRouter::SolveOne`）

对每一条待求连线，按成本由低到高依次经过 4 个阶段，任一阶段产出均需通过统一的 `accept()` 全局碰撞与方向复验：

1. **前置退化检测与窄缝自适应放行**：
   * 起止引脚重合 → 标记 `Outcome::kCoincidentPins`、`Status::kFailed`；
   * 仅当起点或终点引脚埋入**第三方模块本体矩形 `rect.Inside(end)`** 内部时标记 `Outcome::kBuriedPin`、`Status::kDisplayOnly`；
   * 若引脚位于第三方模块 `buffer` 膨胀缝隙中（`gap < cfg.buffer` 且未进入本体矩形），不直接弃算，而是将该邻近障碍与对应起止端点的有效缓冲动态收缩至 `std::max(0.0, gap * 0.5 - 0.5)`（`LazyHananAStarSolver` 与 `accept()`/`ValidateAndSimplifyRoute()` 共享同一公式），允许从窄缝正常出线并执行完整 A* 避障。
2. **轻量快路径与手动路径锁定（`Config::fast_paths` / `Route::user_modified`）**：
   * **第零优先级 — 手调路径保留（`user_modified == true`）**：当用户通过 `MoveRouteSegment()` 手动调整过连线线段后，在后续关联模块拖动（含松手 `settled = true`）时优先尝试整线平移与**端点正交修补复用（`RouteMethod::kReuse`）**，通过 `accept()` 复验即保留 `user_modified = true` 与手调中段（不再被 `Simple` L/Z 强行覆盖）；仅当手调中段撞上障碍物或方向非法无法通过复验时，才回退常规求解并自动清除 `user_modified`。
   * **第一优先级 — 整线平移复用（`RouteMethod::kReuse`）**：当起止端点位移量 `(dx, dy)` 完全相等（典型为框选批量拖动内部连线）时，整条旧折线平移后直接复验，命中即微秒级返回。
   * **第二优先级 — L / Z 轻量候选（`RouteMethod::kSimple`）**：优先尝试 2 段 L 型折线及中点/缓冲边界对齐的 3 段 Z 型折线并复验；只要拖动后存在无障碍的简洁 2/3 段通路，折线会立即自动收直，彻底消除旧中段残留导致的 4 拐回折尾巴。在显式整理（`AutoRouteConnectors`）模式下，会跳过 `kReuse` 并检查候选是否与 `occupied_wires` 重叠，避免快路径强挤拥挤缝隙。
   * **第三优先级 — 端点修补复用（`RouteMethod::kReuse`）**：仅当 `Simple` 被障碍物阻挡无法通过且旧路径 `points.size() >= 3` 时，保持原折线中段不变，仅正交修正首尾拐点衔接新引脚位置并复验。
3. **局部窗口迭代补障 + 求解器调用（`RouteMethod::kNative`）**：
   * 初始窗口取起止点包围盒外扩 `padding + buffer`，随后每轮重试按 `padding * 2^attempt` 扩张（最多 `retries` 轮）。
   * 若窗口内障碍数超过 `budget`（默认 `120`），优先保留 `required` 集合（起止模块 + 前序轮次实际碰撞到的第三方模块），其余按到起止线段垂直距离用 `std::nth_element` 截断保留最近的障碍物。
   * 求解器返回候选折线后，`accept()` 逐段回查**完整全局 `SpatialGridIndex`**；若发生碰撞，将命中的实际障碍物写入 `required` 并进入下一轮重试。
4. **Simulink 风格兜底降级（`MakeSimulinkFallback`）**：
   * 若出现引脚埋入（`Outcome::kBuriedPin`）、重试耗尽或超出障碍预算（`Outcome::kObstacleBudget` / `Outcome::kCollision`），保留 `Status::kDisplayOnly` 语义（绝不伪装为 `Status::kValid`），并按模块是否重叠严格分支：
     * **贴边顺向对齐**：`a == b` 或起止方向相对且顺向对齐（如 `Right -> Left` 且 `a.y == b.y && b.x >= a.x`）时直接返回 2 点短连线 `{a, b}`，无多余绕线；
     * **非重叠场景严格收紧（`!src.rect.Pad(cfg.buffer).Intersects(dst.rect.Pad(cfg.buffer))`）**：回退为紧凑的 2/3 段正交 L/Z 折线（`CompactFallback`），绝不因第三方遮挡或超预算生成跨大范围画布的外环；
     * **起止模块重叠 / 紧贴越界外环（`src.rect.Pad(cfg.buffer).Intersects(dst.rect.Pad(cfg.buffer))`）**：基于起止模块联合外包围盒 `U = union(src.rect, dst.rect)` 外扩 `margin = max(buffer * 2.5, 20.0)`，从 `sp.dirs` 沿外侧延伸、从 `dp.dirs` 逆向绕外侧进线，比较上下/左右外侧横梁到起止中心线的距离并择近（等距优先上方/左侧），生成严格满足 `sp.dirs` 与 `dp.dirs` 的干净 6 点 5 段外环（正交或同向组合对称简化）。

### 3.3 默认求解器：`LazyHananAStarSolver`

`LazyHananAStarSolver()` 返回的实现摒弃了传统全局可见性图指针链表，采用纯连续数组与线程局部工作区复用，核心流程如下：

1. **非均匀正交坐标离散化**：
   * 收集起止引脚精确坐标、所有局部障碍物按动态有效缓冲膨胀后的边界 `rect.Pad(eff_buf)` 及外围逃逸环，排序去重并保护引脚坐标不漂移。
   * 当坐标数 `< 200` 时，按空隙宽度优先向大通道注入中点（上限 `240`），使路径自然走通道中央而非紧贴模块边缘。
2. **微秒级二维阻塞位图（`edge_h` / `edge_v`）与拥挤度软惩罚（`kEdgeWireOccupied`）**：
   * 针对每个障碍物膨胀框，用二分定位其覆盖的网格区间并标记位掩码（`kBlkOther`、`kBlkSrc`、`kBlkDst`、`kEdgeBorder`）。
   * 预计算起点沿 `sp.dirs` 离开 `src` 缓冲框、终点沿 `dp.dirs` 离开 `dst` 缓冲框的合法逃逸射线，禁止在起止缓冲框内部提前拐弯，同时绝不豁免任何第三方障碍。
   * **窄缝多线拥挤度软惩罚**：当 `Request::occupied_wires` 非空且 `Config::wire_overlap_penalty > 0.0` 时，求解器先对直连快路径执行不同源共线重叠校验，再遍历 `occupied_wires`（跳过同源 `NetKey {src, sp}`）用二分定位覆盖的 `edge_h`/`edge_v` 微段置位 `kEdgeWireOccupied = 16`，并在状态转移及出/入逃逸段叠加非负软代价 `+ cfg.wire_overlap_penalty`（保持启发式 `h` 严格可采纳）。为守住万级 10k 拖拽 `< 5.0 ms` 性能红线，`RouteAllParallel` 与高频 `RerouteAffected` 交互拖拽保持 `occupied_wires = nullptr` 零开销，仅在 `AutoRouteConnectors` 显式整理（或直接传入 `Request::occupied_wires`）时启用。
3. **带方向状态与最小拐点启发式（`MinRemainingBends`）**：
   * 搜索状态编码为 `((iy * w + ix) << 2) | dir_idx`（4 方向：`kUp/kDown/kLeft/kRight`，禁止 180° 掉头）。
   * 通过四方向坐标旋转将目标入口方向统一映射到 `+X`，按位移与当前朝向奇偶性精确计算最少剩余拐点数（0–4 拐），启发式 `h = manhattan + min_bends * penalty` 严格可采纳。
   * 贴边微段附加极小软偏置（`1e-4`），在等长路径间优先选择居中通道。
4. **两阶段搜索与零堆分配工作区**：
   * **Pass 1（精确最优）**：权重 `1.0`，紧凑步数上限（截断窗口 `800` 步、常规窗口 `2,500` 步），保证绝大多数连线获得最优低拐点居中路径。
   * **Pass 2（深层绕障回退）**：Pass 1 触及步数上限且队列非空时，复用同一阻塞位图执行 `1.15` 加权 A*（对截断长线窗口叠加对角走廊引导偏置 `0.02 * perp_dist`，上限 `20,000` 步）。
   * 使用 `thread_local SolverWorkspace` 复用所有数组，并通过单调递增的 `run_stamp` 消除每轮大数组清零开销。

### 3.4 局部通道分轨（`ChannelNudger`，`src/channel_nudger.h/.cc`）

为消除多条独立连线走中间通道时重合叠成单线（如平行对连母线、级联走廊重叠）的问题，系统在 `Config::enable_channel_nudging = true` 时于单次重算/冷启动末尾执行轻量级 $O(K \log K)$ 通道分轨（支持通过 `LocalOrthogonalRouter::set_enable_channel_nudging(bool)` 与 `AsyncLocalRouter::set_enable_channel_nudging(bool)` 运行时切换）：

1. **活动线与只读锚定线严格隔离（`mutable_conns` vs Fixed Anchors）**：
   * 全图冷启动（`RouteAllParallel` 合并后，`full_graph = true`）：全图 `Status::kValid` 且 `user_modified == false` 的连线属于可移动集，按确定性升序 `Id` 执行单趟全图等距分轨；凡 `user_modified == true` 的连线在全图与局部模式下均作为只读锚定轨道（Fixed Anchors），不被擅自挪动。
   * 交互拖拽与局部编辑（`RerouteAffected`、`RouteConnector`、`AutoRouteConnectors` 及 `AsyncLocalRouter::Poll` 局部收口，`full_graph = false`）：仅本次重算且未锁定（`!user_modified`）的直连线集为 `mutable_conns`；`CollectParticipants()` **不再拼合整条路由的二维大 `union_box`**，而是按确定性升序 `Id` 遍历 `mutable_conns` 中 `Status::kValid && points.size() >= 4` 连线的各可移动中间段 `[1, pts.size() - 3]`，沿线段外扩 `max(2 * nudge_distance, 80.0)` 执行窄条空间查询（局部紧凑短段簇 `<= 32` 格合并一次 `Query()`，跨区短段按 `8` 条、超长段 `> 600 px` 按 `4` 条分批 `QueryUnion()`，达到参与上限 `cap = max(256, selected.size())` 即早停；仅收集不足 `16` 条时回退补查起止端点及邻接块）。查询到的邻近非关联连线及所有 `user_modified == true` 连线**仅作为只读占用轨道（Fixed Anchors）参与冲突检测与避让**，确保拖拽时非关联线与手调线 100% 静止。
2. **中间段与电气网表语义（`NetKey`）**：仅平移 `pts.size() >= 4` 连线的中间正交段（索引 `[1, pts.size() - 3]`），绝不篡改起止引脚位置与出线方向；按 `NetKey { src_block, src_pin }` 归组——**同一源引脚扇出允许共享主干，不同 `NetKey` 独立信号强制分轨**（同源固定线已存在时新扇出优先锚定其主干）。增量模式下若整个坐标带或冲突簇内不含可移动线段，直接整段跳过。
3. **坐标带分桶 + 区间冲突聚类 + 固定轨道避让分配**：分别处理垂直与水平中间段；冷启动按 `<= 1.5 px` 聚合共线坐标带，交互态按 `< nudge_distance` 收集冲突簇。若簇内均为可动线且不撞固定轨道，按 Minimal-Crossing 排序在 `[free_min, free_max]` 内居中均匀摊开；当可动线与只读固定线坐标差 `< nudge_distance - 1e-3` 且区间重叠时，固定线纹丝不动，可动线通过 `PickNearestFreeTrack()` 预过滤垂直相交横向锚点（`cross_anchors`）、将候选轨道按 `(displacement, coord)` 升序去重后搜索（命中首个净空且零交叉轨道即早停），落位至空闲轨道。
4. **安全净宽夹紧 + 局部 3 段增量复验与回滚**：通过 `SpatialGridIndex::Query` 找出同跨度左右/上下障碍物膨胀框及相邻拐点约束得到安全通道 `[free_min, free_max]`（空间极窄不足 `2 * nudge_distance` 时自动选择最大净空子隙中心）；对单一中间段非退化平移（`e_end - e_start == 1` 且相邻段无零长退化）优先调用 **`LocalOrthogonalRouter::ValidateNudgedMiddleSegment()`**，仅复验实际变更的相邻 3 段（`seg_idx - 1`、`seg_idx`、`seg_idx + 1`）的正交性、引脚出线方向与全局障碍碰撞（在首末段直接复用同一次 `index_.Query` 扫描计算 `src_buf`/`dst_buf`，其余端按需延迟查表保持完整窄缝缓冲语义），多段并移或退化时才回退 `ValidateAndSimplifyRoute()`；若任一线冲突仅单线回滚，**100% 保留原有 `Valid` 合法率**。

### 3.5 异步会话模型（`AsyncLocalRouter`）

* **双副本状态隔离**：主线程（Owner）仅操作 `model_`，后台工作线程独享一份独立的 `LocalOrthogonalRouter` 副本，彻底避免读写锁竞争或工作线程触碰 UI 数据。
* **严格 FIFO 增量拓扑与拖动合并（`pending_ops_`）**：高频拖动调用 `Move()` 时，同模块位移合并覆盖到 `edits_`；当调用 `AddBlock`、`RemoveBlock`、`AddConnector`、`RemoveConnector`、`UpdateBlockPins`、`RestoreRoute`、`MoveRouteSegment` 或 `AutoRouteConnectors` 时，先经 `FlushCoalescedEditsLocked()` 将此前合并的 `edits_` 刷入有序队列 `pending_ops_` 末尾，再入队本次操作并递增 `generation_` 丢弃过期后台结果，确保双副本重放严格保序、零悬空 ID 竞态。
* **视口优先与首屏分轨**：调用 `Prioritize(viewport, selection)` 后，后台队列优先处理选中块与可见区域内的连线；当当前视口 `priority_` 连线全部计算并出队后，不打断后台全图波次即在 Owner 侧先行触发一次局部 `ApplyChannelNudging()`，全图波次清空时再做最终收口。

### 3.6 局部焊点检测、无损端口更新与手动调线

1. **按视口局部计算同源分叉焊点（`ComputeSolderJoints(conn_ids)`）**：
   * 纯只读局部查询接口（仅扫描调用方传入的当前视口可见连线子集，绝不扫描全图）：先过滤 `Status::kValid` 且点数 `>= 2` 的连线，按 `(src, sp)` 排序归组，对单线组直接零分配跳过。
   * 对多线同源组提取水平/垂直线段，检出三类电气分叉点（排除引脚起终点本身）：**T 型分叉点**（一方内顶点严格落在另一方线段内部）、**共享段分叉端点**（共线重叠区间端点处两条线朝不同方向离开）、**同源十字交点**（同源水平段与垂直段严格内部相交）；结果按 `1e-3` 容差去重。不同源（不同 `(src, sp)`）交叉绝不产生焊点。
2. **原生无损端口更新与路径恢复（`UpdateBlockPins` / `RestoreRoute`）**：
   * `UpdateBlockPins(block_id, new_rect, new_pins)` 原地更新模块包围盒与端口列表，自动移除失效引脚上的连线，对仍保留引脚上的连线**保持原 `conn_id` 与 `user_modified` 锁定状态**并置脏待重算；`RestoreRoute(conn_id, route)` 用于轻量增量 `Undo/Redo` 微秒级回放路径状态。
3. **手动拖拽连线线段与同源主干随拖联动（`MoveRouteSegment`）**：
   * 拖动中间段（`1 <= seg_index <= pts.size() - 3`）时平移该段端点坐标，拖动 2 点直连或首/末段时自动插入正交折弯；同时自动查找同源 `(src, sp)` 兄弟连线中共线重叠的共享主干段同步平移并置 `user_modified = true`，保持 T 型焊点不断裂。
   * **禁止推动非关联线**：`MoveRouteSegment()` 与鼠标松手绝不触发 `ChannelNudger`，手调线严格停留在用户指定位置（撞障即置为 `Status::kDisplayOnly`），保证周边任意连线零扰动。
4. **显式整理连线（`AutoRouteConnectors(conn_ids)`）**：
   * 仅当用户显式触发整理时，清除指定连线的 `user_modified = false`，跳过 `kReuse` 并采集局部邻近不同源 `Status::kValid` 线段（上限 `128` 条）传入 `Request::occupied_wires` 执行避障与拥挤分流求解，随后运行局部 `ChannelNudger` 完成等距分轨。

## 4. 核心数据结构与配置参考

### 4.1 几何与拓扑模型（`include/localroute/local_route_solver.h`）

* **`Point { double x, y; }`**、**`Rect { double l, t, r, b; }`**（世界坐标系，要求 `l <= r` 且 `t <= b`）。
* **`Direction`**：位掩码枚举 `kUp = 1`, `kDown = 2`, `kLeft = 4`, `kRight = 8`，可按位或组合。
* **`Pin { Id id; double x, y; unsigned dirs; }`**: `x, y` 为相对模块矩形的 `[0.0, 1.0]` 归一化坐标；支持多条连线共享同一引脚扇出。
* **`Block { Id id; Rect rect; std::vector<Pin> pins; }`**、**`Conn { Id id, src, sp, dst, dp; }`**。
* **`OccupiedSegment { Point u, v; Id src_block, src_pin; }`** 与 **`Request::occupied_wires`**：可选的只读邻近他线线段视图（默认 `nullptr`），用于求解器在显式整理时规避不同源拥挤通道，同源 `(src_block, src_pin)` 不受惩罚。

### 4.2 路由器配置 `localroute::Config`

| 字段 | 默认值 | 说明 |
| :--- | :--- | :--- |
| `cell` | `256.0` | `SpatialGridIndex` 单元格边长（建议为模块平均宽度的 2～3 倍） |
| `padding` | `250.0` | 初始局部路由窗口外扩边距，重试时按 $2^k$ 扩张 |
| `buffer` | `8.0` | 连线与模块边界的最小避障缓冲净空（近距排版场景可适当调小至 `4.0 ~ 6.0`） |
| `penalty` | `50.0` | 单次 90° 拐弯惩罚（越高越倾向少拐点） |
| `nudge_distance` | `6.0` | `ChannelNudger` 通道内相邻平行线目标间距（像素） |
| `retries` | `3` | 局部窗口增量补障最大重试次数 |
| `budget` | `120` | 单次局部求解传入求解器的最大障碍物数量预算 |
| `settle_nudge` | `false` | 松手时是否传递 `nudge = true` 给求解器 |
| `fast_paths` | `true` | 是否启用旧折线整线平移、L/Z 轻量候选与端点修补快路径 |
| `profile` | `false` | 是否记录 `Route::profile` 分项耗时（建图、搜索、查询、复验） |
| `reroute_third_party_blocked` | `false` | **默认关闭**：关闭时移模块压到无关线不重算（允许穿模）；开启时维护线段索引并重算被挡第三方线 |
| `enable_channel_nudging` | `true` | **默认开启**：是否启用局部通道等距分轨（支持 `set_enable_channel_nudging(bool)` 实时切换） |
| `wire_overlap_penalty` | `25.0` | 窄缝多线共线重合软惩罚权重（设为 `0.0` 时关闭拥挤检测） |

### 4.3 路由结果状态 `Status`、`Outcome` 与锁定标志

* **`Route::user_modified`**（`bool`，默认 `false`）：由 `MoveRouteSegment()` 置位，表示该路径含用户手动拖拽调整的线段，后续增量重算优先保留其中段且 `ChannelNudger` 不擅自移动；调用 `AutoRouteConnectors()` 或中段碰撞障碍回退重算后清除。

* **`Status::kValid`**：通过全部几何、方向与全局障碍复验的合法正交折线。
* **`Status::kPending`**：异步模式或脏标等待重算中（可配合 `Preview(id)` 或保留的旧点列绘制虚线预览）。
* **`Status::kDisplayOnly`**：物理无解（如 `Outcome::kCoincidentPins`、`Outcome::kBuriedPin`）或超预算降级（`Outcome::kObstacleBudget`、`Outcome::kCollision`），仅用于展示，不计为避障成功。
* **`Status::kFailed`**：求解异常或非法输入。

## 5. C++ API 接入指南

### 5.1 CMake 集成

在主工程 `CMakeLists.txt` 中引入本仓库（可关闭测试与演示程序）：

```cmake
set(LOCALROUTE_BUILD_TESTS OFF CACHE BOOL "" FORCE)
set(LOCALROUTE_BUILD_QT_DEMO OFF CACHE BOOL "" FORCE)
add_subdirectory(path/to/local_orthogonal_router)

target_link_libraries(your_app PRIVATE local_router)
```

### 5.2 同步模式接入示例（批处理 / 离线布局 / 简单交互）

```cpp
#include "localroute/local_orthogonal_router.h"

localroute::Config cfg;
cfg.buffer = 8.0;
cfg.reroute_third_party_blocked = false;  // 默认：移模块压线不重算

localroute::LocalOrthogonalRouter router(cfg);  // 默认 LazyHananAStarSolver()

// 1. 注册模块与连线
router.AddBlock({1, {0, 0, 100, 60}, {{1, 1.0, 0.5, localroute::kRight}}});
router.AddBlock({2, {260, 0, 360, 60}, {{2, 0.0, 0.5, localroute::kLeft}}});
router.AddBlock({3, {130, -20, 220, 80}, {}});  // 中间障碍物
router.AddConnector({101, 1, 1, 2, 2});

// 2. 多线程首次全图布线（0 表示自动使用全部逻辑核心）
router.RouteAllParallel(0);

// 3. 拖拽模块并仅重算关联连线
router.MoveBlocksBatch({{1, {0, 40, 100, 100}}});
std::vector<localroute::Id> changed =
    router.RerouteAffected({1}, /*settled=*/false);

for (localroute::Id cid : changed) {
  const localroute::Route& r = router.route(cid);
  // r.status == Status::kValid 时，r.points 为严格正交折线点列
}

// 4. 手动微调连线线段、计算视口焊点与显式 Auto-route 恢复
router.MoveRouteSegment(101, /*seg_index=*/1, /*new_coord=*/180.0);
std::vector<localroute::Point> solder_joints = router.ComputeSolderJoints({101});
router.AutoRouteConnectors({101});
```

### 5.3 异步模式接入示例（推荐用于万级节点 Qt / 桌面 GUI）

```cpp
#include "localroute/async_local_router.h"

// 1. 先在同步模型中构建初始拓扑
localroute::LocalOrthogonalRouter initial(localroute::Config{});
// ... AddBlock / AddConnector ...

// 2. 移交构造异步会话，立即设置视口优先级并启动后台全量计算
localroute::AsyncLocalRouter async_router(std::move(initial));
async_router.Prioritize(current_viewport_rect, selected_block_ids);
async_router.StartAll();

// 3. 鼠标拖拽事件（主线程，典型耗时 < 0.2 ms）
void OnMouseDrag(
    const std::vector<std::pair<localroute::Id, localroute::Rect>>& moves) {
  async_router.Move(moves);
  std::vector<localroute::Id> updated;
  async_router.Poll(128, &updated);  // 立即尝试拉取已就绪结果
  // 对仍为 Status::kPending 的连线，可用 async_router.view().Preview(cid) 绘制虚线预览
}

// 4. 16ms 定时刷新（60Hz）
void OnTimerTick() {
  std::vector<localroute::Id> updated;
  if (async_router.Poll(128, &updated) > 0) {
    // 增量刷新 updated 对应连线的渲染缓存并触发重绘
  }
}

// 5. 鼠标松手
void OnMouseRelease() { async_router.Settle(); }
```

### 5.4 自定义求解器扩展

如需在未来替换或实验新的局部算法（例如引入强栅格吸附、自定义走廊偏好），只需继承 `LocalRouteSolver` 并注入构造即可，外层空间索引、快路径、复验与异步调度完全复用：

```cpp
class CustomSolver final : public localroute::LocalRouteSolver {
 public:
  const char* name() const override { return "CustomSolver"; }
  std::vector<localroute::Point> Solve(
      const localroute::Request& req) const override {
    // req.obstacles 包含局部障碍物（必含起止模块），req.conn 为目标连线
    return {};
  }
};

localroute::LocalOrthogonalRouter router(cfg, std::make_shared<CustomSolver>());
```

## 6. 线程安全与 `qt_demo` 交互能力说明

1. **同步对象线程契约**：`LocalOrthogonalRouter` 本身不是线程安全的；除其内部 `RouteAllParallel()` 冻结只读阶段外，禁止多线程并发读写同一实例。
2. **异步增量拓扑变更规则**：`AsyncLocalRouter` 通过 `AddBlock`、`RemoveBlock`、`AddConnector`、`RemoveConnector` 与 `UpdateBlockPins` 原生支持运行时增量拓扑编辑（与 `Move` 共用 FIFO `pending_ops_` 队列并递增 `generation_`），无需销毁重建会话或整图 `Clone()`。
3. **GUI 渲染与 Simulink 对标交互能力（`qt_demo`）**：
   * **视口裁剪、增量缓存与入线箭头**：通过 `view().QueryBlocks(viewport)` 做模块裁剪、增量维护 `route_cache_` 避免全图重建；`zoom >= 0.45` 时在目标端口绘制实心入线方向箭头（低缩放 LOD 自动剔除以保帧率）。
   * **同源焊点、交叉跳线弧与撞障警示**：`zoom >= 0.25` 时绘制同源分叉焊点圆点；`zoom >= 0.4` 且开启 `交叉跳线弧 (Hops)` 时绘制向上半圆跳线弧；手调线段碰撞障碍物（`user_modified && Status::kDisplayOnly`）时固定以红色虚线警示，常规兜底线默认深蓝渲染并支持 `高亮兜底线 (Debug)` 开关。
   * **多方式连线与主干随拖**：支持从端口圆点拉线，以及在已有信号线上 **`Ctrl+左键拖拽`** 直接分叉引出同源新分支；直接拖拽线段可平移线段并自动联动同源共享主干，拖拽过程支持向起止端口坐标直线吸附（`<= 5.0`，按住 `Alt` 取消），松手不推动任何旁路非关联线；点 `整理选中线 (Auto-route)` 恢复避障与分轨。
   * **轻量增量 Undo/Redo 与建模操作**：支持 `Ctrl+Z / Ctrl+Y`（及工具栏按钮）基于轻量增量 `EditCommand` 回放移动、增删、改端口、手调线与 `Auto-route`（10k 规模 `< 1 ms`）；支持 `Ctrl+拖拽` 模块复制（`>= 5 px` 触发，保留 `Ctrl+点击` 多选）、端口对齐 Smart Guides、工具栏 `导出 SVG` 以及左侧图元库拖放、右键/面板动态修改端口数。
   * **无黑框桌面启动**：Windows 下以 `WIN32` 子系统链接 `Qt5::WinMain`，双击不弹出控制台黑框；命令行传入 `--smoke-bench` 时自动通过 `AttachConsole(ATTACH_PARENT_PROCESS)` 恢复控制台/管道输出（并支持 `--smoke-out <path>`）。
4. **近距模块缓冲**：若业务画布允许两个相连模块间距小于 `2 * cfg.buffer`（默认 `16 px`），建议将 `cfg.buffer` 调至 `4.0 ~ 6.0` 以减少缓冲框互叠导致的 `Status::kDisplayOnly` 退化。

## 7. 万级规模实测性能基线（Release `-O3`，开启 `enable_channel_nudging = true`）

以下为默认 `LazyHananAStarSolver` + `ChannelNudger` 在 10,000 模块 / 20,000 连线（seed 42，含 15% 超长跨度连线，Windows Clang + Ninja Release）下的实测数据（完整历史 `libavoid` 同台对照记录归档于 Git 标签 `v1-libavoid-baseline`）：

| 场景（10,000 模块 / 20,000 连线） | 指标 | 实测表现 |
| :--- | :--- | :--- |
| 同步 4 线程全图冷启动（`D_cold`） | 总耗时 / 合法率 | **2.35 s**（`2346.29 ms`） / **99.53%**（19,906 Valid, 94 DisplayOnly, 0 Failed） |
| 同步单长线局部拖拽（`D_drag`，10 帧） | P50 / P99 | **0.361 ms** / **0.937 ms** |
| 同步单模块拖拽（`A`，200 帧） | P50 / P99 | **0.410 ms** / **4.556 ms**（mean `0.614 ms`，非关联线误提交 0） |
| 同步 20 模块框选拖拽（`B20`，200 帧） | P50 / P99 | **1.445 ms** / **4.986 ms**（mean `1.608 ms`） |
| 同步穿插密集连线区（`C`，200 帧） | P50 / P99 | **0.604 ms** / **1.690 ms**（P95 `1.175 ms`，非关联线误提交 0） |
| `qt_demo` Async 10k 交互冒烟（含箭头/焊点/跳线弧） | `Paint Time` / 增量 `Undo` / `Redo` | **5.66 ~ 8.85 ms** / **0.978 ms** / **0.117 ms** |
| `qt_demo` Sync 10k 局部渲染（含箭头/焊点/跳线弧） | `Paint Time` | **6.84 ~ 8.72 ms** |
