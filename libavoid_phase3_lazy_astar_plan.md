# libavoid 万级布线三阶段执行方案：需求纠偏 + `LazyHananAStarSolver` 核心提速

> **适用对象**：本地代码代理（Coding Agent）直接在当前 `cola/libavoid/simulink_ext/` 代码库上执行
> **前置基础**：二阶段已完成的 `SpatialGridIndex`、`ILocalRouteSolver`、`LocalOrthogonalRouter`、`AsyncLocalRouter`、分项计时 `Profile` 与 13/13 单元测试
> **本阶段核心目标**：
> 1. **纠偏交互需求**：默认关闭第三方阻挡线重算，严格满足“移动模块压到无关连线时，线保持静止、允许穿模”的 Simulink 交互语义。
> 2. **攻克 87.3% 建图瓶颈**：通过已预留的 `localroute::ILocalRouteSolver` 插槽，新增零链表、缓存友好的 **`LazyHananAStarSolver`**，在**不降低任何几何与全局碰撞复验标准**的前提下，将 10k 同步单拖 P99、框选 20 模块 P99 与 4 线程冷启动推向目标门槛（`<5 ms` / `<16 ms` / `<3 s`）。
> 3. **零改动 `libavoid` 内核**：本阶段完全在 `cola/libavoid/simulink_ext/` 内完成，不再修改原库任何文件。

## 1. 任务一：纠偏第三方阻挡重算开关（回归“移模块压线，线不动”）

### 1.1 源码改动清单

1. **`cola/libavoid/simulink_ext/ILocalRouteSolver.h`**：
   - 在 `localroute::Config` 中新增开关（默认 `false`）：
     ```cpp
     bool rerouteThirdPartyBlocked = false;
     ```
2. **`cola/libavoid/simulink_ext/LocalOrthogonalRouter.cpp`**：
   - `moveBlocksBatch()`（约第 130 行）：仅当 `cfg.rerouteThirdPartyBlocked` 为 `true` 时才调用 `invalidateObstacle(m.first, blocks.at(m.first).rect);`。
   - `publish()` / `clearSegments()`（约第 159–172 行）：仅当 `cfg.rerouteThirdPartyBlocked` 为 `true` 时维护 `segments` 空间网格；默认 `false` 时跳过全部 `segments.insert()` / `segments.remove()`，彻底消除拖拽和冷启动提交阶段的线段网格开销。
3. **`cola/libavoid/simulink_ext/tests/test_unit.cpp`**：
   - `DirtyOnlyNoRerouteOnOverlap`：在默认 `cfg.rerouteThirdPartyBlocked == false` 下，将带直连线的模块 C 移动至直接压住连线 A–B 中段，严格断言：
     - C 的直连线被提交求解；
     - A–B **未被标 `Pending`、提交求解次数为 0、点列逐点完全不变（允许穿模）**。
   - 原有第三方挡线测试（`FastReuseAndThirdPartyObstruction` 中涉及第三方挡线的子项）：显式设置 `cfg.rerouteThirdPartyBlocked = true` 保留该可选模式的回归验证。

## 2. 任务二：实现 `LazyHananAStarSolver`（替换 `LibavoidLocalSolver` 作为默认高性能求解器）

### 2.1 为什么它能消灭 87.3% 的建图耗时

二阶段 Profiling 证明：`LibavoidLocalSolver` 87.3% 的耗时花在 `generateStaticOrthogonalVisGraph`（大量 `new/delete` 构建 `VertInf`/`EdgeInf` 指针链表），且 10k 中 2,303 条跨度 ≥10,000 的长线吃掉了 78.6% 建图时间。

`LazyHananAStarSolver` 萃取 `libavoid` 最核心的 3 个算法精华（**膨胀边界+通道中线非均匀网格、带方向与最小拐点启发式的 A*、引脚出线方向约束**），但采用**连续数组 + 阻塞位图**实现：
- **零指针链表、零频繁堆分配**（配合 `thread_local` 工作缓冲复用内存）；
- **120 障碍物建位图仅需数十微秒**，比 `libavoid` 扫描线建图快 1～2 个数量级；
- **复用 `LocalOrthogonalRouter::accept()` 完全相同的全局碰撞复验与重试机制**，保证结果质量绝不缩水。

### 2.2 文件与构建接入

- 新建 `cola/libavoid/simulink_ext/LazyHananAStarSolver.cpp`。
- 在 `cola/libavoid/simulink_ext/ILocalRouteSolver.h` 中声明工厂函数：
  ```cpp
  std::shared_ptr<const ILocalRouteSolver> lazyHananAStarSolver();
  ```
- 在 `cola/libavoid/simulink_ext/CMakeLists.txt` 中将 `LazyHananAStarSolver.cpp` 加入 `local_router` 库。
- 保留 `libavoidSolver()` 不删除，便于同台压测对比。

### 2.3 `LazyHananAStarSolver::solve(const Request &q)` 详细算法设计

#### 第 1 步：构造非均匀正交坐标网格 `xs`、`ys`
1. 计算起点 `a`、终点 `b`（与 `LocalOrthogonalRouter` 完全一致的归一化引脚坐标公式），并找到起止模块 `src`、`dst` 及其引脚 `sp`、`dp`。
2. 收集 X / Y 坐标集合（均为 `std::vector<double>`）：
   - 起止引脚坐标：`a.x, b.x` 加入 `xs`；`a.y, b.y` 加入 `ys`。
   - 所有障碍物 `bObs`（含 `src`/`dst`）的膨胀包围盒 `r = bObs->rect.pad(q.cfg.buffer)`：将 `r.l, r.r` 加入 `xs`，`r.t, r.b` 加入 `ys`。
   - 窗口最外围逃逸环（保证绕行空间）：取所有障碍物与起止点的全局最小/最大坐标，向四周外扩 `std::max(q.cfg.buffer * 2.0, 32.0)`，加入 `xs`、`ys`。
   - 排序并按容差 `1e-6` 去重（保留精确的 `a.x, b.x, a.y, b.y` 不被合并漂移）。
3. **通道中线注入（借鉴 `libavoid` 居中美学）**：
   - 当 `xs.size() < 280` 时，对相邻坐标间距 `xs[i+1] - xs[i] > q.cfg.buffer * 2.5` 的空隙，补充中点 `0.5 * (xs[i] + xs[i+1])`（Y 轴同理），再次排序去重。这样在控制网格尺寸（典型 `W, H <= 320`）的同时，让连线优先走通道中央而非紧贴模块边缘。

#### 第 2 步：构建二维阻塞位图（微秒级预处理）
定义两条紧贴 `LocalOrthogonalRouter::accept()` 几何语义的阻塞判定数组（使用 `thread_local` 复用缓冲，每轮 `assign(size, 0)`）：
- `blockedH[iy * (W - 1) + ix]`：水平微段 `(xs[ix], ys[iy]) -> (xs[ix+1], ys[iy])` 是否穿入任意障碍物膨胀框内部；
- `blockedV[iy * W + ix]`：垂直微段 `(xs[ix], ys[iy]) -> (xs[ix], ys[iy+1])` 是否穿入任意障碍物膨胀框内部。

标记规则（严格对齐 `LocalOrthogonalRouter.cpp` 的 `crosses(u, v, rect)` 开区间语义）：
- 对每个障碍物膨胀框 `r = bObs->rect.pad(q.cfg.buffer)`：
  - 用 `std::lower_bound` 定位满足 `r.l < xs[ix]` 且 `xs[ix+1] < r.r`（或中点落在 `(r.l, r.r)` 内）的水平微段范围，以及满足 `r.t < ys[iy] < r.b` 的 `iy` 范围，将对应 `blockedH` 置 `1`；
  - 同理，对满足 `r.l < xs[ix] < r.r` 的 `ix` 与中点落在 `(r.t, r.b)` 内的垂直微段，将对应 `blockedV` 置 `1`。
  - **严格注意边界条件**：沿着膨胀框**边线本身**（`ys[iy] == r.t` 或 `r.b`，`xs[ix] == r.l` 或 `r.r`）走是不算穿入内部的（与 `crosses` 保持完全一致），因此允许沿障碍物外扩边界贴边绕行。
- **起止模块引脚逃逸通道豁免（关键）**：
  - 仅针对起点 `a` 沿 `sp.dirs` 允许的方向，从 `a` 所在格点出发，将**完全位于 `src.rect.pad(buffer)` 内部且未穿入任何第三方障碍物**的出口射线微段解除阻塞（即 `blockedH/V` 置为特殊豁免标记，确保只允许离开 `src`，不允许穿过第三方障碍）。
  - 终点 `b` 沿 `dp.dirs` 允许的反向入口射线做完全对称的豁免处理。
  - 若 `src` 或 `dst` 与其他障碍物重叠导致某微段同时被第三方障碍阻塞，**绝不豁免第三方障碍**（仍保持阻塞）。

#### 第 3 步：带方向状态 + 最小拐点启发式的正交 A*
- **状态空间**：`state = ((iy * W + ix) << 2) | dirIdx`（`dirIdx ∈ {0:Up, 1:Down, 2:Left, 3:Right}`），总状态数 `4 * W * H`。
- **工作内存复用**：在 `thread_local` 结构体中复用 `gCost`、`parent`、`visitedStamp`（使用单调递增的 `uint32_t runStamp` 避免每次 `fill` 整个大数组，仅重置 `runStamp++`！），大幅减少冷启动期间的内存带宽开销。
- **初始状态**：定位起点格 `(sx, sy)`，仅将满足 `sp.dirs` 的方向 `(sx, sy, d)` 以 `g = 0` 压入最小堆 `std::priority_queue`。
- **终止判定**：弹出状态 `(ix, iy, d)` 满足 `ix == gx && iy == gy`，且**进入方向 `d` 的反方向**在 `dp.dirs` 中（例如终点引脚允许 `Left` 出线，则连线必须以 `Right` 方向进入终点）。
- **状态转移（最多 3 个后继：直行、左转、右转，禁止 180° 掉头）**：
  - 检查对应 `blockedH` 或 `blockedV` 未被阻塞；
  - `stepCost = stepLen + (newDir != curDir ? q.cfg.penalty : 0.0)`；
  - 针对贴着障碍物边界的微段可加极小软偏置（如 `+ 0.01`，小于任何真实长度差），促使等长路径优先走通道中线，视觉更接近 `libavoid`。
- **可采纳最小拐点启发式 `h(ix, iy, dir)`**：
  - `manhattan = std::abs(xs[ix] - b.x) + std::abs(ys[iy] - b.y)`；
  - 根据当前点相对目标点 `(b.x, b.y)` 的方向关系、当前朝向 `dir` 以及终点允许入口方向 `dp.dirs`，精确估算最少剩余拐点数 `minBends ∈ {0, 1, 2, 3}`：
    - 若已在同一直线且正朝目标前进且终点方向匹配：`minBends = 0`；
    - 否则按方向是否与目标位移同号快速判别 `1` 或 `2`（保守下界，保证严格可采纳 `h <= h*`）；
  - `h = manhattan + minBends * q.cfg.penalty`。
- **搜索步数安全上限**：设置合理的节点弹出上限（如 `maxPops = 40000`），极端死锁时安全返回空路径交由外层重试/降级。

#### 第 4 步：路径回溯、共线合并与 Profile 埋点
- 从终止状态沿 `parent` 回溯至起点，反转序列后合并相邻共线点，确保首尾精确等于 `a` 和 `b`。
- 计时埋点对齐 `ILocalRouteSolver.h` 的 `Profile`：
  - `xs/ys` 离散化 + 阻塞位图构建耗时写入 `q.profile->graphMs`；
  - A* 搜索 + 回溯合并耗时写入 `q.profile->searchMs`。

## 3. 任务三：单元测试与双求解器万级同台压测

### 3.1 单元测试扩展（`tests/test_unit.cpp`）

1. 将核心单元测试（`DirtyOnlyNoRerouteOnOverlap`、`LocalObstacleAvoidance`、`AdaptiveWindowExpansion`、`OverlappingBlocksFallback`、`ParallelColdStartDeterminism`、`FastReuseAndThirdPartyObstruction`、`AsyncDiscardsStaleResults`）对 **`lazyHananAStarSolver()`** 全部跑通一遍，同时保留 `libavoidSolver()` 回归测试，确保 Debug & Release **100% 通过**。
2. 新增 1 个针对 `LazyHananAStarSolver` 的边界专项测试：验证贴边绕行、通道居中、多线扇出共享引脚、以及起点/终点被第三方障碍半包围场景下均能通过 `LocalOrthogonalRouter::accept()` 严格全局复验。

### 3.2 万级压测对比（`tests/bench_large_scale.cpp`）

- 在 Release（`-O3 -DNDEBUG`，seed 42）下，使用**完全相同的合成拓扑与 `rerouteThirdPartyBlocked = false`（默认穿模不动）**，运行 `LazyHananAStarSolver` 全 4 档（1k / 3k / 5k / 10k，三轮取中位数，建议 A/B/C 场景每轮统一 200 帧），并与二阶段 `LibavoidLocalSolver` 结果同表对比。
- 输出：
  - 同步汇总表与 `benchmark_results.csv`（含 `solver_name = LazyHananAStarSolver`）；
  - 10k 分项计时汇总（重点对比 `graphMs` 与 `searchMs` 降幅）；
  - `AsyncLocalRouter` 10k 交互代理耗时（重点观察单模块/20 模块松手完成时间与后台全量完成时间）；
  - 10k 局部视口对比图 `drag_after_lazy_astar.svg`，用于人工核对布线美观度与避障质量。

## 4. 三阶段硬性验收标准

| 类别 | 验收项（10,000 模块 / 20,000 连线） | 目标门槛 |
| :--- | :--- | :--- |
| **硬性正确性** | Debug / Release 全部单元测试（双求解器） | **100% 通过**，零断言崩溃 |
| **硬性正确性** | 默认穿模模式下，非关联连线误提交求解数 | **严格 0 条**（`Pending` 与重算均为 0） |
| **硬性质量** | 10k 全量冷启动 `Valid` 避障合法率（同一套全局碰撞复验） | **≥ 98.5%**（不低于 `LibavoidLocalSolver` 的 98.12%） |
| **性能目标** | 同步 4 线程全图冷启动耗时 | **< 3.0 s**（对比二阶段 54.1 s） |
| **性能目标** | 同步单模块拖拽 P99（200 帧，不含 UI 绘制） | **< 5.0 ms**（对比二阶段 332.9 ms） |
| **性能目标** | 同步框选 20 模块拖拽 P99（不含 UI 绘制） | **< 16.0 ms**（对比二阶段 497.7 ms） |
