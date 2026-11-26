# Local Orthogonal Router 架构设计与 API 接入说明

本文档对应仓库精简后的纯 C++17 正交布线引擎（命名空间 `localroute`），遵循 Google C++ Style Guide（仓库根目录 `.clang-format` 采用 `BasedOnStyle: Google`），涵盖整体分层架构、核心算法原理、线程模型、配置参数说明及 C++ / GUI 接入范例。

## 1. 设计目标与核心契约

本引擎面向类 Simulink 电气拓扑与框图建模软件，针对 **1,000 ~ 10,000+ 模块、2,000 ~ 20,000+ 连线** 的交互式编辑场景设计，遵循以下四条核心契约：

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
* **内部单元免哈希优化**：`Query(area)` 遍历查询矩形覆盖的网格时，仅对边界格元素执行精确相交判断 `boxes_.at(id).Intersects(area)`；完全落入查询框内部的网格直接批量拷贝，最后排序去重。
* **并发只读安全**：`Query()` 为严格无副作用的 `const` 方法（不在实例内维护可变 `queryStamp`），支持 `RouteAllParallel()` 多线程并发只读查询。

### 3.2 多级路由管线（`LocalOrthogonalRouter::SolveOne`）

对每一条待求连线，按成本由低到高依次经过 4 个阶段，任一阶段产出均需通过统一的 `accept()` 全局碰撞与方向复验：

1. **前置退化检测**：
   * 起止引脚重合 → 标记 `Outcome::kCoincidentPins`、`Status::kDisplayOnly`；
   * 起点或终点引脚落在第三方模块膨胀缓冲框内部 → 标记 `Outcome::kBuriedPin`、`Status::kDisplayOnly`。
2. **轻量快路径（`Config::fast_paths = true` 且非松手微调时启用）**：
   * **第一优先级 — 整线平移复用（`RouteMethod::kReuse`）**：当起止端点位移量 `(dx, dy)` 完全相等（典型为框选批量拖动内部连线）时，整条旧折线平移后直接复验，命中即微秒级返回。
   * **第二优先级 — L / Z 轻量候选（`RouteMethod::kSimple`）**：优先尝试 2 段 L 型折线及中点/缓冲边界对齐的 3 段 Z 型折线并复验；只要拖动后存在无障碍的简洁 2/3 段通路，折线会立即自动收直，彻底消除旧中段残留导致的 4 拐回折尾巴。
   * **第三优先级 — 端点修补复用（`RouteMethod::kReuse`）**：仅当 `Simple` 被障碍物阻挡无法通过且旧路径 `points.size() >= 3` 时，保持原折线中段不变，仅正交修正首尾拐点衔接新引脚位置并复验。
3. **局部窗口迭代补障 + 求解器调用（`RouteMethod::kNative`）**：
   * 初始窗口取起止点包围盒外扩 `padding + buffer`，随后每轮重试按 `padding * 2^attempt` 扩张（最多 `retries` 轮）。
   * 若窗口内障碍数超过 `budget`（默认 `120`），优先保留 `required` 集合（起止模块 + 前序轮次实际碰撞到的第三方模块），其余按到起止线段垂直距离用 `std::nth_element` 截断保留最近的障碍物。
   * 求解器返回候选折线后，`accept()` 逐段回查**完整全局 `SpatialGridIndex`**；若发生碰撞，将命中的实际障碍物写入 `required` 并进入下一轮重试。
4. **兜底降级**：
   * 若重试耗尽或 `required.size() > budget`，保留确定的 2/3 段正交展示折线并标记 `Status::kDisplayOnly`（附具体 `Outcome`），绝不伪装为 `Status::kValid`。

### 3.3 默认求解器：`LazyHananAStarSolver`

`LazyHananAStarSolver()` 返回的实现摒弃了传统全局可见性图指针链表，采用纯连续数组与线程局部工作区复用，核心流程如下：

1. **非均匀正交坐标离散化**：
   * 收集起止引脚精确坐标、所有局部障碍物膨胀框 `rect.Pad(buffer)` 的四条边界及外围逃逸环，排序去重并保护引脚坐标不漂移。
   * 当坐标数 `< 200` 时，按空隙宽度优先向大通道注入中点（上限 `240`），使路径自然走通道中央而非紧贴模块边缘。
2. **微秒级二维阻塞位图（`edge_h` / `edge_v`）**：
   * 针对每个障碍物膨胀框，用二分定位其覆盖的网格区间并标记位掩码（`kBlkOther`、`kBlkSrc`、`kBlkDst`、`kEdgeBorder`）。
   * 预计算起点沿 `sp.dirs` 离开 `src` 缓冲框、终点沿 `dp.dirs` 离开 `dst` 缓冲框的合法逃逸射线，禁止在起止缓冲框内部提前拐弯，同时绝不豁免任何第三方障碍。
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
   * 全图冷启动（`RouteAllParallel` 合并后，`full_graph = true`）：全图 `Status::kValid` 连线均属于可移动集，按确定性升序 `Id` 执行单趟全图等距分轨。
   * 交互拖拽与局部编辑（`RerouteAffected`、`RouteConnector` 及 `AsyncLocalRouter::Poll` 局部收口，`full_graph = false`）：仅本次重算的直连线集为 `mutable_conns`；联合包围盒外扩 `max(2 * nudge_distance, 80.0)` 查询到的邻近非关联连线（参与上限 `256` 条）**仅作为只读占用轨道（Fixed Anchors）参与冲突检测与避让**，严禁修改任何非 `mutable_conns` 连线，确保拖拽时非关联线 100% 静止。
2. **中间段与电气网表语义（`NetKey`）**：仅平移 `pts.size() >= 4` 连线的中间正交段（索引 `[1, pts.size() - 3]`），绝不篡改起止引脚位置与出线方向；按 `NetKey { src_block, src_pin }` 归组——**同一源引脚扇出允许共享主干，不同 `NetKey` 独立信号强制分轨**（同源固定线已存在时新扇出优先锚定其主干）。
3. **坐标带分桶 + 区间冲突聚类 + 固定轨道避让分配**：分别处理垂直与水平中间段；冷启动按 `<= 1.5 px` 聚合共线坐标带，交互态按 `< nudge_distance` 收集冲突簇。若簇内均为可动线且不撞固定轨道，按 Minimal-Crossing 排序在 `[free_min, free_max]` 内居中均匀摊开；当可动线与只读固定线坐标差 `< nudge_distance - 1e-3` 且区间重叠时，固定线纹丝不动，可动线按**最小交叉优先 + 距原坐标最近 + 与全部重叠固定线/已放轨道间距 `>= nudge_distance`** 的准则落位至空闲轨道。
4. **安全净宽夹紧 + 逐线全局复验回滚**：通过 `SpatialGridIndex::Query` 找出同跨度左右/上下障碍物膨胀框及相邻拐点约束得到安全通道 `[free_min, free_max]`（空间极窄不足 `2 * nudge_distance` 时自动选择最大净空子隙中心）；每条调整后走 `ValidateAndSimplifyRoute()` 全量碰撞与正交复验，若任一线冲突仅单线回滚，**100% 保留原有 `Valid` 合法率**。

### 3.5 异步会话模型（`AsyncLocalRouter`）

* **双副本状态隔离**：主线程（Owner）仅操作 `model_`，后台工作线程独享一份独立的 `LocalOrthogonalRouter` 副本，彻底避免读写锁竞争或工作线程触碰 UI 数据。
* **拖动合并与代际丢弃**：高频拖动调用 `Move()` 时，同模块多次位移直接合并覆盖到 `edits_` 表并递增 `generation_`；后台计算完毕提交结果时，若结果所属代际已落后于当前最新位置，自动丢弃过期路径。
* **视口与选区优先**：调用 `Prioritize(viewport, selection)` 后，后台队列优先处理当前选中块和可见区域内的连线；单波次 `pending_` 清空时在 Owner 侧收口触发局部/全图 `ChannelNudger`。

## 4. 核心数据结构与配置参考

### 4.1 几何与拓扑模型（`include/localroute/local_route_solver.h`）

* **`Point { double x, y; }`**、**`Rect { double l, t, r, b; }`**（世界坐标系，要求 `l <= r` 且 `t <= b`）。
* **`Direction`**：位掩码枚举 `kUp = 1`, `kDown = 2`, `kLeft = 4`, `kRight = 8`，可按位或组合。
* **`Pin { Id id; double x, y; unsigned dirs; }`**: `x, y` 为相对模块矩形的 `[0.0, 1.0]` 归一化坐标；支持多条连线共享同一引脚扇出。
* **`Block { Id id; Rect rect; std::vector<Pin> pins; }`**、**`Conn { Id id, src, sp, dst, dp; }`**。

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

### 4.3 路由结果状态 `Status` 与 `Outcome`

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
2. **异步拓扑变更规则**：`AsyncLocalRouter` 面向已建好拓扑的编辑会话；执行增删模块/连线等拓扑结构变更时，应先销毁旧会话（或 `Clone()` 重建后调用 `StartDirty()` / `StartAll()`）再创建新会话。
3. **GUI 渲染与自由建模能力（`qt_demo`）**：
   * 渲染层通过 `view().QueryBlocks(viewport)` 做模块视口裁剪、为每条连线缓存包围盒做 $O(1)$ 裁剪，并在低缩放倍率下启用合批绘制与 LOD。
   * 支持从左侧**图元库（`LibraryWidget`）**拖拽 7 类预设模块到画布（或从 `0 模块 (空白画布)` 起步），右侧面板及右键菜单支持动态修改选中模块四向端口个数（`左/右 0~16`、`上/下 0~8`），并支持悬停端口拖拽手动连线与 `Del` 删除。
   * 工具栏内置 `预设: B1-B2 10线平行对连` 与 `预设: B9/B3/B8 级联布线` 视觉验收场景，配合 `通道分轨 (Nudging)` 复选框可一键开关对比分轨效果。
4. **近距模块缓冲**：若业务画布允许两个相连模块间距小于 `2 * cfg.buffer`（默认 `16 px`），建议将 `cfg.buffer` 调至 `4.0 ~ 6.0` 以减少缓冲框互叠导致的 `Status::kDisplayOnly` 退化。

## 7. 万级规模实测性能基线（Release `-O3`，开启 `enable_channel_nudging = true`）

以下为默认 `LazyHananAStarSolver` + `ChannelNudger` 在 10,000 模块 / 20,000 连线（seed 42，含 15% 超长跨度连线，Windows Clang + Ninja Release）下的实测数据（完整历史 `libavoid` 同台对照记录归档于 Git 标签 `v1-libavoid-baseline`）：

| 场景（10,000 模块 / 20,000 连线） | 指标 | 实测表现 |
| :--- | :--- | :--- |
| 同步 4 线程全图冷启动 | 总耗时 / 合法率 | **2.38 s**（`2377.42 ms`） / **99.53%**（19,906 Valid, 94 DisplayOnly, 0 Failed） |
| 同步单模块拖拽（200 帧） | P50 / P99 | **0.404 ms** / **3.635 ms**（非关联线误提交 0） |
| 同步 20 模块框选拖拽（200 帧） | P50 / P99 | **1.687 ms** / **3.982 ms** |
| 同步穿插密集连线区（200 帧） | P50 / P99 | **0.682 ms** / **1.883 ms**（非关联线误提交 0） |
| `qt_demo` Async 10k 交互冒烟 | `Route Time` / `Paint Time` | **0.017 ~ 0.057 ms** / **7.07 ~ 10.16 ms** |
