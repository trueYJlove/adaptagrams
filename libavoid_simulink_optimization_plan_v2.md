# libavoid 超大规模电气拓扑（万级模块）性能优化与测试构建执行方案（V2 定稿版）

> **适用对象**：本地代码代理（Coding Agent）直接执行
> **目标仓库**：`mjwybrow/adaptagrams`（核心目录：`cola/libavoid/`）
> **目标环境**：Windows 10 x64 + MinGW-w64 G++ 11.2.0 + CMake 3.25.0 + Ninja 1.10.2（兼容 Linux/macOS GCC/Clang）
> **业务场景**：类 Simulink 电气建模与仿真软件，支持 **1,000 ~ 10,000+ 模块、2,000 ~ 20,000+ 连线** 的流畅交互式布线
> **核心交互契约**：
> 1. 拖拽模块时，**仅向求解器提交与移动模块直接相连的连线**；其余非关联连线**零提交、保持绝对静止（允许临时穿模）**。
> 2. 拖拽交互态路由计算（不含 UI 渲染）控制在高帧率预算内（目标：单模块拖拽 P99 < 5ms，框选 20 模块拖拽 P99 < 16ms）。
> 3. 保留 `libavoid` 核心优势：正交避障最短路 A*、端口出线方向约束（`ShapeConnectionPin`）、拐点惩罚与障碍物缓冲距离。
> 4. **演进约束**：第一版仅实现基于 `libavoid` 的局部求解器（最小范围快速落地验证），通过轻量接口 `ILocalRouteSolver` 解耦调度层与求解器，为后续按需替换自研求解器（如局部 Hanan+Midline 正交 A*）预留插槽。

## 1. 现状诊断与架构设计总览

### 1.1 原生 `libavoid` 性能瓶颈（已对照源码核实）

| 瓶颈位置 | 源码位置 | 问题根因 | 万级模块影响 |
| :--- | :--- | :--- | :--- |
| 普通正交连线重复寻路 | `cola/libavoid/connector.cpp:923–944, 986–998` | 仅启用不可见图的 `ConnType_PolyLine` 注册了增量边监听；普通正交连线无条件置 `m_false_path = true`（仅固定路线与 hyperedge 跳过） | 移动单个模块后，非固定正交连线通常会被再次寻路，并导致被压到的无关连线重排 |
| 非空事务重建全局正交图 | `cola/libavoid/router.cpp:641–654, 431–449`<br>`cola/libavoid/orthogonal.cpp:1730` | 非空事务（非 `SimpleRouting`）会销毁并全量扫描重建静态正交可见性图 | 障碍物规模达数千至上万时，全局扫描线建图与搜索空间急剧膨胀 |
| 全局 Nudging 成对遍历与 VPSC | `cola/libavoid/router.cpp:1000`<br>`cola/libavoid/orthogonal.cpp:2533–2594, 3177–3248` | `improveOrthogonalRoutes()` 内每个维度均对正交连线执行成对拆分与交叉统计并跑全局 VPSC，且默认开启 `performUnifyingNudgingPreprocessingStep` | 万级连线规模下成对交叉计算与 VPSC 开销极高 |
| 同位扫描线排序 UB 与构建缺失 | `cola/libavoid/scanline.cpp:294–307`<br>`cola/libavoid/tests/Makefile.am` | `compare_events` 对分别 `new` 的 `Node*` 做指针相减并强转 `int`（64 位下存在未定义行为与非确定性排序）；仓库无 CMake 与大规模压测套件 | 影响同坐标对齐矩形排序确定性；工程集成与量化验收缺失 |

### 1.2 目标架构：内核最小修复 + 可插拔局部窗口路由

采用**三层解耦架构**，第一版完全由外层控制“只算直连线”，**不对 `libavoid` 做脏标记魔改**：

1. **内核层（仅 2 处极小、高安全补丁）**：
   - 新增 `RoutingOption::enableOrthogonalNudging`（默认 `true`），切入点下沉至 `orthogonal.cpp:2547–2584` 内部——**保留共线简化（`simplifyOrthogonalRoutes()`），仅跳过 unifying 与 $O(E^2)$ nudging/VPSC**。
   - 修复 `scanline.cpp:294–307` 堆指针相减强转 `int` 的 64 位 UB，改为基于稳定 ID 的确定性比较。
   - **不修改 `connector.cpp`、`shape.cpp`、`dllexport.h`**。
2. **调度封装层（`LocalOrthogonalRouter` + `SpatialGridIndex`）**：
   - **摒弃全局单一 `Avoid::Router` 实例**。外层维护真正只读并发安全的均匀空间网格索引（`SpatialGridIndex`）。
   - **拖拽交互态**：只收集与移动模块直连的 $k$ 条连线，查询局部窗口障碍物，交给 `ILocalRouteSolver` 求解，并**对每条返回路径做严格几何与全局碰撞复验**（彻底消除 `libavoid` 无解退化返回 2 点直连的误判）。
   - **全图冷启动加载态**：冻结模型，使用 C++17 线程池并发调用无状态的 `ILocalRouteSolver::solve()`，每个工作线程使用私有去重缓冲并独占写入预分配结果槽位。
3. **求解器抽象层（`ILocalRouteSolver`）**：
   - 轻量纯虚接口，隔离具体几何引擎。第一版唯一实现为 `LibavoidLocalSolver`。

## 2. 阶段一：构建系统现代化（CMake + MinGW-w64 + Ninja）

在不破坏原有 Autotools 文件的前提下，新增独立的 `cola/libavoid/CMakeLists.txt`。

### 2.1 `cola/libavoid/CMakeLists.txt` 规范

- CMake 最低版本 `3.16`，C++ 标准 `C++17`（`CMAKE_CXX_STANDARD_REQUIRED ON`）。
- 静态库目标 `avoid`（`add_library(avoid STATIC ...)`）：
  - 头文件导出目录：`${CMAKE_CURRENT_SOURCE_DIR}/..`（即 `cola/`）。
  - 静态宏定义：`target_compile_definitions(avoid PUBLIC LIBAVOID_NO_DLL)`（匹配 `dllexport.h:29–36` 原生宏，无需修改 `dllexport.h`）。
  - 编译选项：GCC/MinGW 在 `Release` 下启用 `-O3 -DNDEBUG`；在 `Debug` 下注意避免在 MSVC 下误触 `<afx.h>`（本机以 MinGW-w64 G++ 11.2.0 为主工具链）。
- 按 `cola/libavoid/Makefile.am:7–29` 完整列出全部 23 个核心源文件：
  - `connectionpin.cpp`, `connector.cpp`, `connend.cpp`, `geometry.cpp`, `geomtypes.cpp`, `graph.cpp`, `junction.cpp`, `makepath.cpp`, `obstacle.cpp`, `orthogonal.cpp`, `router.cpp`, `shape.cpp`, `timer.cpp`, `vertices.cpp`, `viscluster.cpp`, `visibility.cpp`, `vpsc.cpp`, `hyperedge.cpp`, `hyperedgeimprover.cpp`, `mtst.cpp`, `hyperedgetree.cpp`, `scanline.cpp`, `actioninfo.cpp`。
- 可选构建开关：
  - `OPTION(LIBAVOID_BUILD_SIMULINK_EXT "Build Simulink local router extension" ON)`
  - `OPTION(LIBAVOID_BUILD_TESTS "Build unit tests and large-scale benchmarks" ON)`
  - `OPTION(LIBAVOID_BUILD_LEGACY_SMOKE_TESTS "Build selected original libavoid smoke tests" ON)`

### 2.2 本机 Windows PowerShell 构建命令（明确工具链，不依赖 `ctest.exe`）

```powershell
$CMAKE = "D:\Program Files\MATLAB\R2023b\bin\win64\cmake\bin\cmake.exe"
$NINJA = "D:\Program Files\MATLAB\R2023b\toolbox\shared\coder\ninja\win64\ninja.exe"
$GXX   = "D:\SimuNPS-EMT\tools\mingw\bin\g++.exe"
$GCC   = "D:\SimuNPS-EMT\tools\mingw\bin\gcc.exe"

& $CMAKE -S cola/libavoid -B build_libavoid -G Ninja `
  -DCMAKE_MAKE_PROGRAM="$NINJA" `
  -DCMAKE_C_COMPILER="$GCC" `
  -DCMAKE_CXX_COMPILER="$GXX" `
  -DCMAKE_BUILD_TYPE=Release

& $CMAKE --build build_libavoid -j
```

**验收**：`libavoid.a` 及所有测试可执行文件零错误编译链接通过。

## 3. 阶段二：`libavoid` 内核最小补丁（仅 2 处）

> **ABI 提示**：向 `enum RoutingOption` 追加成员会改变 `lastRoutingOptionMarker` 的值（`router.h:858` 用于定长数组），所有调用方需统一使用新头文件重新编译，不得与旧二进制混链。

### 3.1 补丁 A：新增 `enableOrthogonalNudging`（保留路径简化，仅跳过微调）

1. **`cola/libavoid/router.h`**（在 `lastRoutingOptionMarker` 第 290 行之前追加）：
   ```cpp
       //! 控制是否在 improveOrthogonalRoutes() 中执行 unifying 与正交微调 (nudging/VPSC)。
       //! 默认为 true（保持原有行为）；设为 false 时仍保留共线路径简化，但跳过 O(E^2) 交叉排序与 VPSC。
       enableOrthogonalNudging,
   ```
2. **`cola/libavoid/router.cpp`**（`Router::Router(const unsigned int flags)` 约第 94–101 行初始化默认值）：
   ```cpp
   m_routing_options[enableOrthogonalNudging] = true;
   ```
3. **`cola/libavoid/orthogonal.cpp`**（`ImproveOrthogonalRoutes::execute()`，第 2533–2594 行）：
   - **保留**开头第 2540 行的 `simplifyOrthogonalRoutes();`。
   - 当 `!m_router->routingOption(enableOrthogonalNudging)` 时，直接跳过 unifying（第 2554–2566 行）、nudging 循环（第 2570–2584 行）、二次简化、`improveOrthogonalTopology()` 以及检查点缓存构建（第 2545 / 2592 行），直接结束返回。

### 3.2 补丁 B：修复扫描线同位事件比较 UB（`cola/libavoid/scanline.cpp:294–307`）

- 将 `compare_events`（以及 `scanline.cpp:37–49` 如有同类指针平局打破逻辑）中对分别 `new` 的 `Node*` 做 `v1 - v2` 并强转 `int` 的逻辑，替换为**确定性、无 UB 的稳定比较**：
  1. 若 `a->v == b->v`，返回 `0`（相同节点比较为相等，不触发错误断言）。
  2. 若 `a->v != b->v`，优先按节点关联对象的稳定键值比较（例如 `(isObstacle, objID, vertNumber, pos)`）；仅在极端完全相同键时使用 `std::less<const void*>{}(a->v, b->v)` 返回 `-1 / 1`（绝不做指针减法截断为 `int`）。
- 补丁完成后，用对齐、接触、完全重叠的矩形构造回归用例验证无断言崩溃且排序稳定。

## 4. 阶段三：高性能局部路由封装层（`simulink_ext`）

在 `cola/libavoid/simulink_ext/` 目录下新建独立的 C++17 封装库（零第三方依赖，仅依赖 STL 与 `avoid`）。

### 4.1 目录与文件规划

```text
cola/libavoid/simulink_ext/
├── SpatialGridIndex.h        # 均匀空间网格索引（Header-only，支持并发只读查询）
├── ILocalRouteSolver.h       # 轻量求解器抽象接口与通用数据结构（预留扩展口子）
├── LibavoidLocalSolver.h     # 第一版唯一求解器声明（基于局部 Avoid::Router）
├── LibavoidLocalSolver.cpp   # 第一版唯一求解器实现（含局部 ID 映射、扇出共享、所有权安全）
├── LocalOrthogonalRouter.h   # 主调度器接口（面向 ILocalRouteSolver 编程）
├── LocalOrthogonalRouter.cpp # 局部窗口、几何/碰撞复验、增量补障重试、并行冷启动、SVG 导出
└── tests/
    ├── test_framework.h      # 零依赖轻量断言与高精度计时宏
    ├── test_unit.cpp         # 6 大增强功能单元测试
    ├── topology_generator.h  # 万级模块电气拓扑生成器（含长线与高扇出特征）
    └── bench_large_scale.cpp # 1k/3k/5k/10k 统一规模压测与质量/耗时双轨报告
```

### 4.2 线程安全空间索引设计（`SpatialGridIndex.h`）

- **数据结构**：
  - `double m_cellSize = 256.0;`
  - `std::unordered_map<uint64_t, std::vector<BlockId>> m_cells;`（将 `(int32_t cx, int32_t cy)` 按无符号位拼接为 `uint64_t`；网格索引计算使用 `static_cast<int32_t>(std::floor(coord / m_cellSize))`，正确处理负坐标）。
  - `std::unordered_map<BlockId, Entry> m_entries;`（`Entry` 同时保存**精确 `Rect exactBox`** 与 `CellRange`）。
- **核心约束**：
  - **禁止内部可变 `queryStamp`**：`query()` 必须是严格无副作用的 `const` 方法，由调用方/工作线程传入线程私有去重缓冲 `struct QueryScratch { std::vector<BlockId> candidates; };`（先收集再 `sort + unique` 或线程私有哈希集去重），并用 `exactBox` 做精确相交过滤（含边界相切判定）。
  - **同格移动更新精确坐标**：`update(id, newBox)` 即使 `CellRange` 未变，也**必须更新 `m_entries[id].exactBox = newBox`**，仅跳过网格桶的删除/插入。
  - **输入防御**：对 `NaN`/`Inf` 或反向矩形（`minX > maxX`）做显式归一化或拒绝。

### 4.3 轻量求解器抽象层（`ILocalRouteSolver.h` & `LibavoidLocalSolver`）

```cpp
using BlockId = uint32_t;
using PinId   = uint32_t;
using ConnId  = uint32_t;

struct Rect {
    double minX = 0.0, minY = 0.0, maxX = 0.0, maxY = 0.0;
    bool isFiniteAndValid() const;
    bool intersects(const Rect& other, double eps = 0.0) const;
    Rect expanded(double pad) const;
    static Rect unite(const Rect& a, const Rect& b);
};

struct PinSpec {
    PinId id = 0;
    double relX = 0.0;            // 相对模块左上角的 [0, 1] 归一化比例
    double relY = 0.0;
    unsigned int visDirs = 0;     // Avoid::ConnDirLeft / Right / Up / Down
    bool allowFanout = true;      // 电气/Simulink 默认允许同引脚多线扇出
};

struct BlockSpec {
    BlockId id = 0;
    Rect bounds;
    std::vector<PinSpec> pins;
};

struct ConnSpec {
    ConnId id = 0;
    BlockId srcBlock = 0;
    PinId   srcPin   = 0;
    BlockId dstBlock = 0;
    PinId   dstPin   = 0;
};

enum class RouteStatus : uint8_t {
    Valid = 0,               // 通过全部几何、方向与全局障碍缓冲复验
    FallbackDisplayOnly = 1, // 极端退化/超预算降级为 3 段正交显示线（不计入避障成功）
    Failed = 2               // 不可路由（如非法输入或求解异常）
};

struct RoutePolyline {
    ConnId connId = 0;
    std::vector<std::pair<double, double>> points;
    RouteStatus status = RouteStatus::Failed;
    int retryCount = 0;
};

struct LocalRouterConfig {
    double gridCellSize           = 256.0;
    double initialWindowPadding   = 250.0; // 初始局部窗口外扩像素
    int    maxExpandRetries       = 3;     // 最大重试轮次（增量补障 + 扩窗）
    size_t maxObstaclesPerWindow  = 120;   // 单次局部求解的障碍物预算上限
    double shapeBufferDistance    = 8.0;   // 连线与模块边界的避障间距
    double segmentPenalty         = 50.0;  // 拐点惩罚
    double idealNudgingDistance   = 6.0;   // 松手微调时的平行线间距
    bool   nudgeOnSettle          = false; // 拖拽期间永不微调；仅松手时按此开关局部微调
};

// 单次局部窗口求解请求（批次内冻结只读视图；请求内每个 ConnId 必须唯一）
struct LocalRouteRequest {
    Rect windowBounds;
    std::vector<const BlockSpec*> obstacles; // 窗口内完整障碍物（必含所有待求连线的起止模块）
    std::vector<const ConnSpec*>  conns;     // 本次需计算的脏连线
    const LocalRouterConfig*      config        = nullptr;
    bool                          enableNudging = false;
};

// 可插拔局部求解器抽象接口（无状态、线程安全）
class ILocalRouteSolver {
public:
    virtual ~ILocalRouteSolver() = default;
    virtual const char* name() const = 0;

    // 必须为 req.conns 中每条唯一 ConnId 产出一个对应的 RoutePolyline
    virtual void solve(const LocalRouteRequest& req,
                       std::vector<RoutePolyline>& outRoutes) const = 0;
};
```

#### 第一版唯一实现：`LibavoidLocalSolver` 的 5 条强制工程约束

1. **隔离局部 ID 命名空间**：不要把业务的 `BlockId` 和 `ConnId` 直接传入 `Avoid::Router`（原库 shape/conn 共用 ID 空间会撞号）。在 `solve()` 内部重新分配连续局部 ID：`shapeLocalId = 1 .. M`，`connLocalId = M+1 .. M+K`；同时确保传入 `ShapeConnectionPin` 的 `classId` **严格非零**（`pinClassId = pinIndex + 1`，见 `connectionpin.cpp:80`）。
2. **显式配置扇出共享**：`libavoid` 方向引脚默认独占（`connectionpin.cpp:49,68`）。创建 `ShapeConnectionPin` 后必须调用 `pin->setExclusive(!pinSpec.allowFanout)`，防止多线扇出时部分连线无法绑定引脚。
3. **严格遵守对象所有权**：在栈上创建 `Avoid::Router router(Avoid::OrthogonalRouting)`；所有 `new ShapeRef(...)`、`new ShapeConnectionPin(...)`、`new ConnRef(...)` 均由 `router` 析构时统一释放，**禁止用 `std::unique_ptr` 管理或手动 `delete`**（否则触发 `shape.cpp:48–52` / `connector.cpp:109–115` 的 `abort()`）。必须在 `router` 离开作用域前将 `displayRoute()` 坐标完整拷贝到 `outRoutes`。
4. **高性能参数配置**：
   ```cpp
   router.setTransactionUse(true);
   router.setRoutingParameter(Avoid::shapeBufferDistance, cfg.shapeBufferDistance);
   router.setRoutingParameter(Avoid::segmentPenalty, cfg.segmentPenalty);
   router.setRoutingParameter(Avoid::idealNudgingDistance, cfg.idealNudgingDistance);
   router.setRoutingOption(Avoid::performUnifyingNudgingPreprocessingStep, false);
   router.setRoutingOption(Avoid::improveHyperedgeRoutesMovingJunctions, false);
   router.setRoutingOption(Avoid::nudgeSharedPathsWithCommonEndPoint, false);
   router.setRoutingOption(Avoid::enableOrthogonalNudging, req.enableNudging);
   ```
5. **禁止在并发工作线程中调用 `libavoid` 调试输出**：严禁在 `solve()` 中调用 `outputInstanceToSVG` / `outputDiagram`（会修改全局 `LC_NUMERIC`）。

### 4.4 主调度器、严格几何复验与增量补障（`LocalOrthogonalRouter`）

1. **模型生命周期维护**：
   - `addBlock` / `removeBlock`（删除模块时同步清理或解绑所有关联连线与邻接表）。
   - `moveBlock` / `moveBlocksBatch`（批量移动时**先更新全部模块坐标和 `SpatialGridIndex`，再统一收集去重的脏连线集**）。
   - 维护统计计数器（例如 `totalSubmittedConns`），便于单元测试精确断言非关联线提交次数为 0。

2. **前置退化检查（调用求解器前）**：
   - 检查连线起止引脚坐标是否重合、模块矩形是否退化、或引脚绝对坐标是否落在第三方障碍物内部。此类物理上无合法避障解的情况直接进入 `FallbackDisplayOnly`（3 段正交 L/Z 展示线），绝不误计为 `Valid`。

3. **逐线严格几何与全局碰撞复验（`validateRoute()`）**：
   - 彻底修复“两点即成功”漏洞（`connector.cpp:1267–1273` 寻路失败会退化返回 2 点直连）。对每条返回路线逐项检查：
     1. 坐标全部有限且点数 $\ge 2$；
     2. 起点/终点与真实引脚坐标误差 `< 1e-3`；
     3. 每段非零且严格轴对齐（水平或垂直，$|dx| < 1e-4$ 或 $|dy| < 1e-4$）；
     4. 首段方向符合 `srcPin.visDirs`（从 `srcPin` 向外），末段方向符合 `dstPin.visDirs`（按**从 `dstPin` 向外**的方向校验，即末段逆向矢量匹配 `dstPin.visDirs`）；
     5. **回查完整 `SpatialGridIndex` 做全局碰撞复验**：逐段检查是否穿入任意非起止模块的膨胀包围盒（考虑 `shapeBufferDistance - eps`）；对起止模块本身，仅豁免首段离开 `srcBlock` 与末段进入 `dstBlock` 的必要逃逸通道，不允许回穿自身模块主体。

4. **迭代补障与自适应扩窗（替代盲目按中点截断）**：
   - 初始窗口：取连线（或聚合小组）起止点包围盒外扩 `initialWindowPadding`，查出局部障碍物（必含起止模块；若小组起止模块本身超预算则先拆分为单线独立求解）。
   - 求解后逐线运行 `validateRoute()`：
     - 通过则标记 `RouteStatus::Valid` 并锁存结果；
     - 未通过且未达 `maxExpandRetries` 时，**仅针对失败连线**重试：将 `validateRoute()` 在完整空间索引中检测到的**实际碰撞障碍物**优先补入局部障碍集合（在 `maxObstaclesPerWindow` 预算内），并将窗口外扩 `padding *= 2` 再解；
     - 若重试耗尽或障碍物预算溢出，生成确定性 3 段正交显示兜底线并标记 `RouteStatus::FallbackDisplayOnly`（绝不伪装成 `Valid`）。

5. **万级模块并发冷启动（`routeAllParallel(size_t numThreads = 0)`）**：
   - 若 `numThreads == 0`，取 `std::max<size_t>(1, static_cast<size_t>(std::thread::hardware_concurrency()))`。
   - 批次执行期间冻结模型只读；预分配稠密任务数组 `std::vector<RoutePolyline> taskResults(totalConns)`；工作线程原子领取下标、使用线程私有 `QueryScratch`、只写自己的 `taskResults[idx]`；所有线程 `join()` 后由主线程单线程写回最终结果表并统一执行 SVG 导出。

## 5. 阶段四：测试套件与大规模压测平台构建

### 5.1 原生回归烟测（Legacy Smoke Tests）

在 CMake 中独立编译 5 个原生程序（`example`, `connectionpin01`, `inlineShapes`, `checkpoints01`, `overlappingRects`），在构建目录下创建可写 `output/` 子目录作为工作目录直接运行。
> **定位说明**：这 5 个原生程序仅验证补丁未导致编译失败、断言崩溃或非零退出（**烟测**），不替代几何正确性断言。

### 5.2 6 大增强功能单元测试（`test_unit.exe`）

全部用例以独立可执行程序运行并输出逐项 `PASS/FAIL`：

| 用例名称 | 测试目标 | 强制断言条件 |
| :--- | :--- | :--- |
| `SpatialGridBasic` | 空间索引正确性与边界 | 覆盖随机 10,000 矩形 + **负坐标、同格移动、贴边相切、跨多格与删除**；精确查询结果与暴力 AABB 扫描 100% 一致 |
| `DirtyOnlyNoRerouteOnOverlap` | **核心需求：移模块压线，线不动** | 模块 C 自带连线并移动到直接覆盖连线 A–B 中段；断言 **C 的连线被提交求解**，而 **A–B 提交求解次数严格为 0** 且点列逐点完全不变 |
| `LocalObstacleAvoidance` | 避障、缓冲、扇出与反例 | 断言正交、出线方向、`shapeBufferDistance` 净空、同引脚多线扇出有效；**专门构造内核无解退化返回 2 点直连的反例**，断言 `validateRoute()` 必须将其判为非 `Valid` |
| `AdaptiveWindowExpansion` | 增量补障与扩窗验证 | 构造挡墙使第 1 轮必然碰撞/失败（断言 `retryCount >= 1`），后续轮次自动补入碰撞障碍并扩窗绕行，最终路径通过完整空间索引复验且为 `Valid` |
| `OverlappingBlocksFallback` | 重叠容错与三态分离 | 分别测试两端重叠、同点引脚、第三方障碍压住引脚；断言零崩溃，且物理无解用例明确返回 `FallbackDisplayOnly`（绝不误报 `Valid`） |
| `ParallelColdStartDeterminism` | 并行冷启动与同坐标排序 | 500 模块（含刻意对齐、接触、重叠矩形）分别用 1 线程和 4 线程重复执行 `routeAllParallel`；按 `ConnId` 对比合法率与路径一致性 |

### 5.3 大规模电气拓扑生成器（`tests/topology_generator.h`）

固定 `seed = 42` 生成可复现的合成电气/Simulink 拓扑，统一覆盖 **4 档规模：`1,000`、`3,000`、`5,000`、`10,000` 模块**（约 `2,000 ~ 20,000` 条连线）：
- **空间布局**：多子系统簇网格排布 + 随机抖动；模块尺寸 `80x50` ~ `140x80`，通道间距 `40` ~ `120`。
- **引脚与连线**：左侧输入（`ConnDirLeft`）、右侧输出（`ConnDirRight`），包含高扇出母线节点（单引脚扇出 4~8 条线）；85% 短中距局部连线 + 15% 长跨度反馈/总线连线。

### 5.4 性能与质量双轨基准压测（`bench_large_scale.exe`）

所有场景均在 `1k / 3k / 5k / 10k` 四档统一执行，并将每档帧数提升至 **200 帧**（确保 P99 具统计意义而非等同于 Max），结果输出至控制台 Markdown 表与 `benchmark_results.csv`（含 `solver_name` 字段）：

1. **场景 A — 单模块连续拖拽（200 帧）**：随机选 10 个不同度数（含高扇出）模块各连续拖拽 20 帧；计时覆盖“更新坐标与空间索引 -> `rerouteAffected` -> 几何复验提交”。
2. **场景 B — 框选批量拖拽（200 帧）**：框选相邻 20 模块与 50 模块连续拖拽，记录每帧 `Avg / P50 / P95 / P99 / Max`。
3. **场景 C — 模块拖过密集连线区（200 帧，穿模零干扰测试）**：将带连线的模块拖过最密集通道，断言非关联线提交求解数严格为 `0`。
4. **场景 D — 全图冷启动 + 原生基线对比**：
   - 使用同一份输入拓扑对比：`Native Global (Default)`、`Native Global (Tuned)`、`LocalOrthogonalRouter (1T)`、`LocalOrthogonalRouter (Multi-Thread)`。
   - **进程外超时控制**：通过 PowerShell 包装或独立子进程以 **15 秒上限**运行原生基线（包含 1k 档）；实际超时标记 `TIMEOUT`，因前置档超时跳过更大规模标记 `SKIPPED`。
   - **必报质量指标**：同步输出每档 `Valid / FallbackDisplayOnly / Failed` 比例、局部窗口障碍物 `Avg / P99 / Max`、平均重试次数、峰值内存 RSS，以及冷启动分阶段耗时（建索引 / 求解 / 全局复验 / 合并）。
5. **SVG 目视验证导出**：单线程导出 10k 场景局部视口的 `drag_before.svg` 与 `drag_after.svg`（高亮本次重算线、区分 `Valid` 与 `FallbackDisplayOnly`）。

## 6. 阶段五：本地 Agent 执行顺序与分层验收标准

请按以下顺序分步执行，每完成一步输出该步实际命令、编译/运行日志摘要后再进入下一步：

1. **构建验证**：编写 `cola/libavoid/CMakeLists.txt`，用 MinGW-w64 + Ninja 编译 `avoid` 与 5 个原生烟测并在构建侧 `output/` 目录运行。
2. **应用最小内核补丁**：修改 `router.h`、`router.cpp`、`orthogonal.cpp`（补丁 A）与 `scanline.cpp`（补丁 B），重编并重跑 5 个原生烟测。
3. **实现 `simulink_ext`**：编写 `SpatialGridIndex.h`、`ILocalRouteSolver.h`、`LibavoidLocalSolver.h/.cpp`、`LocalOrthogonalRouter.h/.cpp`（第一版仅实现 `LibavoidLocalSolver`）。
4. **运行增强单元测试**：编译并运行 `test_unit.exe`，确保 6 个用例 100% 通过。
5. **运行万级规模双轨压测与导出 SVG**：在 `Release (-O3)` 下运行 `bench_large_scale.exe`（原生基线配进程外 15s 超时），输出 `benchmark_results.csv` 与 `drag_before.svg` / `drag_after.svg`。

### 6.1 分层验收标准（硬性正确性门槛 vs 性能目标）

| 类别 | 验收项 | 门槛 / 目标 |
| :--- | :--- | :--- |
| **硬性正确性** | 5 个原生烟测 + 6 个增强单元测试 | **100% 通过**，零断言崩溃 |
| **硬性正确性** | 非关联连线提交求解率（含穿模场景） | **严格 0%**（0 条提交） |
| **硬性正确性** | 常规非重叠拓扑 `Valid` 避障合法率 | **> 98%**（漏障/2 点直连不得计入 `Valid`） |
| **性能目标** | 10,000 模块单模块拖拽单帧耗时（P99，不含 UI 渲染） | **< 5.0 ms**（实测报告为准） |
| **性能目标** | 10,000 模块框选 20 模块拖拽单帧耗时（P99，不含 UI 渲染） | **< 16.0 ms**（实测报告为准） |
| **性能目标** | 10,000 模块多线程冷启动全图布线耗时 | **< 3.0 s**（同步汇报质量与峰值内存） |
