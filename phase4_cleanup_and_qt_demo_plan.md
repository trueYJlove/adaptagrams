# 四阶段执行方案：仓库独立瘦身（脱离 libavoid）+ 本地 Qt 万级交互体验程序

> **适用对象**：本地代码代理（Coding Agent）执行
> **核心目标**：
> 1. **仓库彻底瘦身**：在保留 Git 历史标签（`v1-libavoid-baseline`）的前提下，删除整个 `adaptagrams-master/` 与 `libavoid` 依赖，将仓库整理为**零第三方依赖、纯 C++17（约 1,300 行核心代码）的独立正交路由引擎**。
> 2. **搭建本地 Qt 交互体验程序（`qt_demo/`）**：基于本机 Qt 环境构建可真实操作 1,000 ~ 10,000 模块拓扑的可视化建模画布，直观验证 Simulink 级拖拽手感、穿模零重算与同步/异步模式对比。

## 1. 任务一：仓库瘦身与目录重构

### 1.1 安全保留历史基线（清理前必做）
在删除任何旧文件前，先打一个 Git 标签锁定包含 `libavoid` 同台压测证据的当前提交，方便日后随时回溯：
```bash
git tag -a v1-libavoid-baseline -m "Archive baseline with libavoid comparison and phase 3 benchmarks"
```

### 1.2 目标仓库目录结构
删除整个 `adaptagrams-master/`（`libavoid`, `libcola`, `libvpsc`, `libtopology`, `libdialect`, Autotools, 旧测试）、`LibavoidLocalSolver.cpp` 以及临时历史压测目录，整理为以下干净的独立 C++17 工程：

```text
.
├── CMakeLists.txt                  # 顶层统一构建：核心库 + 单测 + 压测 + 可选 Qt Demo
├── README.md                       # 项目说明、架构简介与构建运行指南
├── .gitignore
├── include/
│   └── localroute/
│       ├── ILocalRouteSolver.h     # 移除 libavoidSolver() 声明，纯 C++17 STL
│       ├── SpatialGridIndex.h
│       ├── LocalOrthogonalRouter.h
│       └── AsyncLocalRouter.h
├── src/
│   ├── LazyHananAStarSolver.cpp    # 唯一默认求解器（零外部依赖）
│   ├── LocalOrthogonalRouter.cpp
│   └── AsyncLocalRouter.cpp
├── tests/
│   ├── topology_generator.h
│   ├── test_unit.cpp               # 移除 libavoid/scanline.h 与 libavoidSolver() 对照项
│   └── bench_large_scale.cpp       # 移除 Avoid::Router 原生基线，保留 1k/3k/5k/10k 同步/异步/分项压测
└── qt_demo/
    ├── CMakeLists.txt
    ├── main.cpp
    ├── MainWindow.h / .cpp         # 顶部工具栏、规模切换、模式切换与状态栏
    └── CanvasWidget.h / .cpp       # 高性能视口裁剪自绘建模画布（支持万级模块流畅拖拽）
```

### 1.3 顶层 `CMakeLists.txt` 要求
- CMake ≥ 3.16，C++17，纯静态库 `local_router`（仅链接 `Threads::Threads`，零外部库依赖）。
- 提供清晰选项：
  - `OPTION(LOCALROUTE_BUILD_TESTS "Build unit tests and benchmarks" ON)`
  - `OPTION(LOCALROUTE_BUILD_QT_DEMO "Build Qt interactive demo" ON)`
- 瘦身完成后先运行全部单元测试（Debug/Release 均 100% 通过），确认剥离 `libavoid` 后零编译错误、零回归。

## 2. 任务二：搭建本地 Qt 交互测试程序（`qt_demo/`）

### 2.1 第一步：探测本机 Qt SDK 与编译器 ABI 匹配（关键！）
请本地 Agent 先执行只读探测，确认本机 Qt 安装位置与编译器工具链：
- 检索常见路径（如 `C:\Qt\`, `D:\Qt\`, 环境变量 `QTDIR` / `CMAKE_PREFIX_PATH` / `PATH`），识别 Qt 版本（Qt 6 或 Qt 5）及其编译 ABI（`msvc2019_64` / `msvc2022_64` / `mingw_64`）。
- **ABI 强一致要求**：必须使用与本地 Qt 库**同 ABI 的编译器**构建 `qt_demo`（例如 Qt 为 `msvc2022_64` 时可用 Clang-CL / MSVC ABI 或 MSVC；Qt 为 `mingw_64` 时使用配套 MinGW G++），避免链接期 ABI 不匹配。
- 构建完成后自动调用 `windeployqt.exe`（或配置启动脚本注入 Qt `bin/` 到 `PATH`），确保双击或命令行运行 `qt_demo.exe` 不会缺少 `Qt*.dll` / `platforms/qwindows.dll`。

### 2.2 防 Qt 渲染成为瓶颈的画布架构设计（核心！）
> **重要警示**：在 10,000 模块 + 20,000 连线场景下，如果直接往 `QGraphicsScene` 里塞 30,000 个独立 `QGraphicsItem` 并每帧重建 `QPainterPath`，**Qt 自身的渲染与 BSP 索引就会先卡成 PPT**，导致无法真实感受路由算法的性能。

因此 `CanvasWidget` **必须采用“轻量自绘 `QWidget` + 视口裁剪（Viewport Culling）+ LOD”方案**：
1. **视口空间裁剪**：
   - 利用画布当前平移/缩放矩阵算出世界坐标可见矩形 `worldViewport`。
   - **模块绘制**：直接通过 `SpatialGridIndex`（或在渲染侧维护只读副本）只查询并绘制 `worldViewport` 内的模块（局部缩放下通常仅 30～200 个）。
   - **连线绘制**：为每条连线缓存世界坐标包围盒 `routeBounds`（仅在该条线更新时顺手更新），绘制时做一次 $O(1)$ 矩形相交测试，跳过视口外连线；处于拖拽中的关联连线始终置顶高亮绘制。
2. **缩放分级细节（LOD）**：
   - 放大查看（`scale >= 0.4`）：绘制完整模块圆角矩形、模块 ID 标签、输入/输出引脚小圆点、清晰正交连线与拐点。
   - 中等缩放（`0.15 <= scale < 0.4`）：省略文本与引脚细节，仅绘制矩形块与 1px 连线。
   - 全局俯瞰（`scale < 0.15`）：简化绘制（关闭抗锯齿 `QPainter::Antialiasing`），确保 10,000 模块全览缩略视图拖动缩放依然顺滑。
3. **拖拽更新闭环（双模式可一键切换）**：
   - **异步模式（默认，基于 `AsyncLocalRouter`）**：
     - `mouseMoveEvent`：更新被拖模块坐标 → 调用 `asyncRouter->moveBlocks(...)`（~0.2 ms）→ 立即用 `preview()` 或平滑预览重绘；
     - 通过 `QTimer`（16ms / 60Hz）调用 `asyncRouter->pollResults(128)` 批量拉取后台算好的新路径，刷新缓存并触发 `update()`；
     - `mouseReleaseEvent`：通知松手并快速补齐剩余连线。
   - **同步模式（基于 `LocalOrthogonalRouter`）**：
     - `mouseMoveEvent` 直接同步调用 `syncRouter->moveBlocksBatch()` + `rerouteAffected()` 并立即 `update()`，让你直观对比同步（3.8ms P99）与异步（0.2ms 主线程）的真实手感差异。

### 2.3 交互功能与实时 HUD 监控面板

#### 工具栏控制项
| 控件 | 功能说明 |
| :--- | :--- |
| 规模下拉框 | 一键切换并重建 **`1,000` / `3,000` / `5,000` / `10,000`** 模块拓扑（复用 `topology_generator.h`） |
| 路由模式切换 | **`Async（异步）`** vs **`Sync（同步）`** 实时切换对比 |
| 穿模策略开关 | **`第三方压线重算（默认关闭）`** 复选框——默认关闭以体验“移模块压线，线不动、直接穿模”；勾选后可直观对比重算差异 |
| 视角操作 | `适应全图 (Fit All)`、`重置局部 100% 视角`、`重新全图布线 (Route All)` |

#### 鼠标与键盘交互（对标 Simulink）
- **滚轮**：以鼠标光标为中心平滑缩放（`0.05x ~ 4.0x`）。
- **中键拖拽 / 空格 + 左键拖拽**：平滑平移画布。
- **左键点击 / 拖拽单个模块**：选中并实时拖动模块，高亮该模块直连的连线。
- **左键空白处拉框（Rubber Band）**：框选多个模块（如 20～50 个）后整体拖拽。
- **快捷键 `R`**：重置视角到高密度局部区域；**`F`**：适应全图。

#### 左上角半透明 HUD 实时性能浮层
每次拖拽/渲染实时刷新以下指标，明确分离**算法耗时**与 **Qt 绘制耗时**：
- `Scale`: 当前拓扑规模（例如 `10,000 blocks / 20,000 conns`）
- `Mode`: `Async` / `Sync` & `ThirdPartyReroute: OFF/ON`
- `FPS`: 实时帧率
- `Route Time`: 本帧路由耗时（同步模式为实际求解 ms；异步模式为提交+拉取 ms）与最近 100 帧 P99
- `Paint Time`: 本帧 Qt `paintEvent` 绘制耗时（ms），以及当前视口实际绘制的 `visibleBlocks / visibleConns`
- `Route Status`: `Valid`（深蓝实线） / `Pending`（灰蓝虚线） / `DisplayOnly`（橙色实线）实时计数

## 3. 执行顺序与验收标准

1. **先探测本地 Qt 环境**：确认 Qt 路径、版本与编译器 ABI，若本机未找到 Qt 安装路径则暂停并向用户汇报已检索位置，请用户指定 Qt 目录。
2. **打标签并完成仓库瘦身**：创建 `v1-libavoid-baseline` tag，重构为独立目录结构，编译并验证精简后的单元测试（100% PASS）与 10k 基准压测无退化。
3. **实现并编译 `qt_demo`**：编写 `MainWindow` 与高性能视口裁剪 `CanvasWidget`，完成编译与 DLL 部署（`windeployqt`）。
4. **启动冒烟与验收汇报**：汇报可执行的 `qt_demo.exe` 路径、启动命令，以及在 10,000 模块模式下验证冷启动加载、单模块拖拽、框选 20 模块拖拽时的实测 `Route Time` 与 `Paint Time` 数据。
