# Local Orthogonal Router (Simulink-Scale C++17 Routing Engine)

零外部依赖的纯 C++17 万级局部正交布线引擎，支持 10,000 模块 / 20,000 连线规模下的亚毫秒级同步局部重算、异步非阻塞增量更新与实时 Qt 交互可视化演示。代码命名、文件命名与格式规范遵循 **Google C++ Style Guide**（仓库根目录已配置 `.clang-format`）。

> 历史对照基线（包含 `adaptagrams/libavoid` 原生对比与三阶段同台压测记录）已归档于 Git 标签 `v1-libavoid-baseline`，可通过 `git checkout v1-libavoid-baseline` 查看。

## 目录结构

```text
.
├── .clang-format                      # Google C++ 代码格式化规范
├── CMakeLists.txt
├── README.md
├── doc/
│   └── architecture.md                # 架构设计与 API 接入说明
├── include/localroute/
│   ├── local_route_solver.h           # 求解器抽象 LocalRouteSolver 与配置/几何类型
│   ├── spatial_grid_index.h           # 2D 空间网格索引
│   ├── local_orthogonal_router.h      # 同步局部正交路由器
│   └── async_local_router.h           # 异步增量会话与优先级调度
├── src/
│   ├── lazy_hanan_a_star_solver.cc    # 零堆链表、阻塞位图 + 两阶段正交 A* 求解器
│   ├── local_orthogonal_router.cc     # 局部窗口查询、快速复用与全局碰撞复验
│   └── async_local_router.cc          # 后台工作线程与版本丢弃
├── tests/
│   ├── topology_generator.h
│   ├── local_router_test.cc           # 9 项核心单元与边界测试
│   ├── bench_large_scale.cc           # 1k / 3k / 5k / 10k 同步/异步/分项压测
│   └── run_benchmark.py
└── qt_demo/                           # 基于 Qt Widgets 视口裁剪 + LOD 的万级交互画布
    ├── CMakeLists.txt
    ├── canvas_widget.h / .cc
    ├── main_window.h / .cc
    └── main.cc
```

## 构建与运行（Windows / VS 2022 + Clang + Ninja + Qt 5）

```powershell
# 配置 Release（含核心库、测试与 qt_demo）
cmd /c "`"C:/Program Files/Microsoft Visual Studio/2022/Community/VC/Auxiliary/Build/vcvars64.bat`" && cmake -S . -B out-Release -G Ninja -DCMAKE_MAKE_PROGRAM=`"C:/Program Files/Microsoft Visual Studio/2022/Community/Common7/IDE/CommonExtensions/Microsoft/CMake/Ninja/ninja.exe`" -DCMAKE_CXX_COMPILER=`"C:/Program Files/LLVM/bin/clang++.exe`" -DCMAKE_BUILD_TYPE=Release -DCMAKE_PREFIX_PATH=C:/Qt/Qt5.12.11/5.12.11/msvc2017_64 && cmake --build out-Release -j 8"

# 运行单元测试
ctest --test-dir out-Release --output-on-failure

# 运行 10,000 模块基准压测
out-Release\bench_large_scale.exe local4 10000

# 运行 Qt 万级交互演示（已通过 windeployqt 部署运行依赖）
out-Release\qt_demo\qt_demo.exe

# 使用 clang-format 检查 / 格式化代码
clang-format --dry-run --Werror include/localroute/*.h src/*.cc tests/*.h tests/*.cc qt_demo/*.h qt_demo/*.cc
```
