# Local Orthogonal Router 仓库工程约束（AGENTS.md）

本文件为 `local_orthogonal_router` 仓库的长期维护铁律，所有人工维护与 AI Coding Agent 在本仓库开发时均须严格遵守：

1. **文档唯一权威分工**
   - [README.md](README.md) 仅维护项目简介、仓库目录结构、环境要求与快速构建/运行入口，禁止堆砌详细设计或阶段性报告。
   - [docs/ARCHITECTURE.md](docs/ARCHITECTURE.md) 为系统架构、路由算法原理、线程模型、配置字段、C++ API 与性能基线的**唯一权威文档**。

2. **架构文档同步更新规则**
   - 凡修改 `include/localroute/` 公开接口、`localroute::Config` 配置字段、核心布线/分轨算法（`src/`），或 `qt_demo/` 的核心交互/内置演示能力，**必须在所有测试与验收通过后同步更新 `docs/ARCHITECTURE.md`**，再执行最终 Git 提交。

3. **清理一次性过程文档**
   - `phase*_plan.md`、`*_plan.md`、`*_review.md`、`*_results.md` 等均属一次性过程方案或评审记录，**禁止长期残留主干**。相关内容验收并合入 `docs/ARCHITECTURE.md` 后，提交前须将其删除（并已由 `.gitignore` 拦截误提交）。

4. **技术栈与编码红线**
   - 核心静态库 `local_router` 保持**纯 C++17、零第三方依赖**（仅依赖标准库与 `Threads::Threads`）。
   - 代码命名、文件命名（`snake_case.h` / `snake_case.cc`）与排版严格遵循 **Google C++ Style Guide**（仓库根目录已配置 `.clang-format`）。
   - 任何提交合入前，Debug 与 Release 全量单元测试必须 **100% PASS**，核心改动需通过 `bench_large_scale` 性能回归验证。
