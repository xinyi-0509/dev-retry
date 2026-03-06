<div align="center">

# LibHGP

**多语言几何算法库 | Multi-Language Geometry Processing Library**

![C++17](https://img.shields.io/badge/C++-17-blue?logo=cplusplus)
![Python](https://img.shields.io/badge/Python-3.8+-yellow?logo=python)
![pybind11](https://img.shields.io/badge/pybind11-%E2%9C%93-green)
![CMake](https://img.shields.io/badge/CMake-3.16+-red?logo=cmake)
![CGAL](https://img.shields.io/badge/CGAL-%E2%9C%93-blue)
![Platform](https://img.shields.io/badge/Platform-Windows%20%7C%20Linux-lightgrey)

> 针对实验室既有几何算法实现（liblgp / libhgp）代码零散、接口混乱且缺乏 Python 调用与可视化支持的问题，  
> 本项目对底层算法库进行工程化整合与接口重构，构建支持 **C++ / Python / Web** 三种调用形态的几何算法库。

</div>

---

## 目录

- 项目简介
- 核心特性
- 系统架构
- 仓库目录概览
- 依赖与环境
- 构建（Linux / Windows）
- Python 扩展（hgp_py）说明与示例
- 常见配置选项
- 发布与运行时注意事项
- 开发计划
- 贡献
- 许可证

---

## 项目简介

LibHGP（Library for HGP / LGP）是将实验室历史遗留的两套几何算法库（liblgp / libhgp）进行工程化整合与接口重构后的产物。目标是：

- 统一算法实现并维护单一 API 边界；
- 同时提供动态库与静态库以满足不同部署需求；
- 提供 pybind11 封装以供 Python 调用；
- 提供基于 FastAPI 的 Web 服务和基于 Three.js 的前端可视化（后续模块）；
- 形成跨平台（Windows / Linux）的一致构建与发布流程。

---

## 核心特性

- 整合 libhgp / liblgp，设计 API 边界层以解决跨模块 ABI 兼容问题
- 构建动态库 `libhgp` 与静态库 `libhgp_static`
- 为静态库启用 PIC（POSITION_INDEPENDENT_CODE），便于静态链接跨平台使用
- 基于 pybind11 封装 `hgp_py` Python 扩展（���目中封装约 147 个 2D/3D 算法接口）
- 使用统一的 CMake 构建体系，支持 MSVC（Windows）与 GCC/Clang（Linux）
- 通过 vcpkg 管理第三方依赖（推荐用于 Windows 与一致的依赖管理）
- 统一 MSVC 运行时（/MD 或 /MDd）以保证运行时一致性（与 vcpkg triplet 保持兼容）

---

## 系统架构（简要）

调用层：
- C++ 客户端（libhgp）
- Python（hgp_py）
- Web（FastAPI 后端 + Three.js 前端，后续）

中间层：
- API 边界、类型转换（pybind11）、ABI 适配

核心层：
- 算法实现：hgp2d, hgp3d, hgpmesh, hull3D, kdtree, cgal 等
- 第三方：CGAL、Clipper 等

数据/展示层：
- Model（几何数据结构）
- Visualizer（前端 Three.js 渲染 / 可视化工具）

---

## 仓库目录（概览）

```
LibHGP/
├── CMakeLists.txt
├── libhgp.h
├── hgp2d.cpp
├── hgp3d.cpp
├── hgpio.cpp
├── hgpmesh.cpp
├── hull3D.cpp
├── hull3D.h
├── kdtree.cpp
├── kdtree.h             
├── lgp_api.cpp               # libhgp部分导出函数封装
├── hgp_py.cpp                # pybind11 绑定（hgp_py 模块）
├── clipper/                  # Clipper 库文件
├── local_libs/               # 本地头文件或第三方轻量依赖
├── build/                    # 构建目录
├── out/                      # 构建输出目录（动态库等）
├── out_libhgp/               # 客户端头文件
├── images/                   # 文档图片资源
└── README.md


## 依赖与环境

- CMake >= 3.16  （CMakeLists 中使用 3.16）
- C++17 支持（CMAKE_CXX_STANDARD 17）
- 编译器：
  - Windows: MSVC 2019 / 2022（Visual Studio 2019/2022）
  - Linux: GCC 9+ / Clang 等
- 第三方库：
  - CGAL（配置为 REQUIRED，使用 CGAL::CGAL 和 CGAL::CGAL_Core）
  - pybind11（用于 Python 绑定）
  - Clipper（已内置或放在 clipper/）
- Python >= 3.8（构建/使用 Python 扩展时）
- vcpkg（推荐在 Windows 上使用以统一依赖管理）

vcpkg 示例：
- Windows:
  vcpkg install cgal[core] --triplet x64-windows-static-md
  vcpkg install pybind11 --triplet x64-windows-static-md
- Linux:
  vcpkg install cgal[core]

---

## 构建说明

本项目使用 CMake 管理构建。CMakeLists.txt 已包含如下约定（摘要）：

- cmake_minimum_required(VERSION 3.16)
- MSVC 平台上显式设置运行时为 MultiThreadedDLL（/MD）或 MultiThreadedDebugDLL（/MDd）
- 两个库目标：libhgp (SHARED) 与 libhgp_static (STATIC)
- 为静态库设置 POSITION_INDEPENDENT_CODE ON
- 可选构建项 BUILD_PYTHON_BINDINGS（默认 ON），查找 pybind11 并生成 hgp_py 模块

下面给出常见平台的构建步骤。

### Linux（建议在拥有 CGAL 等依赖的环境）
```bash
git clone https://github.com/xinyi-0509/LibHGP.git
cd LibHGP
mkdir build && cd build

cmake .. \
  -DCMAKE_BUILD_TYPE=Release \
  -DCMAKE_TOOLCHAIN_FILE=/path/to/vcpkg/scripts/buildsystems/vcpkg.cmake

cmake --build . --config Release
```

### Windows（MSVC，使用 vcpkg）
```powershell
git clone https://github.com/xinyi-0509/LibHGP.git
cd LibHGP
mkdir build; cd build

cmake .. `
  -G "Visual Studio 17 2022" -A x64 `
  -DCMAKE_TOOLCHAIN_FILE="C:/path/to/vcpkg/scripts/buildsystems/vcpkg.cmake" `
  -DVCPKG_TARGET_TRIPLET="x64-windows-static-md"

cmake --build . --config Release
```

### 仅构建/禁用 Python 绑定
默认会构建 Python 绑定（BUILD_PYTHON_BINDINGS=ON）。若希望禁用：
```bash
cmake .. -DBUILD_PYTHON_BINDINGS=OFF
cmake --build .
```

---

## Python 扩展（hgp_py）说明

- 构建选项：BUILD_PYTHON_BINDINGS（ON/OFF）
- CMake 会调用 find_package(pybind11 CONFIG REQUIRED) 并通过 pybind11_add_module(...) 生成 `hgp_py` 模块
- `hgp_py` 链接到静态库 `libhgp_static`（静态链接可以实现自包含分发）
- 推荐在构建环境中使用与目标 Python 一致的解释器与 pybind11（通过 vcpkg 或系统包安装）

示例 Python 使用（示例函数名需根据实际绑定调整）：
```python
import hgp_py

# 假设绑定中存在 Point2D, distance 等接口
p1 = hgp_py.Point2D(0.0, 0.0)
p2 = hgp_py.Point2D(3.0, 4.0)
print("distance:", hgp_py.distance(p1, p2))
```

部署注意：
- Windows 下如果使用静态链接（libhgp_static）并将其编译进 hgp_py，则通常不需要额外的 DLL；若使用动态库 libhgp，则需要同时部署 libhgp.dll。
- Linux 下遵循常规 .so 部署与 LD_LIBRARY_PATH／rpath 配置。

---

## 常见配置与源码要点（根据当前 CMakeLists.txt）

- 强制 CMake policy CMP0091 在 project() 之前设置以控制 MSVC 运行时行为：
  cmake_policy(SET CMP0091 NEW)
- MSVC 运行时： set(CMAKE_MSVC_RUNTIME_LIBRARY "MultiThreaded$<$<CONFIG:Debug>:Debug>DLL")
- 默认启用本地库路径（local_libs）作为 PRIVATE include
- 使用 find_package(CGAL REQUIRED COMPONENTS Core)，并将 CGAL::CGAL、CGAL::CGAL_Core 链接到目标
- pybind11 通过 CONFIG 模式查找并生成模块 hgp_py（pybind11_add_module）
- 静态库 libhgp_static 设置 POSITION_INDEPENDENT_CODE ON 以便安全地与共享对象链接

---

## 发布与运行时注意事项

- 构建时请确认使用的 vcpkg triplet 与运行时选项一致（特别是 Windows MSVC 的 /MD vs /MT）
- 若打算发布 Python wheel，请先确保 hgp_py 对应平台的运行时依赖都已静态或打包好
- 推荐使用 CI（例如 GitHub Actions）自动化构建并对 Linux/Windows 提供二进制发布包

---

## 开发计划（Roadmap）

短期（已完成项）：
- libhgp / liblgp 整合与 API 边界层（已完成）
- CMake 跨平台构建体系（已完成）
- pybind11 Python 扩展（约 147 个接口，已完成）
- Clipper、CGAL、KD 树、凸包等模块整合（已完成）

中期 / 长期（待办）：
- 完善单元测试与持续集成
- 补全并公开 API 文档（Doxygen / Sphinx）
- 提供 FastAPI 后端与 Three.js 前端的演示与部署脚本
- 发布 PyPI / 二进制 Release（Windows .pyd / Linux .so）
- 探索 WebAssembly 调用形态（Emscripten / WASM）

---

## 贡献

欢迎提交 issue 或 PR。建议在贡献前先打开 Issue 讨论设计或改动点，或创建分支按功能/修复命名（例如 feature/pybind-cleanup 或 fix/cmake-runtime）。

贡献流程概览：
1. Fork 仓库
2. 新建分支并实现变更
3. 提交 PR，描述变更内容与测试情况
4. 等待 review 与合并

---

## 许可证

本项目使用 MIT 许可证（LICENSE 文件中说明）。如需其他许可或有合作事宜，请联系仓库维护者。

---

<div align="center">
  Made with ❤️ | 2025.02 – 2025.07
</div>
