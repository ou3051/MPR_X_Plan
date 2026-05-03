# 工程施工组织与 VS2022 构建计划 v0.1

## 1. 构建基线

- 平台：Windows x64。
- 生成器：Visual Studio 17 2022。
- 构建系统：CMake presets。
- 编译标准：C++20。
- 默认编译入口：
  - 配置：`cmake --preset vs2022-x64-debug`
  - 编译：`cmake --build --preset app-debug`
  - 测试：`ctest --preset debug`
- 本地 DCMTK 开发包默认接入 `D:/lib/xray/DCMTK`；CMake 会优先匹配其中的 `dcmtk-*-build`，再匹配 `dcmtk-*-install`。
- 仅有 `bin/*.dll` 和工具程序时视为 runtime，不启用编译链接。

## 2. 并行施工轨道

| 轨道 | 责任 |
| --- | --- |
| Track A | Core/Public API、CMake、依赖、测试入口 |
| Track B | DCMTK DICOM、坐标、MPR |
| Track C | 器械规划、3D、工程包 |
| Track D | CPU/CUDA DRR、phantom 验证 |

## 3. 阶段顺序

1. 工程骨架、VS2022 CMake presets、依赖策略。
2. Core 数据接口与坐标测试。
3. DICOM 导入与 MPR。
4. 器械规划与工程包。
5. CPU/CUDA DRR。
6. 回归测试与追踪矩阵。

## 4. 修改边界

- `measurement_core` 不得依赖 Qt、VTK、DCMTK、CUDA。
- 外部库类型不得出现在公共 domain 接口中。
- VTK 与物理坐标转换只放在 `measurement_vtk_adapter`。
- `.mprproj` 由 `measurement_persistence` 统一读写。
- CMake configure 不得隐式下载依赖。
