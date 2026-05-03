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

## 5. 当前实施分派

阶段 1 工程骨架和依赖基线已完成，当前应从 Track B 开始推进真实数据链路。下一轮实施顺序如下：

| 顺序 | 实施人/角色 | 任务 | 输入 | 完成标准 |
| --- | --- | --- | --- | --- |
| 1 | Track B：DICOM/MPR 负责人 | 实现 DCMTK 单序列 CT 导入，完成 tag 读取、HU 转换、slice 排序、`VolumeData` 输出 | `measurement_dicom`、DCMTK 3.7.0、`PublicInterfaces_v0.1` | `DicomVolumeLoader::loadFolder` 可读标准 CT folder；DICOM tag/HU 单元测试通过 |
| 2 | Track A：Core/Public API 负责人 | 补齐导入所需 core 类型、错误码和测试夹具，保持接口文档同步 | `measurement_core`、`PublicInterfaces_v0.1` | core 不引入外部库；坐标和错误路径测试通过 |
| 3 | Track B：MPR 负责人 | 接入 `vtkImageReslice + MprViewState` 三正交视图采样 | `measurement_mpr`、`measurement_vtk_adapter` | Axial/Sagittal/Coronal reslice 参数测试通过；可显示三视图基础结果 |
| 4 | Track C：Planning/Project 负责人 | 器械创建编辑、VTK actor 映射、`.mprproj` manifest 扩展 | `measurement_planning`、`measurement_persistence` | 器械保存加载、几何 mesh、端点/方向测试通过 |
| 5 | Track D：DRR/Validation 负责人 | 在真实 `VolumeData` 上完善 CPU DRR reference，并准备 CUDA DRR kernel 边界 | `measurement_drr`、phantom 数据 | sphere/cylinder/step-HU phantom 验证通过 |
| 6 | QA/验证负责人 | 建立功能测试脚本和追踪矩阵，固化验收入口 | `ctest`、PRD/SAD/TestScope | `ctest --preset debug` 作为每日验收入口，追踪矩阵可关联需求和测试 |

PM/架构师负责维护 `docs/OpenIssues_v0.1.md`，任何待确认项不得只留在正文描述中。
