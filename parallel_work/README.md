# 并行施工包

本目录用于给并行实施人员分发最小必要上下文。实现人员默认只需要阅读：

1. `00_common_contracts.md`
2. 自己对应的岗位施工说明
3. 岗位说明中列出的源码文件

PRD、SAD、SOUP、完整接口文档由 PM/架构师维护。除非岗位说明明确要求，实施人员不需要通读 `docs/*.md`。

## 岗位文件

| 文件 | 岗位 | 当前优先级 |
| --- | --- | --- |
| `01_track_a_core_api.md` | Track A：Core/Public API 负责人 | P0，并行支持 Track B |
| `02_track_b_dicom_import.md` | Track B1：DICOM 导入负责人 | P0，下一步主实施 |
| `03_track_b_mpr.md` | Track B2：MPR 负责人 | P1，等待真实 `VolumeData` 后接入 |
| `04_track_c_planning_project_ui.md` | Track C：Planning/Project/UI 负责人 | P1 |
| `05_track_d_drr_validation.md` | Track D：DRR/Validation 负责人 | P1 |
| `06_qa_traceability.md` | QA/验证与追踪负责人 | P1，伴随所有轨道 |

## 当前施工顺序

1. Track B1 先实现 DCMTK 单序列 CT 导入。
2. Track A 同步补齐导入所需 core 类型、错误码和测试夹具。
3. Track B2 在真实 `VolumeData` 可用后推进 MPR 三视图。
4. Track C 接器械规划和 `.mprproj` 工程包。
5. Track D 完善 CPU/CUDA DRR 与 phantom 验证。
6. QA 持续维护 CTest、功能测试和追踪矩阵。

## 协作规则

- 每个人只修改自己岗位说明中的“可修改范围”。
- 需要跨模块改接口时，先在对应 PR/提交说明中写明调用方、被调用方和测试影响。
- 不要因为自己任务方便而绕过 `measurement_core` 公共数据接口。
- 不要把 Qt、VTK、DCMTK、CUDA 类型引入 `measurement_core` public headers。
- 不要在 CMake configure 阶段联网下载依赖。
- 每次交付至少运行 `ctest --preset debug`；涉及 Release 或依赖配置时同时运行 Release preset。
