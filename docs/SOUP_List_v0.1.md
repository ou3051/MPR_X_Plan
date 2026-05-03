# SOUP 清单 v0.1

文档状态：草案
版本：v0.1
日期：2026-05-03

## 1. 已配置依赖

| 名称 | 版本/来源 | 用途 | 获取方式 | 当前状态 |
| --- | --- | --- | --- | --- |
| Qt | Qt6，本机 CMake package | 桌面 UI | 系统 SDK | Debug/Release 配置中已发现 |
| VTK | VTK9，本机 CMake package | 3D、MPR、VTK adapter | 系统 SDK | Debug/Release 配置中已发现 |
| CUDA Toolkit | CUDA v12.8 | CUDA DRR 路径 | 系统 SDK | Debug/Release 配置中已发现 |
| GoogleTest | v1.14.0 / f8d7d77c06936315286eb55f8de22cd23c188571 | 单元测试与算法验证 | `third_party/googletest` 显式拉取 | Debug/Release 测试已使用 |
| DCMTK | DCMTK-3.6.8 / 139972c69896afdbcc5e58828e017b3b9c26cbf3 | DICOM 解析 | 系统优先，允许 `third_party/dcmtk` fallback | 当前系统未发现，DICOM loader 受控返回依赖缺失 |

## 2. 依赖控制要求

- CMake configure 阶段不得隐式联网下载依赖。
- `third_party` 依赖必须由显式脚本或明确命令拉取。
- 所有依赖必须固定版本、tag 或 commit。
- 医疗软件发布基线必须记录依赖版本、许可证、用途和验证结果。
