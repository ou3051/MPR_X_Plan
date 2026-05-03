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
| DCMTK development | 3.7.0 / tag `DCMTK-3.7.0` / commit `ccfd10b84ff3c9a40b7b331698aedf06d421fc43` | DICOM 解析开发库 | 本机编译版本 `D:\lib\xray\DCMTK\dcmtk-3.7.0-build` | 已接入 CMake，Debug/Release 可发现开发 target |
| DCMTK install/runtime | 3.7.0 / tag `DCMTK-3.7.0` / commit `ccfd10b84ff3c9a40b7b331698aedf06d421fc43` | DICOM 工具与安装树 | 本机安装路径 `D:\lib\xray\DCMTK\dcmtk-3.7.0-install` | 作为 build tree 之后的 fallback |
| DCMTK source fallback | `DCMTK-3.7.0` / commit `ccfd10b84ff3c9a40b7b331698aedf06d421fc43` | DICOM 解析源码 fallback | 允许 `third_party/dcmtk` fallback | 当前未使用，脚本已固定 tag/commit |

## 2. DCMTK 3.7.0 来源确认

| 证据 | 值 |
| --- | --- |
| 官方发布页 | `https://support.dcmtk.org/redmine/projects/dcmtk/files` |
| 官方源码包 | `dcmtk-3.7.0.tar.gz`，发布日期 `2025-12-15 12:05` |
| 官方源码包 SHA256 | `4158ecde05904b075204db1ce07b0f82922dd62ab79586a5ad50cea22e92be08` |
| 官方 zip 包 SHA256 | `7d7f4ac1f617cdcf7c745b210848535b1582cc2db3f19470134fb0da1ae4b296` |
| 本地源码版本文件 | `D:\lib\xray\DCMTK\dcmtk-3.7.0\VERSION` = `3.7.0` |
| Git tag object | `refs/tags/DCMTK-3.7.0` = `dd841c3a858dfb20cfa8574a2a54a33d032b0de4` |
| Git peeled commit | `refs/tags/DCMTK-3.7.0^{}` = `ccfd10b84ff3c9a40b7b331698aedf06d421fc43` |

## 3. 依赖控制要求

- CMake configure 阶段不得隐式联网下载依赖。
- `third_party` 依赖必须由显式脚本或明确命令拉取。
- 所有依赖必须固定版本、tag 或 commit。
- 医疗软件发布基线必须记录依赖版本、许可证、用途和验证结果。
