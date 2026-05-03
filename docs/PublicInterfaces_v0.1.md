# 公共数据接口 v0.1

## 1. Core 接口

`measurement_core` 提供以下公共接口：

- `Vec3d`
- `Mat4d`
- `Size3i`
- `Result<T>`
- `ErrorInfo`
- `VolumeMetadata`
- `VolumeTransform`
- `VolumeData`
- `Instrument`
- `SurgicalPlan`
- `ProjectionParams`
- `XrayView`

## 2. 模块边界

- DICOM 模块输出 `VolumeData`，不暴露 DCMTK 类型。
- MPR 模块使用 `MprViewState`，不使用 VTK widget 作为状态来源。
- DRR 模块使用 `ProjectionParams`，CPU 与 CUDA 共享同一输入。
- Persistence 模块读写 `.mprproj` 单文件工程包。

## 3. 坐标约定

- 业务坐标统一为 DICOM Patient Coordinate。
- 单位统一为毫米。
- `VolumeTransform::voxelToPatient` 与 `patientToVoxel` 是坐标转换唯一入口。
