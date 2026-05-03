# 公共施工约定

## 1. 技术基线

- 平台：Windows x64。
- 生成器：Visual Studio 17 2022。
- 构建系统：CMake Presets。
- 语言标准：C++20。
- UI/渲染：Qt6 + VTK9。
- DICOM：DCMTK 3.7.0，本机路径 `D:/lib/xray/DCMTK`。
- DRR：CPU reference + CUDA realtime path。
- 测试：GoogleTest + CTest。

标准命令：

```powershell
cmake --preset vs2022-x64-debug
cmake --build --preset app-debug
ctest --preset debug
```

Release 验收命令：

```powershell
cmake --preset vs2022-x64-release
cmake --build --preset app-release
ctest --preset release
```

## 2. 最小接口约定

公共类型只来自 `measurement_core`：

- `Vec3d`, `Mat4d`, `Size3i`
- `Result<T>`, `ErrorInfo`
- `VolumeMetadata`, `VolumeTransform`, `VolumeData`, `IImageVolume`
- `Instrument`, `InstrumentPatch`, `SurgicalPlan`
- `ProjectionParams`, `XrayView`, `DrrRenderSettings`, `DrrImage`

公共 header 位置：

- `src/measurement/core/include/measurement/core/Geometry.h`
- `src/measurement/core/include/measurement/core/Result.h`
- `src/measurement/core/include/measurement/core/Volume.h`
- `src/measurement/core/include/measurement/core/Instrument.h`
- `src/measurement/core/include/measurement/core/Xray.h`

硬性边界：

- `measurement_core` 禁止依赖 Qt、VTK、DCMTK、CUDA。
- `measurement_dicom` 可以依赖 DCMTK，但 public API 不暴露 DCMTK 类型。
- `measurement_mpr` 的状态来源是 `MprViewState`，不使用 `vtkResliceCursorWidget` 保存核心状态。
- `measurement_drr` 只接受 `VolumeData`、`ProjectionParams`、`DrrRenderSettings`。
- `measurement_vtk_adapter` 负责 VTK world 与 patient/physics 坐标转换。
- `measurement_persistence` 负责 `.mprproj` 单文件工程包。
- v0.1 不建立独立 `measurement_qt_adapter` target，Qt UI 代码归属 `src/app/`。

## 3. 数据阅读约定

DICOM 输入：

- v0.1 只支持 CT DICOM folder。
- 不做患者病历系统。
- 不做多序列选择 UI。
- 合法输入按单序列 CT stack 处理；发现多个不一致 `SeriesInstanceUID` 时应返回明确错误，而不是静默混合。
- HU 转换使用 DICOM rescale slope/intercept。
- 输出统一为 `VolumeData`，坐标统一为 DICOM patient coordinate，单位 mm。

MPR 数据：

- 三视图状态来自 `MprViewState`。
- Crosshair 使用 patient coordinate。
- Axial/Sagittal/Coronal 的方向由 `VolumeMetadata` 的 row/column/slice direction 推导。

器械数据：

- 导针和椎弓根螺钉均为参数化自绘制几何。
- 器械由 `entryPointPatientMm`、`directionPatientUnit`、`lengthMm`、`diameterMm` 定义。
- 器械业务数据只保存在 `SurgicalPlan`，VTK actor 不作为事实来源。

X 射线数据：

- DRR 几何统一使用 `ProjectionParams`。
- `sourcePosPatientMm`、`detectorCenterPatientMm`、`detectorUPatientUnit`、`detectorVPatientUnit` 均在 patient coordinate 中表达。
- DRR 输出图像不加水印。
- 仿真属性通过 UI、文件名、metadata 或工程包 manifest 标识。

工程包数据：

- 扩展名：`.mprproj`。
- 工程包是单文件容器。
- 包内主入口：`manifest.json`。
- v0.1 默认不复制原始 DICOM 数据，只保存引用路径、UID 和 data hash。

## 4. 命名与测试约定

- 类型名：`PascalCase`。
- 函数、变量、成员变量：`camelCase`。
- 具名常量：`kPascalCase`。
- 枚举值：`PascalCase`，不加 `k` 前缀。
- 错误码：`ALL_CAPS_WITH_UNDERSCORES`。
- 测试名：`Module_Behavior_ExpectedResult`。

新增或修改公共接口时必须同时：

- 更新对应 header。
- 更新或新增 GoogleTest。
- 在交付说明中写明调用方影响。

## 5. 交付前检查

最低检查：

```powershell
git diff --check
cmake --build --preset app-debug
ctest --preset debug
```

涉及依赖、CMake、DCMTK、CUDA 或 Release 行为时追加：

```powershell
cmake --fresh --preset vs2022-x64-release
cmake --build --preset app-release
ctest --preset release
```
