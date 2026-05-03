# Track B2 岗位施工说明：MPR 三视图

## 1. 岗位目标

基于 `VolumeData` 和外部 `MprViewState` 实现 Axial、Sagittal、Coronal 三视图 reslice。核心方案固定为 `vtkImageReslice + 外部控制`。

## 2. 必读文件

- `parallel_work/00_common_contracts.md`
- `src/measurement/mpr/include/measurement/mpr/MprResliceEngine.h`
- `src/measurement/mpr/src/MprResliceEngine.cpp`
- `src/measurement/core/include/measurement/core/Volume.h`
- `src/measurement/core/include/measurement/core/Geometry.h`
- `src/measurement/vtk_adapter/include/measurement/vtk/VtkPhysicsAdapter.h`

真实 DICOM 导入完成前，只做状态、矩阵和参数测试，不阻塞 Track B1。

## 3. 可修改范围

可修改：

- `src/measurement/mpr/**`
- `src/measurement/vtk_adapter/**` 中与 MPR/VTK image bridge 直接相关的新适配代码
- `tests/unit` 中新增 MPR 参数测试

需先协调后修改：

- `src/measurement/core/**`
- `src/app/**`
- `tests/functional/**`

禁止修改：

- DICOM loader 的解析策略
- DRR engine 的采样策略
- 使用 `vtkResliceCursorWidget` 作为业务状态来源

## 4. 实施方案

1. 保持 `MprViewState` 是唯一 MPR 状态来源。
2. 使用 `VolumeMetadata` 的 row/column/slice direction 推导三正交平面。
3. 为每个 plane 构建 reslice axes：
   - Axial：row/column 作为平面轴，slice direction 为法向。
   - Sagittal：以 patient coordinate 约定构造稳定横纵轴。
   - Coronal：以 patient coordinate 约定构造稳定横纵轴。
4. 将 crosshair patient point 转换为 reslice origin。
5. 在 VTK 适配层接 `vtkImageData` 与 `vtkImageReslice`；domain/core 层不包含 VTK 类型。
6. 处理 window center/width、zoom、pan，但不要让 UI widget 保存真实状态。
7. 增加参数级单元测试：plane normal、reslice axes 正交性、crosshair origin、非法 volume 错误。

## 5. 交付物

- MPR reslice 参数构建逻辑。
- VTK-backed reslice 适配代码或清晰的 adapter stub。
- MPR 单元测试。
- 与 `src/app` UI 接入所需的最小接口说明。

## 6. 验收标准

- 不使用 `vtkResliceCursorWidget` 承载核心状态。
- 三视图 crosshair 共享同一个 patient coordinate。
- 空 volume 返回明确错误。
- 方向矩阵正交、单位化、可测试。
- `ctest --preset debug` 通过。

## 7. 交接给谁

交接给 Track C 的 UI 负责人，用于三视图显示和属性面板联动；同时向 QA 提供三视图同步功能测试入口。
