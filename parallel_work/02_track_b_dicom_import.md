# Track B1 岗位施工说明：DCMTK DICOM 导入

## 1. 岗位目标

实现 `DicomVolumeLoader::loadFolder`，把单序列 CT DICOM folder 装载为 `VolumeData`，为 MPR、DRR 和工程包提供真实体数据入口。

这是当前下一步主实施任务。

## 2. 必读文件

- `parallel_work/00_common_contracts.md`
- `src/measurement/dicom/include/measurement/dicom/DicomVolumeLoader.h`
- `src/measurement/dicom/src/DicomVolumeLoader.cpp`
- `src/measurement/dicom/CMakeLists.txt`
- `src/measurement/core/include/measurement/core/Volume.h`
- `src/measurement/core/include/measurement/core/Result.h`
- `tests/CMakeLists.txt`

需要 core 支撑时，再读 Track A 的岗位说明，不需要通读 SAD。

## 3. 可修改范围

可修改：

- `src/measurement/dicom/**`
- `tests/unit` 中新增 DICOM loader 单元测试
- 必要时向 Track A 提出 core 接口需求

需先协调后修改：

- `src/measurement/core/**`
- `cmake/Dependencies.cmake`
- `CMakePresets.json`
- `docs/PublicInterfaces_v0.1.md`

禁止修改：

- `src/app/**`
- `src/measurement/mpr/**`
- `src/measurement/drr/**`
- `src/measurement/persistence/**`
- 将 DCMTK 类型暴露到 public API

## 4. 实施方案

1. 保留现有 folder 存在性校验。
2. 使用 DCMTK 扫描 folder 内 DICOM 文件，忽略非 DICOM 或返回可诊断错误，策略需写入测试。
3. 读取并校验 CT 所需 tag：
   - SOP Class / Modality
   - StudyInstanceUID
   - SeriesInstanceUID
   - ImagePositionPatient
   - ImageOrientationPatient
   - PixelSpacing
   - SliceThickness 或相邻 slice 距离
   - Rows / Columns
   - BitsAllocated / BitsStored / PixelRepresentation
   - RescaleSlope / RescaleIntercept
4. v0.1 不做多序列选择。发现多个不一致 `SeriesInstanceUID` 时返回 `DICOM_MULTI_SERIES_UNSUPPORTED`。
5. 按 patient position 沿 slice direction 排序，构建连续 CT stack。
6. 将像素数据转换为 int16 HU，填充 `IImageVolume` 实现。
7. 计算 `VolumeMetadata`、`VolumeTransform`、bounds、min/max HU、study/series UID、sourceFolder、dataHash。
8. 所有失败路径返回 `Result<VolumeData>::failure`，错误码稳定可测。
9. 添加 DICOM loader 单元测试。没有真实 DICOM 测试数据时，先补可运行的错误路径测试和最小 DCMTK 接口测试；真实样本由 QA 后续补入 `tests/data`。

## 5. 交付物

- 可工作的 `DicomVolumeLoader::loadFolder`。
- DICOM 错误码列表写在交付说明中。
- DICOM 单元测试。
- 如新增 core volume 存储类型，需由 Track A 合并或确认。

## 6. 验收标准

- 本机 DCMTK 3.7.0 Debug/Release 均可编译链接。
- 空路径、非目录、无 DICOM、非 CT、多序列、tag 缺失均有明确错误。
- 单序列 CT folder 可输出有效 `VolumeData`。
- HU 转换符合 slope/intercept。
- `makeVolumeTransform` 往返测试仍通过。
- `ctest --preset debug` 通过。

## 7. 交接给谁

完成后交接给 Track B2：MPR 负责人和 Track D：DRR 负责人。交接内容包括一个可复用的 CT folder、期望 dimensions/spacing/orientation、HU 范围和 dataHash。
