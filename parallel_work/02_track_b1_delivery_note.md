# Track B1 交付说明（2026-05-03）

## 1. 交付范围

本轮完成 `DicomVolumeLoader::loadFolder` 的 DCMTK 单序列 CT 导入，实现位置：

- `src/measurement/dicom/src/DicomVolumeLoader.cpp`
- `tests/unit/DicomVolumeLoaderTests.cpp`
- `tests/CMakeLists.txt`

无 public API 变更，无 DCMTK 类型泄漏到 `measurement_core`。

## 2. 已交付能力

- 仅扫描目标目录的直接子文件，不递归子目录。
- 仅接受单序列 CT DICOM folder，输出 `VolumeData`。
- 读取并校验：
  - `StudyInstanceUID`
  - `SeriesInstanceUID`
  - `ImagePositionPatient`
  - `ImageOrientationPatient`
  - `PixelSpacing`
  - `SliceThickness`
  - `Rows` / `Columns`
  - `BitsAllocated` / `BitsStored` / `PixelRepresentation`
  - `RescaleSlope` / `RescaleIntercept`
  - `PixelData`
- 按 slice direction 对切片排序。
- 按 DICOM rescale slope / intercept 转换 HU。
- 构建 `VolumeMetadata`、`VolumeTransform`、`IImageVolume`、`studyUid`、`seriesUid`、`sourceFolder`、`dataHash`。
- Debug / Release 单测已接入 `ctest`。

## 3. 稳定错误码

- `DICOM_FOLDER_NOT_FOUND`
- `DICOM_DEPENDENCY_MISSING`
- `DICOM_EMPTY_FOLDER`
- `DICOM_NO_CT_SERIES`
- `DICOM_MULTI_SERIES_UNSUPPORTED`
- `DICOM_MISSING_TAG`
- `DICOM_INCONSISTENT_GEOMETRY`
- `DICOM_IMAGE_BUILD_FAILED`

## 4. 调用方影响

- Track B2 可以直接消费 `VolumeData.image`、`VolumeMetadata`、`VolumeTransform` 做 MPR reslice。
- Track D 可以直接消费同一份 `VolumeData` 做 CPU / CUDA DRR，不需要自行读取 DICOM。
- 调用方需要处理 `Result<VolumeData>` 的失败路径，尤其是多序列、缺 tag、几何不一致和像素构建失败。
- 当前实现要求切片沿 slice axis 连续且间距一致；存在坏片、缺片或非均匀 spacing 的 folder 会返回 `DICOM_INCONSISTENT_GEOMETRY`。

## 5. 自动化验证

已通过：

```powershell
git diff --check
cmake --build --preset app-debug --target measurement_unit_tests
ctest --preset debug
cmake --build --preset app-release --target measurement_unit_tests
ctest --preset release
```

单测覆盖：

- 路径不存在
- 空目录 / 非 DICOM
- 非 CT DICOM
- 缺失必要 tag
- 多序列 CT
- 几何不一致
- HU 转换 / metadata / stable hash
- 无 DCMTK 构建分支

## 6. 真实样本体检结果

原始本机样本：

- `D:\code\dicom`

体检结论：

- 该目录不是可直接交接给下游的“可复用单序列 CT folder”。
- `1-034.dcm` 为截断坏片，`dcmdump` 报错：
  - `PixelData (7fe0,0010) larger (524288) than remaining bytes in file`
- 因该坏片导致原始目录同时存在：
  - 像素读取失败风险
  - 切片链路中断，整体 stack 不再满足连续等间距要求
- 直接对 `D:\code\dicom` 调用 loader 时，当前实现返回：
  - `DICOM_INCONSISTENT_GEOMETRY`

## 7. 可交接真实 CT 子集

本机已验证可导入的连续子集：

- `D:\code\dicom_track_b1_contiguous_035_332`

该子集来源：

- 从 `D:\code\dicom` 选取 `1-035.dcm` 到 `1-332.dcm`
- 共 298 张
- 间距连续，均为 1.5 mm

已实测 loader 输出基线：

- `dimensions`: `512 x 512 x 298`
- `spacingMm`: `(0.759766, 0.759766, 1.500000)`
- `originPatientMm`: `(-193.120117, -414.120117, -176.000000)`
- `rowDirectionPatient`: `(1.000000, 0.000000, 0.000000)`
- `columnDirectionPatient`: `(0.000000, 1.000000, 0.000000)`
- `sliceDirectionPatient`: `(0.000000, 0.000000, 1.000000)`
- `minHu`: `-1024`
- `maxHu`: `3071`
- `studyUid`: `1.3.6.1.4.1.14519.5.2.1.122301539605573587541285203103677780736`
- `seriesUid`: `1.3.6.1.4.1.14519.5.2.1.227862481115741677811684485953284571738`
- `dataHash`: `07259ed7a311a31c`

## 8. 交接给 Track B2

Track B2 可直接使用第 7 节的连续子集做 MPR 接线和基线截图验证，最少依赖如下：

- 读取 `VolumeData.metadata.dimensions`
- 读取 `VolumeData.metadata.spacingMm`
- 读取 `VolumeData.metadata.originPatientMm`
- 读取 `VolumeData.metadata.rowDirectionPatient`
- 读取 `VolumeData.metadata.columnDirectionPatient`
- 读取 `VolumeData.metadata.sliceDirectionPatient`
- 通过 `VolumeData.image->voxelHu(i, j, k)` 采样 HU
- 通过 `VolumeData.transform` 建立 voxel/patient 坐标换算

建议 Track B2 首轮验证项：

- Axial / Sagittal / Coronal 三视图方向是否与基线一致
- 初始切片位置是否落在 volume bounds 内
- 以中心点采样时不会出现轴翻转或 spacing 误用

## 9. 交接给 Track D

Track D 可直接使用第 7 节的连续子集做 DRR reference 验证：

- 体素尺寸和体积尺寸已固定，可作为投影几何输入
- HU 范围已固定，可用于确认窗宽窗位和线积分输入范围
- `dataHash=07259ed7a311a31c` 可作为当前样本版本标识

建议 Track D 首轮验证项：

- 先用该样本跑 CPU reference DRR
- 将 AP / LAT 结果与 phantom 验证链路分开记录
- 若后续改动 loader、HU 转换或 hash 算法，需要重新记录该样本的 `dataHash`

## 10. 手工复核入口

新增一个默认跳过的真实样本 smoke test，可在本机显式启用：

```powershell
$env:MEASUREMENT_REAL_DICOM_FOLDER='D:\code\dicom_track_b1_contiguous_035_332'
.\build\vs2022-x64-debug\tests\Debug\measurement_unit_tests.exe --gtest_filter=DicomVolumeLoaderTests.LoadFolder_ReportsRealDicomBaselineWhenRequested
```

该测试会打印当前样本的 dimensions、spacing、方向、HU 范围和 `dataHash`，用于交接后复核。
