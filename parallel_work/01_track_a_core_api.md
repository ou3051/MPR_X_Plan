# Track A 岗位施工说明：Core / Public API

## 1. 岗位目标

为 DICOM、MPR、Planning、DRR、Persistence 提供稳定公共数据接口和基础测试夹具。当前第一优先级是支持 Track B1 完成真实 DCMTK CT 导入。

## 2. 必读文件

- `parallel_work/00_common_contracts.md`
- `src/measurement/core/include/measurement/core/Geometry.h`
- `src/measurement/core/include/measurement/core/Result.h`
- `src/measurement/core/include/measurement/core/Volume.h`
- `src/measurement/core/include/measurement/core/Instrument.h`
- `src/measurement/core/include/measurement/core/Xray.h`
- `tests/unit/CoreCoordinateTests.cpp`
- `tests/unit/InstrumentTests.cpp`

不要求通读 PRD/SAD。接口背景不足时只查 `docs/PublicInterfaces_v0.1.md`。

## 3. 可修改范围

可修改：

- `src/measurement/core/**`
- `tests/unit/*Core*`
- `tests/unit/*Instrument*`
- `tests/unit` 中新增 core 测试
- `docs/PublicInterfaces_v0.1.md` 中与 public API 同步的片段

需先协调后修改：

- `src/measurement/dicom/**`
- `src/measurement/mpr/**`
- `src/measurement/drr/**`
- `src/measurement/persistence/**`
- CMake 依赖发现逻辑

禁止修改：

- `third_party/**`
- `build/**`
- 将 Qt、VTK、DCMTK、CUDA 头文件加入 core public headers

## 4. 实施方案

1. 梳理 Track B1 DICOM 导入缺失的 core 支撑类型。
2. 如当前 `IImageVolume` 不够表达真实 CT 体数据，可增加 core 内部/公共的密集 HU volume 实现，但不得引入 DCMTK 类型。
3. 补充 metadata 校验工具，至少覆盖 dimensions、spacing、方向向量、rescale 参数。
4. 固化错误码风格，保证 DICOM/MPR/DRR 能返回可测试错误。
5. 为新增接口补 GoogleTest，覆盖正常路径和错误路径。
6. 同步 `docs/PublicInterfaces_v0.1.md`，只写实际 public signatures，不写愿望接口。

## 5. 交付物

- 编译通过的 core public headers 与实现。
- 新增或更新的 core 单元测试。
- 若 public API 变化，更新后的 `docs/PublicInterfaces_v0.1.md`。
- 简短交付说明：新增类型、调用方、兼容性影响。

## 6. 验收标准

- `measurement_core` 不依赖 Qt、VTK、DCMTK、CUDA。
- `ctest --preset debug` 通过。
- core 相关新增测试通过。
- DICOM 导入负责人可以直接使用新增接口输出 `VolumeData`。
- 无未说明的跨模块行为改变。

## 7. 交接给谁

主要交接给 Track B1：DICOM 导入负责人。若变更 `ProjectionParams` 或 `DrrRenderSettings`，同步通知 Track D。
