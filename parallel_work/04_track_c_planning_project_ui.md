# Track C 岗位施工说明：Planning / Project / UI

## 1. 岗位目标

实现导针/椎弓根螺钉规划、VTK actor 映射、属性编辑边界和 `.mprproj` 单文件工程包保存加载。v0.1 不建立独立 `measurement_qt_adapter` target，Qt UI 代码放在 `src/app/`。

## 2. 必读文件

- `parallel_work/00_common_contracts.md`
- `src/measurement/core/include/measurement/core/Instrument.h`
- `src/measurement/planning/include/measurement/planning/InstrumentGeometry.h`
- `src/measurement/planning/src/InstrumentGeometry.cpp`
- `src/measurement/persistence/include/measurement/persistence/ProjectManifest.h`
- `src/measurement/persistence/src/ProjectManifest.cpp`
- `src/app/main.cpp`
- `tests/unit/InstrumentTests.cpp`
- `tests/unit/ProjectManifestTests.cpp`

## 3. 可修改范围

可修改：

- `src/measurement/planning/**`
- `src/measurement/persistence/**`
- `src/app/**`
- `tests/unit/*Instrument*`
- `tests/unit/*Project*`
- 后续新增 `tests/functional/**`

需先协调后修改：

- `src/measurement/core/**`
- `src/measurement/vtk_adapter/**`
- `CMakeLists.txt` 和 app CMake 结构

禁止修改：

- DICOM tag/HU 解析逻辑
- DRR ray casting 算法
- 新增独立 `measurement_qt_adapter` target

## 4. 实施方案

1. 保持 `SurgicalPlan` 是器械业务状态来源。
2. UI 属性编辑产生 `InstrumentPatch`，通过 `SurgicalPlan::updateInstrument` 修改。
3. 导针与椎弓根螺钉 mesh 由 `InstrumentGeometryBuilder` 生成，actor 只作为显示结果。
4. 锁定器械不得被 UI 修改；隐藏器械不参与显示，但仍保留在 plan。
5. `.mprproj` 作为单文件工程包，内部主入口 `manifest.json`。
6. manifest 至少保存：
   - schemaVersion
   - softwareVersion
   - DICOM sourceFolder/studyUid/seriesUid/dataHash
   - instruments
   - xrayViews 或当前 xrayView
   - MPR/3D view state
7. 保存采用临时文件加原子替换策略。
8. 加载时校验 schemaVersion、必要字段和引用 DICOM 一致性。

## 5. 交付物

- 器械创建、编辑、隐藏、锁定、删除逻辑。
- Mesh/actor 映射接口。
- `.mprproj` 保存加载实现。
- Project manifest 序列化/反序列化测试。
- 最小 UI 入口或可由 app 调用的控制器。

## 6. 验收标准

- 器械 endpoint、长度、直径、方向测试通过。
- 保存加载后器械参数、DICOM 引用和 Xray 参数一致。
- 工程包不复制原始 DICOM，除非后续需求变更。
- UI 不直接把 VTK actor transform 写回业务数据。
- `ctest --preset debug` 通过。

## 7. 交接给谁

交接给 Track D 做器械投影 overlay，交接给 QA 做创建/编辑/保存加载功能测试。
