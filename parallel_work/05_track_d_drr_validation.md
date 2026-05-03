# Track D 岗位施工说明：DRR / CUDA / Validation

## 1. 岗位目标

建立可验证的 CPU reference DRR，并推进 CUDA 实时 DRR。所有 DRR 几何输入统一使用 `ProjectionParams`，真实体数据来自 `VolumeData`。

## 2. 必读文件

- `parallel_work/00_common_contracts.md`
- `src/measurement/drr/include/measurement/drr/CpuDrrEngine.h`
- `src/measurement/drr/src/CpuDrrEngine.cpp`
- `src/measurement/drr/include/measurement/drr/CudaDrrEngine.h`
- `src/measurement/drr/src/CudaDrrEngine.cpp`
- `src/measurement/core/include/measurement/core/Xray.h`
- `src/measurement/core/include/measurement/core/Volume.h`
- `src/measurement/vtk_adapter/include/measurement/vtk/VtkPhysicsAdapter.h`
- `tests/unit/CpuDrrTests.cpp`

## 3. 可修改范围

可修改：

- `src/measurement/drr/**`
- `tests/unit/*Drr*`
- `tests/validation/**`
- 必要的 CUDA source/CMake 文件

需先协调后修改：

- `src/measurement/core/include/measurement/core/Xray.h`
- `src/measurement/vtk_adapter/**`
- `cmake/Dependencies.cmake`

禁止修改：

- DICOM loader 的读取策略
- MPR state 模型
- Project manifest schema，除非要保存 DRR 设置并已和 Track C 协调
- DRR 输出像素加水印

## 4. 实施方案

1. 先完善 CPU reference，不先写 CUDA kernel。
2. CPU DRR 必须确定性输出，支持 phantom 验证。
3. 明确 HU 到线积分/衰减的 v0.1 简化模型，写入测试说明。
4. 使用 `ProjectionParams` 计算 ray source、detector pixel 和采样方向。
5. 处理 volume bounds、stepMm、windowCenter/windowWidth、display image。
6. 添加 sphere/cylinder/step-HU phantom。
7. CUDA 路径与 CPU 路径共享输入参数，差异只在执行后端。
8. CUDA 输出与 CPU reference 做误差阈值比较。
9. Xray UI 实时预览可以降分辨率，但不得改变核心参数定义。

## 5. 交付物

- CPU reference DRR 完整测试。
- CUDA DRR engine stub 或 kernel 实现。
- phantom 数据构建工具或测试夹具。
- CPU/CUDA 差异测试。
- 性能记录：分辨率、硬件、平均耗时、最大耗时。

## 6. 验收标准

- 空 volume、非法 settings、非法 projection 均返回明确错误。
- sphere/cylinder/step-HU phantom 结果符合解析预期或固定 golden。
- CPU reference 结果确定性可复现。
- CUDA 可用时 GPU/CPU 差异在设定阈值内。
- `ctest --preset debug` 通过；涉及 CUDA 时 Release 也要通过。

## 7. 交接给谁

交接给 Track C 用于 Xray viewer 和导出，交接给 QA 建立算法验证报告。
