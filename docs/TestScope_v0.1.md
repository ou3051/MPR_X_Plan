# 测试范围 v0.1

## 1. 单元测试

- 坐标矩阵构建和往返。
- DICOM tag 与 HU 转换。
- 器械长度、直径、方向和端点。
- MPR reslice 参数。
- CPU DRR 几何与输出。
- 工程包 manifest 序列化。

## 2. 功能测试

- 导入单序列 CT DICOM。
- 三视图 MPR 十字线同步。
- 创建和编辑导针/椎弓根螺钉。
- 保存并加载 `.mprproj`。
- 生成 AP/LAT DRR。

## 3. 算法验证

- sphere phantom。
- cylinder phantom。
- step-HU phantom。
- 已知点、线、圆柱投影误差。
- CPU/CUDA DRR 差异。

## 4. 构建验证

- VS2022 Debug/Release 配置生成 `.sln`。
- `cmake --build` 通过 VS/MSBuild 编译。
- `ctest --preset debug` 通过。
- 系统依赖和 `third_party` fallback 各验证一次。
