# 代码规范 v0.1

文档状态：草案
版本：v0.1
日期：2026-05-03

## 1. C++ 规则

- 使用 C++20。
- 命名空间统一为 `measurement`。
- 类型名使用 `PascalCase`，包括 class、struct、enum class、type alias。
- 函数、方法、普通变量和成员变量使用 `camelCase`。
- 头文件使用 `.h`，实现文件使用 `.cpp`。
- 不在 core 头文件中包含 Qt、VTK、DCMTK、CUDA 头文件。

### 1.1 常量与枚举命名

- 具名常量使用 `kPascalCase`，适用于 namespace/class/function scope 的 `constexpr`、`const`、`inline constexpr`，例如 `kDefaultDetectorWidth`。
- 枚举类型使用 `PascalCase`，例如 `XrayPreset`。
- 枚举值使用 `PascalCase`，例如 `XrayPreset::AP`、`XrayPreset::LAT`、`InstrumentType::GuidePin`。枚举值不使用 `k` 前缀。
- 预处理宏仅限编译开关和 include guard 等必要场景，使用 `ALL_CAPS`，例如 `MEASUREMENT_HAVE_DCMTK`。
- CMake cache option 使用项目统一前缀和 `ALL_CAPS`，例如 `MPR_ENABLE_CUDA_DRR`。

## 2. 错误处理

- 跨模块返回值使用 `Result<T>` 或 `Result<void>`。
- 不用异常跨模块传递业务错误。
- 错误必须包含 `code`、`message`、`detail` 和 `recoverable`。
- 错误码使用稳定的 `ALL_CAPS_WITH_UNDERSCORES` 字符串，避免把用户可读文案作为逻辑判断条件。

## 3. 测试规则

- 核心算法必须可脱离 UI 测试。
- 新增公共接口必须增加 GoogleTest。
- 测试名称使用 `Module_Behavior_ExpectedResult` 风格。
- 对坐标、DICOM tag/HU、器械几何、DRR、工程包序列化等审批关注功能，测试必须覆盖正常路径和至少一个错误路径。

## 4. CMake 规则

- 所有 target 必须调用 `measurement_apply_common_options`。
- 依赖发现集中在 `cmake/Dependencies.cmake`。
- 不允许在 configure 阶段联网下载依赖。
- 缺失依赖只允许通过显式脚本或明确命令拉取到 `third_party/`，并固定 tag/commit。
