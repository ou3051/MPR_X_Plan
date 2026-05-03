# 代码规范 v0.1

## 1. C++ 规则

- 使用 C++20。
- 命名空间统一为 `measurement`。
- 类型名使用 `PascalCase`。
- 函数和变量使用 `camelCase`。
- 常量使用 `kPascalCase`。
- 头文件使用 `.h`，实现文件使用 `.cpp`。
- 不在 core 头文件中包含 Qt、VTK、DCMTK、CUDA 头文件。

## 2. 错误处理

- 跨模块返回值使用 `Result<T>` 或 `Result<void>`。
- 不用异常跨模块传递业务错误。
- 错误必须包含 `code`、`message`、`detail` 和 `recoverable`。

## 3. 测试规则

- 核心算法必须可脱离 UI 测试。
- 新增公共接口必须增加 GoogleTest。
- 测试名称使用 `Module_Behavior_ExpectedResult` 风格。

## 4. CMake 规则

- 所有 target 必须调用 `measurement_apply_common_options`。
- 依赖发现集中在 `cmake/Dependencies.cmake`。
- 不允许在 configure 阶段联网下载依赖。
