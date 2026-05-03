# QA 岗位施工说明：测试 / 验证 / 追踪矩阵

## 1. 岗位目标

建立审批级工程需要的测试入口、功能测试、算法验证记录和需求追踪矩阵。QA 不直接改业务实现，主要固化验收方式。

## 2. 必读文件

- `parallel_work/00_common_contracts.md`
- `tests/CMakeLists.txt`
- `tests/unit/*.cpp`
- `docs/TestScope_v0.1.md`
- `docs/OpenIssues_v0.1.md`
- `docs/PRD_v0.1.md` 中第 5 章需求表

QA 可以读 PRD 需求表，但不需要读完整 SAD。

## 3. 可修改范围

可修改：

- `tests/**`
- `docs/TestScope_v0.1.md`
- `docs/OpenIssues_v0.1.md`
- 后续新增 `docs/TraceabilityMatrix_v0.1.md`
- 测试数据说明文件

需先协调后修改：

- `src/**`
- `CMakeLists.txt`
- `CMakePresets.json`

禁止修改：

- 为了让测试通过而降低实现错误处理标准。
- 把真实患者隐私数据提交到仓库。
- 将大型 DICOM 数据直接提交，除非团队明确批准。

## 4. 实施方案

1. 维护统一验收入口：`ctest --preset debug`。
2. 为每个功能建立至少一个可自动运行测试或手工功能测试脚本。
3. 建立需求追踪矩阵：
   - PRD requirement ID
   - SAD section 或 interface
   - test case
   - status
   - evidence
4. 建立 DICOM 测试数据策略：
   - 优先使用 synthetic/minimal DICOM。
   - 禁止提交含患者隐私的真实数据。
   - 大数据只记录路径、hash、来源和使用说明。
5. 建立 validation phantom：
   - sphere
   - cylinder
   - step-HU
6. 对 `docs/OpenIssues_v0.1.md` 中 OPEN 项追踪 owner 和关闭证据。
7. 每轮交付后记录构建配置、测试结果、失败项和责任轨道。

## 5. 交付物

- 可运行的 CTest 测试集合。
- 功能测试清单。
- 验证 phantom 测试。
- 需求追踪矩阵。
- 待确认问题状态更新。

## 6. 验收标准

- `ctest --preset debug` 可作为统一每日验收入口。
- 每个新增 public API 有对应单元测试。
- 每个 P0 功能需求至少有一个测试或明确的待补测试项。
- 每个 OPEN 问题有负责人和关闭标准。
- 测试数据不包含未脱敏患者信息。

## 7. 交接给谁

QA 向 PM/架构师交付测试状态和风险列表；向各 Track 反馈失败测试和缺失证据。
