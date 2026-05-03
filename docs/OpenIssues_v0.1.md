# 待确认问题追踪 v0.1

文档状态：草案
版本：v0.1
日期：2026-05-03

## 1. 追踪规则

- 所有待确认项必须有唯一 ID、负责人、状态、关闭标准和关闭证据。
- 状态取值：`OPEN`、`IN_PROGRESS`、`CLOSED`、`DEFERRED`。
- 无负责人或无关闭标准的问题不得进入 PRD/SAD 正文。
- 关闭问题时，必须补充证据链接、会议纪要、测试结果或 commit hash。

## 2. 当前设计待确认项

| ID | 问题 | 负责人 | 状态 | 关闭标准 | 关闭证据 |
| --- | --- | --- | --- | --- | --- |
| OI-001 | AP/LAT 的 patient axis 符号需要用 phantom 和医生习惯共同确认。 | Track D 验证负责人 + 临床顾问 | OPEN | 形成 AP/LAT 几何定义图、phantom 投影结果和临床确认记录。 | 待补充 |
| OI-002 | 目标 GPU 最低型号、显存和 CUDA runtime 版本。 | Track D DRR 负责人 | OPEN | 给出最低硬件配置、推荐配置和性能验收阈值。 | 待补充 |
| OI-003 | 单文件工程包是否需要加密、签名或完整性校验。 | Track C 工程包负责人 + QA/法规负责人 | OPEN | 完成风险评估，决定 v0.1 是否纳入完整性校验或签名。 | 待补充 |
| OI-004 | v0.1 是否需要报告导出。 | 产品负责人 | OPEN | 明确 v0.1 范围内/范围外，并同步 PRD 需求列表。 | 待补充 |

## 3. 文档审查整改记录

| ID | 审查问题 | 负责人 | 状态 | 关闭标准 | 关闭证据 |
| --- | --- | --- | --- | --- | --- |
| DOC-001 | SAD 详细设计与源码结构不一致，`qt_adapter` 是否存在需澄清。 | 架构师 | CLOSED | SAD 明确 v0.1 不设独立 `measurement_qt_adapter` target，Qt 代码归属 `src/app/`。 | 本次文档更新 |
| DOC-002 | SOUP 清单 DCMTK 3.7.0 版本来源需确认官方 release 并补 commit hash。 | Track A 依赖负责人 | CLOSED | SOUP 记录官方源码包 SHA256、tag object 和 peeled commit；拉取脚本固定 3.7.0 commit。 | 本次文档更新 |
| DOC-003 | PublicInterfaces 文档过于简略，缺方法签名。 | Track A Core/API 负责人 | CLOSED | 文档列出 core 与模块服务 public signatures。 | 本次文档更新 |
| DOC-004 | 代码规范常量命名范围模糊，枚举值是否适用未说明。 | 架构师 | CLOSED | 规范明确常量、枚举类型、枚举值、宏、CMake option 命名规则。 | 本次文档更新 |
| DOC-005 | 文档间工程保存术语与 Xray 参数模型不一致。 | 产品负责人 + 架构师 | CLOSED | PRD/SAD 统一为“工程包”，PRD 增加 `ProjectionParams` 字段表。 | 本次文档更新 |
| DOC-006 | 待确认问题缺追踪机制、负责人和关闭标准。 | PM/架构师 | CLOSED | 新增集中追踪表并迁移 PRD/SAD 待确认项。 | 本次文档更新 |
