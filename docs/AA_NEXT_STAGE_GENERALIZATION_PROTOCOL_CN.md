# AA next-stage generalization protocol

最后更新: 2026-06-27 CST

## 目的

当前 TAES 稿件已经把 AA label-barycenter 方法固定在一个清晰边界内: active-output label/moment projection, not recursive LMB update。已有 evidence 覆盖 primary N50、held-out N50、harsh-loss N50、topology-ring N50、partial-FOV N50，并且 full-topology N50 ceiling 已完成并接入 scenario-family gate。full-topology 的作用是 zero-disagreement ceiling equivalence，不是额外增益 claim。

下一阶段如果继续补实验，不应围绕当前数据搜索 `H`、existence threshold、projection cutoff、barycenter weights、label rules 或 packet-loss 参数。更有价值的方向是检验方法机制在真正不同的困难来源下是否仍成立，或者暴露需要新方法设计的边界。

## 当前 submission 边界

本文件是 repository-level internal planning protocol: it records future risk-reduction plans, not current manuscript evidence. It is not a portal upload or source-bundle evidence artifact。The current TAES submission does not wait for these A/B/C extensions；当前稿件只依赖已经进入 `evidence_sources.json`、generated fragments、verifier/readiness gates 和 PDF/source-bundle checks 的证据。

The full-topology ceiling has completed and is now parsed only as an idealized full-neighborhood equivalence boundary；它通过 report、extractor、generated manifest、readiness gate、source-bundle freshness 全部检查后才进入 scenario-family evidence chain，并且不得写成 manuscript、cover letter、supplement 或 response-ready text 中的 gain claim。

## 当前代码支持状态

`RUN/AA/runAaBalancedCardinalityValidation.m` 当前第 9 个参数 `scenarioOverrides` 已支持:

- `scenarioLabel`
- `neighborMapMode`
- `sensorCommRange`
- `sensorFovEnabled`
- `sensorFovHalfAngleDeg`
- `sensorFovRange`
- `sensorMotionType`
- `sensorMotionProcessNoiseStd`
- `targetScenarioMode`
- `targetBirthStates`
- `targetFormationStaggeredBirths`
- `targetFormationBirthInterval`
- `targetFormationStartTime`
- `crossingWindow`
- `simulationLength`

这足够支撑 topology、FOV、idealized full-topology ceiling checks，以及 Candidate A 的 fixed target crossing smoke。它仍不足以把 target maneuver/crossing 当作 paper-grade evidence，因为 extractor/readiness gate、N50 fixed-run manifest、assignment-ambiguity instrumentation 和 source-bundle freshness gate 尚未建立。Covariance-consistency 与 recursive feedback gates 仍未支持。

2026-06-27 代码 checkpoint: `RUN/AA/runAaBalancedCardinalityValidation.m` 已新增 `targetScenarioMode='maneuver-crossing-assignment'` 的固定 10-target close-crossing birth states，并输出 `## Scenario Window Metrics`。N1 smoke report 为 `RUN/AA/AA_BALANCED_CARDINALITY_VALIDATION_N1_SEED71_20260627_162134.md`，仅用于验证代码路径和 crossing-window metric reporting，不作为当前 manuscript evidence。

## 固定原则

1. 先写协议，再跑实验。每个新场景必须有固定 scenario label、改变量、不变量、arms、metrics、interpretation rule 和 no-search rule。
2. 先做 N1/N5 structural smoke，再做 N50。Smoke 只检查代码路径、报告字段、机制分解是否可读；不能用于调参。
3. 若结果 mixed 或 negative，进入 limitation 或 new-method design，不回调当前参数。
4. 新场景只有在 report、extractor、generated manifest、readiness gate、source-bundle freshness 全部通过后，才能进入正文或 cover letter evidence chain。
5. 若只是 response-ready 或 future-work evidence，必须在文档中保持为 internal/protocol status，不能写成 manuscript claim。

## Candidate A: maneuver / crossing assignment stress

### 研究问题

当前方法的主要理论边界是 assignment stability。Maneuver 或 crossing 场景应检验: 当目标间距缩小、轨迹交叉或局部 posterior 偏置时，reference selection 和 Hungarian assignment 是否仍能保持正确 correspondence，或者是否需要 assignment-margin / covariance / track-age gates。

### 建议固定场景

| Field | Proposed fixed value |
| --- | --- |
| `scenarioLabel` | `maneuver-crossing-assignment` |
| 改变量 | target birth states / target velocity schedule 产生接近或交叉 |
| 不变量 | arms `[9 18 19]`, AA controls, packet-loss profile, topology, FOV |
| primary metric | local RMSE and E-OSPA around crossing windows |
| mechanism metric | full vs reference-only RMSE separation |
| boundary metric | assignment ambiguity windows, if instrumented |

### 代码门槛

- `runAaBalancedCardinalityValidation.m` 需要支持 fixed target scenario override，而不是只改 sensor/topology/FOV。
- Ground-truth generator 需要能记录 maneuver/crossing windows，或 report 需要输出 crossing-window metrics。
- Extractor/readiness gate 必须区分 whole-run metrics 和 crossing-window metrics，避免用平均值掩盖 assignment failure。

当前状态: 前两项已在 2026-06-27 N1 smoke 中通过；第三项尚未做，因此 Candidate A 还不能进入主文或 cover-letter evidence chain。

### 解释规则

- Full 在 crossing windows 仍优于 reference-only: 支持 assignment+barycenter mechanism 在更难 correspondence 下仍有空间收益。
- Reference-only 接近或优于 full: 说明 moment barycenter 在 assignment ambiguity 下可能错误合并，应转向 gated projection，而不是调 `H`。
- Full 与 tuned AA 都变差: 说明 tracking model / sensor support 是主瓶颈，不能把结果写成 label-barycenter failure alone。

## Candidate B: covariance-consistency / reliability-weighted barycenter

### 研究问题

当前 moment barycenter 是 first-two-moment least-squares representative。它证明 PSD 和 matched Gaussian mixture moments，但不证明 covariance consistency 或 reliability optimality。Covariance-consistency 验证应检验: 当不同 sensors 的 covariance quality 明显不同时，equal barycenter 是否过度信任低质量 posterior。

### 建议固定场景

| Field | Proposed fixed value |
| --- | --- |
| `scenarioLabel` | `covariance-mismatch-reliability` |
| 改变量 | sensor-specific measurement noise / covariance inflation / detection quality |
| 不变量 | topology, packet-loss profile, target formation, arms `[9 18 19]` |
| primary metric | RMSE, E-OSPA, consistency proxy if available |
| mechanism metric | equal full vs reference-only vs future reliability-weighted variant |
| boundary metric | covariance spread or normalized innovation / consistency statistic |

### 代码门槛

- 明确 covariance mismatch 来自 measurement model、local posterior covariance inflation，还是 fusion-time reliability weights；三者不能混在一起。
- 若新增 reliability-weighted barycenter，它必须作为新 method arm，而不是替换当前 full method 后继续叫同一个方法。
- 需要新增 regression test，保证 existing equal-barycenter arm 不被悄悄改成 tuned reliability arm。

### 解释规则

- Equal barycenter 稳定: 当前 simple projection 在 covariance mismatch 下仍可用，但仍不等于 covariance-consistency guarantee。
- Equal barycenter 变差而 reliability-weighted variant 改善: 这是下一篇或 extension 的方法贡献，不能回写为当前 TAES 主 claim。
- Consistency proxy mixed: 写作上应区分 tracking accuracy 和 uncertainty calibration。

## Candidate C: recursive-online guarded projection

### 研究问题

当前 TAES 方法是 output-level active-track projection。Recursive-online 验证应检验: 如果 projection 反馈进下一步 LMB filtering，是否会在 births/deaths/crossings 下造成错误 label persistence 或 track merge。

### 建议固定场景

| Field | Proposed fixed value |
| --- | --- |
| `scenarioLabel` | `recursive-guarded-projection` |
| 改变量 | projection is fed back into the next filtering step |
| 不变量 | baseline sensors, packet-loss profile, target formation unless paired with Candidate A |
| required arms | upstream AA output-only, recursive projection without guard, recursive projection with fixed guard |
| primary metric | E-OSPA, RMSE, CardErr over time |
| safety metric | wrong-merge / lost-track / label-switch count if instrumented |

### 代码门槛

- Projection gate 必须固定在方法层面，例如 assignment margin、track age、covariance overlap 或 reliability evidence；不能按当前数据搜索阈值。
- Recursive no-guard arm 是必要 failure-control，不应只展示 guarded method。
- 需要长期 state/log instrumentation，否则无法证明错误 merge 是否被 guard 防住。

### 解释规则

- Guarded recursive method improves while no-guard fails: 这是强方法贡献，但属于 next method, not current output-level TAES claim。
- Both recursive arms fail: 当前 paper 的 output-level boundary 是正确的，future work 应聚焦 lifecycle guard。
- Both recursive arms work: 仍需 crossing/maneuver 和 covariance mismatch 才能扩展 claim。

## 执行顺序建议

1. 做 Candidate A 的 N1/N5 smoke，因为它直接检验本文理论中最关键的 assignment-stability boundary。
2. 若 Candidate A 显示 assignment ambiguity 明显，优先设计 guarded projection，而不是 covariance-weighted barycenter。
3. 若 Candidate A 稳定，再做 Candidate B，判断 equal moment barycenter 是否需要 reliability extension。
4. Candidate C 只能在 A/B 的边界明确后启动；它更像下一篇方法，而不是当前 TAES 稿件的必要补丁。

## Paper-facing 使用方式

当前 TAES submission 不应等待 A/B/C 全部完成。它们是 reviewer-risk reduction 或下一阶段 research design:

- Full-topology ceiling 已完成；当前稿件只能把它作为 zero-disagreement equivalence boundary，不能把相同 local metrics 写成方法增益。
- Candidate A/B 的 smoke 结果不应进入主文，只能进入 internal progress 或 response planning。
- Candidate A/B 的 N50 若完成且通过 parser/readiness gate，可视页数进入 supplement or response-ready evidence。
- Candidate C 即使成功，也应作为 method extension 或 follow-up paper，而不是静默扩展当前 output-level projection claim。
