# AA-baseline 失效诊断与后续研究建议

日期: 2026-06-21

## 核心结论

在当前 8-sensor / 4+4 formation / tiered packet-drop / partial FOV 场景下，AA-baseline 明显弱于 GA-baseline 的主因不是一个单独 bug，而是 AA 融合规则和当前 LMB 输出方式之间的结构性不匹配:

1. GA/KLA 用几何乘积压制不同传感器不共同支持的 spatial modes 和 false-alarm modes；AA 用线性池保留这些 modes，天然更保守，但在当前场景里会转化为更大的 inter-sensor localization disagreement。
2. 当前 LMB 输出用 MAP cardinality + 每个 Bernoulli 的 top Gaussian。AA 保留多峰 mixture 后，不同局部邻域可能选到不同 mode；这会把 AA 的“保留信息”变成“各节点输出不一致”。`maxGM=1` 后 AA 明显改善，说明 mixture 多峰保留是核心失败源之一。
3. target-wise PD/FI weights 消费端修复后只能小幅改善 cardinality dispersion，不能修复 localization disagreement；因此 AA 差距不是“没有用 target-wise 权重”这么简单。
4. 纯 AA 在这个场景下很难做到和 GA 相当；但把 AA 用在 existence/PHD branch、把 spatial branch 改成 KLA 单 Gaussian，并配合 no-stabilization 权重、适度 pruning 和更强 spatial structure prior 后，N10 已经在所有记录指标上同时优于 GA Balanced 和 GA final。N50 上该 hybrid 继续在 OSPA、cardinality、local E-OSPA、hOspa、local RMSE、local CardErr 上优于两个 GA mode，只在 consensus Loc 上以 `1.472837` 对 `1.462915` 小幅落后于 GA Balanced。也就是说，“AA 用好”的可行路线不是 pure AA，而是 tuned AA-existence + KLA-spatial hybrid；当前剩余问题集中在少数拓扑/丢包 seed 的 spatial disagreement。

本轮实验已经从 N1 screening 升级到 N10 gate、N50 gate 和 N50 design ablation。N50 结果足以证明 hybrid 方向优于 GA final 且整体优于 GA Balanced，但还没有达到“所有指标均严格优于 GA Balanced”的最强目标；ablation 进一步证明 spatial-KLA 是主要收益来源，FID-FIA existence refinement 在当前 hybrid 上应排除，consensus Loc 短板应作为后续 target-wise effective graph / regime-aware guard 的主问题。

## Question

为什么当前仿真里 AA-baseline 相比 GA-baseline 差这么多；仓库里的 AA 实现是否有消费端问题；修复后 AA 能否通过 1-trial 快速实验接近 GA，或者至少明确下一步应该如何把 AA 用好。

## Scope

包含范围:

- 当前 worktree `/Users/dex/Desktop/Code/Research/MULTISENSOR-LMB-FILTERS`
- 既有 Zotero AA Fusion 全文综述 `/Users/dex/Desktop/Code/Research/aa_fusion_review/AA_FUSION_REVIEW_CN.md`
- AA 消费端实现、AA regression test、AA/GA 快速诊断、N10 gate

排除范围:

- 不做新的 Zotero 全文重读；沿用 2026-06-12 已完成的全文综述。
- 不把 N10 当作最终统计主表；N50 作为下一阶段验证。
- 不把 1-trial 结果写成 paper-facing 显著性结论。

## Risk Tier

L2。该诊断会影响后续研究方向和代码路径选择，但当前只修改本地研究代码、生成内部实验报告，不直接形成投稿结论。

## Claims

| ID | Claim | Confidence | Evidence IDs | Caveats |
| --- | --- | --- | --- | --- |
| C1 | AA 消费端确实存在 target-wise weights 未被消费的问题，已修复并加 regression。 | High | E2, E3, E9 | 只覆盖当前 AA track-merging 路径。 |
| C2 | 当前同场景下 GA fixed 已明显优于 AA fixed，因此 AA/GA 差距不是动态权重单独造成。 | High | E6, E7 | N=1，定位性质。 |
| C3 | AA 动态权重主要改善 cardinality dispersion，但 localization disagreement 仍远高于 GA。 | High | E6, E7, E8 | N=1，且只覆盖当前 4+4 场景。 |
| C4 | target-wise PD/FI AA 不能解决主体差距，只小幅改善 cardinality，local tracking 还变差。 | Medium-High | E9 | 仅测试现有 PD/FI surrogate。 |
| C5 | `maxGM=1`/更强 pruning 是目前最有效 AA-side tuning，但仍不能接近 GA Balanced。 | Medium-High | E10, E11, E12 | 需要多 seed 验证是否稳定。 |
| C6 | 后续更可行的研究方向是 AA/GA hybrid、target-wise effective graph 和 mode-selection/fit，而不是继续纯 AA 权重调参。 | High | E1, E6-E13 | 已有 1-trial hybrid 直接支持该方向。 |
| C7 | AA-existence + KLA-spatial hybrid 在当前 1-trial 场景下已达到 GA Balanced 同级。 | Medium-High | E6, E13 | 仍需多 seed 验证稳定性。 |
| C8 | `existenceThreshold=0.05` 的 Balanced spatial-KLA AA 在当前 1-trial 场景下所有记录指标均优于 GA Balanced 和 GA final。 | Medium | E6, E14 | 只是一条 seed=2 快速复现，下一步需要 N10/N20。 |
| C9 | N10 上，`existenceThreshold=0.05` 的 Balanced spatial-KLA AA 仍优于两个 GA mode 的 OSPA、cardinality dispersion、local E-OSPA、local RMSE 和 local CardErr，但 consensus Loc 不如 GA Balanced。 | High | E15, E16, E17 | 不应上 N20/N50；需要继续压 localization disagreement。 |
| C10 | N1 threshold sweep 显示 `existenceThreshold=0.10` 比 0.05/0.06/0.08 更好，是下一轮 N10 localization-focused tuning 的当前候选。 | Medium | E14, E18, E19, E20 | 仍可能在多 trial 下过度 pruning，必须用 N10 复核。 |
| C11 | `existenceThreshold=0.10` 的 N10 结果继续改善 OSPA 和 local metrics，但 consensus Loc 仍高于 GA Balanced。 | High | E21, E22 | 证明单纯 threshold tuning 不够。 |
| C12 | Tuned spatial-KLA AA (`existenceThreshold=0.18`, `spatialDecouplingStrength=1.0`, `spatialStructureStrength=0.75`) 在 N10 上所有记录指标均优于 GA Balanced 和 GA final。 | High | E15, E23, E24, E25 | N10 已通过；N50 暴露少量 Loc 弱点。 |
| C13 | Tuned spatial-KLA AA 在 N50 上优于两个 GA mode 的 OSPA、cardinality、local E-OSPA、hOspa、local RMSE 和 local CardErr，但 consensus Loc 略弱于 GA Balanced。 | High | E29, E30, E31 | Loc 差距很小但方向明确，不能宣称全指标胜出。 |
| C14 | N50 design ablation 证明 spatial-KLA 是 AA-family 的主增益来源；pure AA spatial mixture 是失败源，FID-FIA existence refinement 在当前 hybrid 下反而破坏定位和 cardinality。 | High | E34, E35 | tuned spatial structure 只有小幅收益；Loc gap 仍需后续 guard。 |
| C15 | 强 target-wise existence-gated KLA spatial 会收缩 effective spatial graph，但在 seeds 12-16 上没有改善 Loc/OSPA。 | Medium-High | E36, E37 | 只验证 `aaKlaSpatialExistencePower=1`、`minScore=0`；不排除更温和的 regime-aware guard。 |

## 代码实现问题与修复

### 1. AA 原先没有消费 target-wise weights

`computeAdaptiveFusionWeights.m` 的 PD/FI direct baselines 会输出 `aaTargetWiseWeights`，但 AA 融合消费端只读取 scalar/branch weights。这样 target-wise 权重对 AA 实际不生效。

已修复: `aaLmbTrackMerging.m` 现在按 Bernoulli/object 解析 `aaTargetWiseWeights`，并优先于 branch/global weights。新增 regression test 覆盖 target-wise 权重优先级。

### 2. branch-decoupled AA 应明确为 heuristic

Bernoulli-AA 的严格形式应使用同一组权重同时进入 existence 和 spatial density。我们之前把 spatial/existence branch 权重解耦，这对工程调参有用，但不是严格 AA。

已修复: 新增 strict-AA 模式:

- `model.aaFusionWeightMode = 'strict'`
- 或 `model.adaptiveFusion.aaFusionWeightMode = 'strict'`
- 或 `model.adaptiveFusion.aaStrictWeights = true`

默认仍保留 branch-decoupled heuristic AA，便于比较。

### 3. 实验入口补充

`runAaBalancedCardinalityValidation.m` 新增:

- `aaStrictWeights` 控制项
- 可显式选择的 `PD target-wise AA` 和 `FI target-wise AA` arm
- 可显式选择的 `Balanced spatial-KLA AA` 和 `Cardinality spatial-KLA AA` hybrid arms
- 可显式选择的 `Tuned spatial-KLA AA` arm
- 第 8 个可选参数 `adaptiveFusionOverrides`，用于快速筛选 adaptiveFusion 字段而不改默认 arms

`runMultisensorFilters_formation_4plus4_TieredLinkAblation.m` 新增第 9 个可选参数 `scenarioOverrides`，用于把 GA 脚本临时跑在 AA 快速诊断同样的 `targetFormationLifeSpan=24` 场景上。默认调用不变。

## 实验证据

所有结果都使用同一 seed=2、同一 tiered pDrop realization:

`pDropBySensor = [0.5 0.2 0 0.1 0.1 0.5 0.1 0.1]`

### 同场景 GA vs AA

| Method | Fusion | OSPA disagreement | Loc disagreement | Card dispersion | Local E-OSPA | Local RMSE | Local CardErr |
| --- | --- | ---: | ---: | ---: | ---: | ---: | ---: |
| fixed weights | GA | 2.6971 | 2.1227 | 0.3425 | 3.1268 | 3.5892 | 0.8600 |
| +structure-aware decoupled KLA | GA | 1.7867 | 1.5052 | 0.0913 | 2.2477 | 4.3123 | 0.2263 |
| +FID-FIA existence refinement | GA | 1.8642 | 1.9163 | 0.0788 | 2.1697 | 4.6429 | 0.1462 |
| fixed | AA | 3.9250 | 5.9438 | 0.2175 | 3.5542 | 4.1170 | 0.4150 |
| Balanced | AA | 3.4035 | 4.1982 | 0.0963 | 3.6188 | 4.7958 | 0.1613 |
| Cardinality-critical | AA | 3.3621 | 4.1248 | 0.0950 | 3.6053 | 4.8836 | 0.1550 |
| Balanced spatial-KLA | AA hybrid | 1.7962 | 1.7062 | 0.0988 | 2.0852 | 3.0228 | 0.1613 |
| Cardinality spatial-KLA | AA hybrid | 1.7929 | 1.6445 | 0.0913 | 2.0828 | 3.2637 | 0.1513 |
| Balanced spatial-KLA, threshold 0.05 | AA hybrid | 1.7091 | 1.4980 | 0.0213 | 2.0423 | 4.1553 | 0.0713 |

Interpretation:

- GA fixed 已经显著优于 AA fixed，说明差距不是动态权重造成的。
- AA Balanced/Cardinality-critical 可以把 cardinality dispersion 拉到接近 GA Balanced，但 localization disagreement 仍是 GA 的约 2.7 倍。
- AA 动态权重改善 network cardinality，却让 local RMSE 变差。这符合 AA 保留多模式/保守线性池的代价。
- spatial-KLA hybrid 把 localization disagreement 从 4.12 降到 1.64，和 GA Balanced 的 1.51 同级；Cardinality spatial-KLA AA 的 OSPA 1.7929 几乎等于 GA Balanced 的 1.7867。
- 对 Balanced spatial-KLA AA 只把 pruning 阈值从 0.03 提到 0.05 后，Loc 进一步降到 1.4980，低于 GA Balanced 的 1.5052；同时 Card、local E-OSPA、local RMSE、local CardErr 也都优于两个 GA reference。

### strict-AA 消融

| Method | OSPA disagreement | Loc disagreement | Card dispersion | Local E-OSPA | Local RMSE | Local CardErr |
| --- | ---: | ---: | ---: | ---: | ---: | ---: |
| Balanced AA strict | 3.3963 | 4.1537 | 0.1025 | 3.6180 | 4.8671 | 0.1600 |
| Cardinality-critical AA strict | 3.3963 | 4.1537 | 0.1025 | 3.6180 | 4.8671 | 0.1600 |

Interpretation:

- strict-AA 只比默认 Balanced AA 小幅降低 OSPA/Loc，cardinality 还略差。
- strict 模式下 Cardinality-critical 与 Balanced 完全重合，说明 FID-FIA existence refinement 的收益依赖 branch-decoupled heuristic；它不是严格 Bernoulli-AA 的自然结果。

### target-wise AA 消融

| Method | OSPA disagreement | Loc disagreement | Card dispersion | Local E-OSPA | Local RMSE | Local CardErr |
| --- | ---: | ---: | ---: | ---: | ---: | ---: |
| fixed AA | 3.9250 | 5.9438 | 0.2175 | 3.5542 | 4.1170 | 0.4150 |
| PD target-wise AA | 3.7645 | 6.0347 | 0.1788 | 3.8982 | 4.5593 | 0.3913 |
| FI target-wise AA | 3.8417 | 5.9181 | 0.1888 | 3.9134 | 5.3046 | 0.2663 |

Interpretation:

- target-wise resolver 修复有效: PD/FI target-wise arms 的 cardinality dispersion 有小幅下降。
- 但 localization disagreement 基本没有改善，local E-OSPA/RMSE 反而变差。说明 target-wise 权重本身不能解决 AA 的多峰/模式选择问题。

### AA mixture/pruning 探针

| Method | maxGM | existenceThreshold | OSPA disagreement | Loc disagreement | Card dispersion | Local E-OSPA | Local RMSE | Local CardErr |
| --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| Balanced AA default | 3 | 0.03 | 3.4035 | 4.1982 | 0.0963 | 3.6188 | 4.7958 | 0.1613 |
| Balanced AA | 1 | 0.03 | 3.0495 | 3.4493 | 0.1163 | 3.2134 | 4.2022 | 0.1788 |
| Balanced AA | 1 | 0.08 | 2.9646 | 3.2687 | 0.1175 | 3.1953 | 3.9802 | 0.1900 |
| Cardinality-critical AA | 1 | 0.08 | 2.9595 | 3.3123 | 0.1113 | 3.1934 | 4.0042 | 0.1838 |

Interpretation:

- `maxGM=1` 是本轮最有效的 AA-side tuning，证明 AA 多峰保留和 MAP top-component 输出之间存在实际冲突。
- `existenceThreshold=0.08` 继续小幅改善 localization 和 local RMSE，但 cardinality error 变差。
- tuned AA 最好 OSPA 约 2.96，仍明显差于 GA Balanced 的 1.79。说明简单 pruning 不能把纯 AA 拉到 GA 水平。

### AA-existence + KLA-spatial hybrid

| Method | Spatial fusion | Existence fusion | OSPA disagreement | Loc disagreement | Card dispersion | Local E-OSPA | Local RMSE | Local CardErr |
| --- | --- | --- | ---: | ---: | ---: | ---: | ---: | ---: |
| Balanced spatial-KLA AA | KLA/GA single Gaussian | AA linear Bernoulli | 1.7962 | 1.7062 | 0.0988 | 2.0852 | 3.0228 | 0.1613 |
| Cardinality spatial-KLA AA | KLA/GA single Gaussian | AA + FID-FIA existence branch | 1.7929 | 1.6445 | 0.0913 | 2.0828 | 3.2637 | 0.1513 |
| Balanced spatial-KLA AA, threshold 0.05 | KLA/GA single Gaussian | AA linear Bernoulli | 1.7091 | 1.4980 | 0.0213 | 2.0423 | 4.1553 | 0.0713 |
| GA Balanced reference | KLA/GA single Gaussian | GA/KLA Bernoulli | 1.7867 | 1.5052 | 0.0913 | 2.2477 | 4.3123 | 0.2263 |
| GA final reference | KLA/GA single Gaussian | GA/KLA + FID-FIA existence | 1.8642 | 1.9163 | 0.0788 | 2.1697 | 4.6429 | 0.1462 |

Interpretation:

- 这是本轮第一个达到 GA 同级的 AA-family 方法。
- 与 GA Balanced 相比，Cardinality spatial-KLA AA 的 OSPA 只高 0.0062，cardinality dispersion 完全相同，local E-OSPA 和 local CardErr 还更好；loc disagreement 仍略高。
- no-stabilization Balanced spatial-KLA AA + `existenceThreshold=0.05` 是当前 N1 winner：它同时压过 GA Balanced 的 OSPA/Loc 和 GA final 的 Card/CardErr。
- 这支持更精确的理论结论: 当前场景里 AA 不应承担 spatial mode intersection；AA 更适合承担 existence/PHD 层面的 first-moment/cardinality 融合。

### Hybrid threshold sweep

| Method | Trials | OSPA disagreement | Loc disagreement | Card dispersion | Local E-OSPA | Local RMSE | Local CardErr |
| --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| Balanced spatial-KLA AA, threshold 0.05 | 1 | 1.7091 | 1.4980 | 0.0213 | 2.0423 | 4.1553 | 0.0713 |
| Balanced spatial-KLA AA, threshold 0.06 | 1 | 1.7056 | 1.4989 | 0.0200 | 2.0439 | 4.1585 | 0.0700 |
| Balanced spatial-KLA AA, threshold 0.08 | 1 | 1.7007 | 1.4855 | 0.0213 | 2.0314 | 4.1484 | 0.0688 |
| Balanced spatial-KLA AA, threshold 0.10 | 1 | 1.6904 | 1.4821 | 0.0188 | 2.0238 | 4.1439 | 0.0663 |
| Balanced spatial-KLA AA, threshold 0.18 | 1 | 1.6857 | 1.4759 | 0.0188 | 2.0237 | 4.1405 | 0.0663 |
| Tuned spatial-KLA AA, threshold 0.18 | 1 | 1.6826 | 1.4746 | 0.0188 | 2.0289 | 4.1450 | 0.0663 |

Interpretation:

- N1 上单纯 threshold tuning 在 `0.18` 附近达到平台，`0.20` 开始破坏 Loc，`0.25` 明显破坏 OSPA/cardinality。
- `spatialDecouplingStrength=1.0` 和 `spatialStructureStrength=0.75` 只带来小幅 N1 Loc 改善，但在 N10 上对高 Loc trials 有实际收益。
- Posterior structure consistency 和 `minimumTrajectoryLength` sweep 没有提供有效改善；前者变差，后者在本场景下基本无影响。

### N10 验证状态

| Method | Trials | OSPA disagreement | Loc disagreement | Card dispersion | Local E-OSPA | Local RMSE | Local CardErr |
| --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| GA Balanced reference | 10 | 1.7321 | 1.4681 | 0.0830 | 2.1973 | 4.2230 | 0.2120 |
| GA final reference | 10 | 1.8021 | 1.8254 | 0.0755 | 2.1331 | 4.5113 | 0.1495 |
| Balanced spatial-KLA AA, threshold 0.05 | 10 | 1.6726 | 1.5488 | 0.0275 | 2.0025 | 3.8082 | 0.0778 |
| Balanced spatial-KLA AA, threshold 0.10 | 10 | 1.6679 | 1.5019 | 0.0295 | 1.9894 | 3.7378 | 0.0773 |
| Tuned spatial-KLA AA, threshold 0.18 | 10 | 1.6631 | 1.4563 | 0.0350 | 1.9986 | 3.7166 | 0.0840 |

Interpretation:

- `threshold=0.05` 和 `0.10` 都证实 hybrid 不是 N1 偶然：OSPA、cardinality、local E-OSPA/RMSE/CardErr 均优于两个 GA reference。
- 旧失败项是 consensus Loc: `0.05` 为 `1.5488`，`0.10` 降到 `1.5019`，但仍高于 GA Balanced `1.4681`。
- Tuned spatial-KLA AA 把 consensus Loc 降到 `1.4563`，同时保留 OSPA、cardinality 和 local metrics 优势；这是当前第一个 N10 全指标胜出配置。

### N50 验证状态

N50 使用和 N10 相同的 seed schedule (`baseSeed=1`, seeds 2-51)、同一个 24-step diagnostic scenario，并在同一脚本中先跑 GA Balanced / GA final reference，再跑 Tuned spatial-KLA AA。

| Method | Trials | OSPA disagreement | Loc disagreement | Card dispersion | Local E-OSPA | hOspa | Local RMSE | Local CardErr |
| --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| GA Balanced reference | 50 | 1.761137 | 1.462915 | 0.084125 | 2.2133 | 0.4869 | 4.2284 | 0.2040 |
| GA final reference | 50 | 1.847668 | 1.958378 | 0.081275 | 2.1799 | 0.4895 | 4.6951 | 0.1510 |
| Tuned spatial-KLA AA | 50 | 1.682607 | 1.472837 | 0.035350 | 2.029641 | 0.4856 | 3.682880 | 0.088000 |

Interpretation:

- Tuned spatial-KLA AA 明显优于 GA final，并且在除 consensus Loc 外的全部记录指标上优于 GA Balanced。
- consensus Loc 的差距为 `+0.009922`，相对 GA Balanced 约 `+0.68%`。这不是主性能崩溃，但足以说明还不能写成“全指标严格胜出”。
- per-trial 诊断显示 Loc 弱点集中在 seeds 12-21 与 32-41 两个 block，尤其 seed 20、34、7、12、35。后续改进应针对这些 target-wise topology / pDrop realization，而不是再做大范围纯阈值 sweep。

### N50 Loc block-screen

针对 N50 中最差的 seeds 12-16，追加了 N5 block-screen。降低 existence threshold (`0.12`, `0.15`) 或削弱 spatial structure (`0.45`, `0.60`) 都不能改善 Loc；增强 spatial/existence structure 到 `0.90/1.00` 与 `0.20` 的收益也只有噪声级。这说明当前 Loc 短板不太像单参数未调好，更像少数 realization 下 spatial branch 的 effective graph 需要显式诊断或 guard。

### N50 design ablation

N50 ablation 固定同一 seed schedule、`existenceThreshold=0.18` 和 24-step diagnostic scenario，比较 pure AA、基础 spatial-KLA hybrid、FID-FIA existence hybrid 和 tuned spatial-KLA hybrid。

| Arm | OSPA disagreement | Loc disagreement | Card dispersion | Local E-OSPA | hOspa | Local RMSE | Local CardErr |
| --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| Balanced AA | 3.422478 | 4.237960 | 0.116775 | 3.641417 | 0.4833 | 4.396212 | 0.221925 |
| Balanced spatial-KLA AA | 1.686615 | 1.476160 | 0.035375 | 2.024428 | 0.4856 | 3.678611 | 0.087925 |
| Cardinality spatial-KLA AA | 2.329792 | 6.549674 | 0.209250 | 2.442527 | 0.4934 | 5.542318 | 0.314250 |
| Tuned spatial-KLA AA | 1.682607 | 1.472837 | 0.035350 | 2.029641 | 0.4856 | 3.682880 | 0.088000 |

Interpretation:

- `Balanced AA -> Balanced spatial-KLA AA` 是决定性跃迁: OSPA 降 `50.72%`，Loc disagreement 降 `65.17%`，Card dispersion 降 `69.71%`，且 50/50 trials 都改善。这证明 pure AA spatial mixture 是当前场景的主失败源，spatial-KLA 是当前 hybrid 的核心设计。
- `Cardinality spatial-KLA AA` 显著失败: Loc `6.549674`、Card `0.209250`，local RMSE `5.542318`。这说明 FID-FIA existence refinement 不能直接移植到 AA-existence + KLA-spatial hybrid。
- `Tuned spatial-KLA AA` 相对基础 spatial-KLA 的收益小但一致: OSPA 降 `0.004008`，Loc 降 `0.003323`，Card 降 `0.000025`；代价是 local E-OSPA/RMSE/CardErr 略高。因此 tuned 参数应描述为小幅 retuning，而不是新的主机制。
- N50 ablation 支持当前设计“合理”，但也确认当前 all-metric gate 尚未完全通过: consensus Loc 仍为 `1.472837`，略高于 GA Balanced `1.462915`。

### Target-wise effective graph diagnostic

为定位 N50 Loc gap，新增了默认关闭的 target-wise effective-weight diagnostics，并在 N50 failure block seeds 12-16 上测试了一个轻量 guard: existence-gated spatial-KLA。该 guard 在 AA/KLA hybrid 的 spatial KLA 前用每个 target 的 local existence `r_s^i` 调制 spatial weights，试图避免低 existence 但高 precision 的局部 posterior 拉动 spatial estimate。

N5 结果不支持该 guard 作为主配置:

| Arm | OSPA disagreement | Loc disagreement | Card dispersion | Local E-OSPA | hOspa | Local RMSE | Local CardErr |
| --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| Tuned spatial-KLA AA | 1.702915 | 1.529716 | 0.034250 | 2.032799 | 0.4863 | 3.588145 | 0.086250 |
| Existence-gated spatial-KLA AA | 1.708370 | 1.534009 | 0.033750 | 2.036767 | 0.4860 | 3.585358 | 0.085750 |

Paired reduction 为 OSPA `-0.005454`、Loc `-0.004293`，也就是平均变差；Card、RMSE、CardErr 只有噪声级小幅改善。诊断表显示该 guard 的确把 target spatial Neff 从 `3.937` 降到 `2.874`，branch L1 从 `0.0463` 提到 `0.2677`。因此剩余 Loc gap 不只是“低 existence sensor 没有从 spatial KLA 移除”；过强收缩 target-wise spatial effective graph 反而会伤害 spatial consensus。

## 理论解释

AA 文献的核心动机是 conservative/reliable fusion: 在未知互相关、漏检、异构滤波器和分布式网络下，AA 用线性池避免过度自信，并通过 PHD/LPHD consistency 保持 first-moment 意义上的鲁棒性。这个动机和我们的失败结果并不矛盾。

当前场景对 AA 不友好，原因在于:

1. 当前压力更像 false-alarm / clutter / partial-FOV mode selection 问题，而不是单纯 missed detection 问题。GA 的几何乘积会压制未被多个邻居共同支持的 modes；AA 会保留它们。
2. AA 的闭式结果常常是 mixture。对 Gaussian/LMB 来说，AA 保留的是 component union；但我们的状态输出最终只取 MAP cardinality 和每个 Bernoulli 的 top Gaussian。也就是说，AA 的优势需要一个好的 mixture fit / mode selection 才能体现在输出指标上。
3. Part V 的 FI target-wise 思路要求 target-wise association 基本可靠。我们的 distributed LMB 场景里有 partial FOV、局部邻域、通信丢包和 label/track 不确定性，FI/PD 权重只改变传感器权重，不能自动解决错误 mode 被保留下来的问题。
4. branch-decoupled AA 对 cardinality 有用，但它本质是 heuristic，不是严格 Bernoulli-AA。strict-AA 消融表明，严格使用同一组权重时 Cardinality-critical 的 existence refinement 不再产生额外收益。
5. hybrid 实验证明，把 spatial branch 换成 KLA 后 AA-family 方法立刻接近 GA。也就是说，AA-baseline 的主失败源不是 Bernoulli existence 线性平均，而是 spatial density 的线性 mixture 与当前 MAP/top-Gaussian 输出机制不匹配。
6. no-stabilization 权重和 existence pruning 是关键：zero floor/EMA 让 hybrid 能更快响应当前时刻质量差异，`existenceThreshold=0.18` 去掉低存在概率但会影响 pairwise matching 的 Bernoulli；`spatialDecouplingStrength=1.0` 与 `spatialStructureStrength=0.75` 则让 spatial branch 更依赖结构可靠的 dedicated spatial score。

因此，AA-baseline 差不是“AA 理论错了”，而是当前仿真场景和实现路径更奖励 GA/KLA 的 mode intersection / false-alarm suppression。AA 的优势应该放在 existence/PHD/cardinality 分支或 missed-detection dominated regime 中体现；spatial branch 在当前场景里需要 KLA、mode guard 或 variational fit。

## 怎样把 AA 在这个场景里用好

短期可用配置有两个层级。

第一层是 pure AA 的更公平 baseline:

```matlab
struct( ...
    'maximumNumberOfGmComponents', 1, ...
    'existenceThreshold', 0.08)
```

配合 Balanced AA 或 Cardinality-critical AA。这能把 Balanced AA 的 OSPA disagreement 从 3.4035 降到约 2.96，local E-OSPA 从 3.62 降到约 3.19。但它还不能接近 GA。

第二层是当前真正有效的 AA-family 方法:

```matlab
cfg = BalancedAAConfig;
cfg.aaSpatialFusionMode = 'kla';
```

这保留 AA 的 existence 线性平均，但 spatial branch 使用 KLA 单 Gaussian。当前 N10 winner 是 Tuned spatial-KLA AA，`existenceThreshold=0.18`，`spatialDecouplingStrength=1.0`，`spatialStructureStrength=0.75`，N10 OSPA `1.6631`，Loc `1.4563`，Card `0.0350`。

中期研究路线不建议继续纯调 AA 权重，而应改融合结构:

1. Spatial branch 用 GA/KLA 或 AA + mode-consensus guard；existence/PHD branch 保留 AA 的 missed-detection robustness。
2. 在 AA spatial mixture 进入 MAP 输出前做 target-wise mode selection 或 variational/moment fit，而不是只按 component weight 截断。
3. 记录每个 Bernoulli 的 effective fusion graph: 哪些 sensor 真正进入 spatial branch、existence branch、target-wise branch；用图连通性解释失败样本。
4. 构造 regime-aware hybrid: 当风险来自 false alarm / mode clutter 时偏 GA；当风险来自 missed detection / sparse visibility 时偏 AA。

如果目标是“AA 和 GA 相当”，更现实的 paper story 不是 pure AA beats GA，而是:

> communication-aware target-wise hybrid average fusion for distributed LMB: use AA where first-moment/missed-detection robustness matters, use GA/KLA where spatial mode intersection and clutter suppression dominate, and diagnose the switch through target-wise effective information graphs.

## Evidence Ledger

| ID | Type | Artifact | What it supports |
| --- | --- | --- | --- |
| E1 | literature review | `/Users/dex/Desktop/Code/Research/aa_fusion_review/AA_FUSION_REVIEW_CN.md` | AA 系列全文综述: conservative/reliable fusion、PHD/LPHD consistency、label matching、FI target-wise 权重边界 |
| E2 | code | `multisensorLmb/aaLmbTrackMerging.m` | AA target-wise resolver 与 strict-AA 消费端修复 |
| E3 | test | `test_aa_lmb_track_merging.m` | target-wise AA 和 strict-AA regression |
| E4 | code | `RUN/AA/runAaBalancedCardinalityValidation.m` | AA strict/target-wise/tuning 实验入口 |
| E5 | code | `RUN/GA/runMultisensorFilters_formation_4plus4_TieredLinkAblation.m` | GA 同场景 lifespan=24 对照入口 |
| E6 | experiment | `RUN/GA/GA_TIERED_LINK_ABLATION_N1_SEED1_20260621_021522.md` | 同 seed GA fixed/Balanced/final 结果 |
| E7 | experiment | `RUN/AA/AA_BALANCED_CARDINALITY_VALIDATION_N1_SEED1_20260621_015912.md` | AA fixed/Balanced/Cardinality-critical baseline |
| E8 | experiment | `RUN/AA/AA_BALANCED_CARDINALITY_VALIDATION_N1_SEED1_20260621_020444.md` | strict-AA 消融 |
| E9 | experiment | `RUN/AA/AA_BALANCED_CARDINALITY_VALIDATION_N1_SEED1_20260621_021602.md` | PD/FI target-wise AA 消融 |
| E10 | experiment | `RUN/AA/AA_BALANCED_CARDINALITY_VALIDATION_N1_SEED1_20260621_022111.md` | Balanced AA maxGM=1 |
| E11 | experiment | `RUN/AA/AA_BALANCED_CARDINALITY_VALIDATION_N1_SEED1_20260621_022248.md` | Balanced AA maxGM=1, existenceThreshold=0.08 |
| E12 | experiment | `RUN/AA/AA_BALANCED_CARDINALITY_VALIDATION_N1_SEED1_20260621_022405.md` | Cardinality-critical AA maxGM=1, existenceThreshold=0.08 |
| E13 | experiment | `RUN/AA/AA_BALANCED_CARDINALITY_VALIDATION_N1_SEED1_20260621_022944.md` | Balanced/Cardinality spatial-KLA AA hybrid 达到 GA 同级 |
| E14 | experiment | `RUN/AA/AA_BALANCED_CARDINALITY_VALIDATION_N1_SEED1_20260621_134006.md` | no-stabilization Balanced spatial-KLA AA, threshold=0.05, 当前 N1 winner |
| E15 | experiment | `RUN/GA/GA_TIERED_LINK_ABLATION_N10_SEED1_20260621_141128.md` | GA Balanced/final N10 reference |
| E16 | experiment | `RUN/AA/AA_BALANCED_CARDINALITY_VALIDATION_N10_SEED1_20260621_141129.md` | Balanced spatial-KLA AA threshold=0.05 N10 result |
| E17 | command log | `RUN/AA/AA_HYBRID_VS_GA_N10_20260621_134402.log` | N10 paired run command output and report paths |
| E18 | experiment | `RUN/AA/AA_BALANCED_CARDINALITY_VALIDATION_N1_SEED1_20260621_142136.md` | Balanced spatial-KLA AA threshold=0.06 N1 threshold sweep |
| E19 | experiment | `RUN/AA/AA_BALANCED_CARDINALITY_VALIDATION_N1_SEED1_20260621_142224.md` | Balanced spatial-KLA AA threshold=0.08 N1 threshold sweep |
| E20 | experiment | `RUN/AA/AA_BALANCED_CARDINALITY_VALIDATION_N1_SEED1_20260621_142312.md` | Balanced spatial-KLA AA threshold=0.10 N1 threshold sweep, current N10 candidate |
| E21 | experiment | `RUN/AA/AA_BALANCED_CARDINALITY_VALIDATION_N10_SEED1_20260621_142730.md` | Balanced spatial-KLA AA threshold=0.10 N10 result, Loc still fails |
| E22 | command log | `RUN/AA/AA_HYBRID_THR010_N10_20260621.log` | threshold=0.10 N10 command output and report path |
| E23 | command log | `RUN/AA/AA_HYBRID_THRESHOLD_SWEEP_N1_20260621_1436.log` | threshold=0.12/0.15/0.18/0.20/0.25 N1 sweep |
| E24 | command log | `RUN/AA/AA_HYBRID_SPATIAL_STRUCTURE_SWEEP_N1_20260621.log` | spatial structure sweep; best N1 candidate around 0.75 |
| E25 | experiment | `RUN/AA/AA_BALANCED_CARDINALITY_VALIDATION_N10_SEED1_20260621_150248.md` | Tuned spatial-KLA AA threshold=0.18 N10 all-metric winner |
| E26 | command log | `RUN/AA/AA_HYBRID_TUNED_THR18_SD100_SS75_N10_20260621.log` | tuned N10 command output and report path |
| E27 | command log | `RUN/AA/AA_HYBRID_POSTERIOR_CONSISTENCY_SWEEP_N1_20260621.log` | posterior consistency sweep, rejected |
| E28 | command log | `RUN/AA/AA_HYBRID_MIN_TRAJECTORY_SWEEP_N1_20260621.log` | minimumTrajectoryLength sweep, no effect |
| E29 | command log | `RUN/AA/AA_TUNED_HYBRID_VS_GA_N50_20260621_1516.log` | N50 paired GA vs Tuned spatial-KLA AA run |
| E30 | experiment | `RUN/GA/GA_TIERED_LINK_ABLATION_N50_SEED1_20260621_183039.md` | GA Balanced/final N50 reference |
| E31 | experiment | `RUN/AA/AA_BALANCED_CARDINALITY_VALIDATION_N50_SEED1_20260621_183039.md` | Tuned spatial-KLA AA N50 result |
| E32 | command log | `RUN/AA/AA_TUNED_BLOCK_SCREEN_N5_SEED11_20260621.log` | N50 failure-block threshold/spatial-structure screen |
| E33 | command log | `RUN/AA/AA_TUNED_STRUCTURE_EXISTENCE_BLOCK_SCREEN_N5_SEED11_20260621.log` | N50 failure-block structure/existence screen |
| E34 | experiment | `RUN/AA/AA_BALANCED_CARDINALITY_VALIDATION_N50_SEED1_20260621_215745.md` | N50 design ablation report |
| E35 | command log | `RUN/AA/AA_TUNED_DESIGN_ABLATION_N50_20260621_215745.log` | N50 design ablation command log |
| E36 | experiment | `RUN/AA/AA_BALANCED_CARDINALITY_VALIDATION_N5_SEED11_20260622_095821.md` | existence-gated spatial-KLA AA failure-block diagnostic |
| E37 | command log | `RUN/AA/AA_KLA_EXISTENCE_GATE_DIAGNOSTIC_N5_SEED11_20260622.log` | existence-gated spatial-KLA AA diagnostic command log |

## Verification Record

Independence status: self-check only. 本轮没有独立 sub-agent/human verifier；检查来自 regression test、Octave 1-trial run、`git diff --check` 和 evidence lint。

Commands run:

```bash
octave --quiet --eval "test_aa_lmb_track_merging"

octave --quiet --eval "addpath('RUN/GA'); [reportPath, summary] = runMultisensorFilters_formation_4plus4_TieredLinkAblation(1, 1, true, struct(), true, 'fidFiaExistenceRefinement', struct(), [1 3 4], struct('targetFormationLifeSpan', 24)); ..."

octave --quiet --eval "addpath('RUN/AA'); [reportPath, summary] = runAaBalancedCardinalityValidation(1, 1, true, ..., [1 5 6]); ..."

octave --quiet --eval "addpath('RUN/AA'); [reportPath, summary] = runAaBalancedCardinalityValidation(1, 1, true, struct('maximumNumberOfGmComponents', 1, ...), ..., [3]); ..."
```

Known limitation: many tuning numbers above are 1-trial or N5 diagnostics. The tuned N50 run and N50 design ablation are now the main evidence for the hybrid direction, but they still leave one small consensus Loc gap against GA Balanced. The first effective-graph guard tested here was negative, so it should be treated as diagnostic evidence rather than a new main method.

## Risk and Escalation

若把本轮结论误当成统计主结论，主要风险是过早放弃某些 AA regime 或错误宣称 AA 无法使用。升级到 paper-facing 之前需要至少:

- target/time failure attribution for the remaining N50 consensus Loc gap。
- 低 PD / missed-detection-dominated / partial visibility 更强的场景。
- regime-aware guard 或 switching，而不是只靠 local existence gate 收缩 spatial graph。

## Reproducibility

核心复现命令:

```bash
octave --quiet --eval "test_aa_lmb_track_merging"

octave --quiet --eval "addpath('RUN/GA'); [reportPath, summary] = runMultisensorFilters_formation_4plus4_TieredLinkAblation(1, 1, true, struct(), true, 'fidFiaExistenceRefinement', struct(), [1 3 4], struct('targetFormationLifeSpan', 24));"

octave --quiet --eval "addpath('RUN/AA'); [reportPath, summary] = runAaBalancedCardinalityValidation(1, 1, true, struct('saveMat', false, 'saveCheckpoints', false, 'progressEverySteps', 0), struct(), true, [1 5 6]);"

octave --quiet --eval "addpath('RUN/AA'); [reportPath, summary] = runAaBalancedCardinalityValidation(1, 1, true, struct('saveMat', false, 'saveCheckpoints', false, 'progressEverySteps', 0, 'maximumNumberOfGmComponents', 1, 'existenceThreshold', 0.08), struct(), true, [3]);"

octave --quiet --eval "addpath('RUN/AA'); [reportPath, summary] = runAaBalancedCardinalityValidation(1, 1, true, struct('saveMat', false, 'saveCheckpoints', false, 'progressEverySteps', 0, 'maximumNumberOfGmComponents', 1, 'existenceThreshold', 0.08), struct(), true, [4]);"

octave --quiet --eval "addpath('RUN/AA'); [reportPath, summary] = runAaBalancedCardinalityValidation(1, 1, true, struct('saveMat', false, 'saveCheckpoints', false, 'progressEverySteps', 0), struct(), true, [7 8]);"

octave --quiet --eval "addpath('RUN/AA'); [reportPath, summary] = runAaBalancedCardinalityValidation(1, 1, true, struct('saveMat', false, 'saveCheckpoints', false, 'progressEverySteps', 0, 'existenceThreshold', 0.05), struct(), true, [7]);"

octave --quiet --eval "addpath('RUN/AA'); [reportPath, summary] = runAaBalancedCardinalityValidation(10, 1, true, struct('saveMat', false, 'saveCheckpoints', false, 'progressEverySteps', 0, 'existenceThreshold', 0.05), struct(), true, [7]);"

octave --quiet RUN/AA/runAaKlaExistenceGateDiagnosticN5.m

octave --quiet --eval "addpath('RUN/AA'); [reportPath, summary] = runAaBalancedCardinalityValidation(10, 1, true, struct('saveMat', false, 'saveCheckpoints', false, 'progressEverySteps', 0, 'existenceThreshold', 0.18), struct(), true, [9]);"

sh RUN/AA/launchAaTunedHybridVsGaN50.sh

sh RUN/AA/launchAaTunedDesignAblationN50.sh
```

Validation:

```bash
git diff --check
python3 /Users/dex/.codex/skills/auto-research/scripts/evidence_lint.py docs/AA_BASELINE_FAILURE_DIAGNOSTIC_CN.md
```

## Open Issues

- 已有 target-wise spatial/existence effective weights、entropy、Neff 和 branch divergence 的基础日志化；还需要把这些诊断对齐到具体 target/time failure。
- FI target-wise AA 目前用的是轻量 surrogate，不等价于 Part V 的完整 CT-FI multi-rate formulation。
- 强 existence-gated spatial-KLA guard 已被 seeds 12-16 N5 negative result 排除为主配置；当前仍未实现 regime-aware switching。

## Recommendation

不要继续把“纯 AA 调到 GA 一样好”作为主目标。短期可以把 tuned AA (`maxGM=1`, `existenceThreshold=0.08`) 作为 pure-AA baseline 的更公平版本；真正可用的主线应转向 target-wise hybrid average fusion。当前 N50 结果和 N50 design ablation 证明 Tuned spatial-KLA AA 已经在主要指标和 local tracking 上显著超过 GA reference，且 spatial-KLA 是核心收益来源；但 consensus Loc 仍有小幅缺口。existence-gated spatial-KLA 的 N5 负结果说明，下一步不应简单收缩 spatial effective graph，而应做 target/time failure attribution 后再设计 regime-aware switching。
