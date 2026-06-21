# AA-baseline 失效诊断与后续研究建议

日期: 2026-06-21

## 核心结论

在当前 8-sensor / 4+4 formation / tiered packet-drop / partial FOV 场景下，AA-baseline 明显弱于 GA-baseline 的主因不是一个单独 bug，而是 AA 融合规则和当前 LMB 输出方式之间的结构性不匹配:

1. GA/KLA 用几何乘积压制不同传感器不共同支持的 spatial modes 和 false-alarm modes；AA 用线性池保留这些 modes，天然更保守，但在当前场景里会转化为更大的 inter-sensor localization disagreement。
2. 当前 LMB 输出用 MAP cardinality + 每个 Bernoulli 的 top Gaussian。AA 保留多峰 mixture 后，不同局部邻域可能选到不同 mode；这会把 AA 的“保留信息”变成“各节点输出不一致”。`maxGM=1` 后 AA 明显改善，说明 mixture 多峰保留是核心失败源之一。
3. target-wise PD/FI weights 消费端修复后只能小幅改善 cardinality dispersion，不能修复 localization disagreement；因此 AA 差距不是“没有用 target-wise 权重”这么简单。
4. 纯 AA 在这个场景下很难做到和 GA 相当；但把 AA 用在 existence/PHD branch、把 spatial branch 改成 KLA 单 Gaussian 后，1-trial 结果已经和 GA Balanced 基本相当。也就是说，“AA 用好”的可行路线不是 pure AA，而是 AA-existence + KLA-spatial hybrid。

本轮全部实验都是 N=1、seed=2 的快速复现，作用是定位失败机理，不是统计显著性结论。

## Question

为什么当前仿真里 AA-baseline 相比 GA-baseline 差这么多；仓库里的 AA 实现是否有消费端问题；修复后 AA 能否通过 1-trial 快速实验接近 GA，或者至少明确下一步应该如何把 AA 用好。

## Scope

包含范围:

- 当前 worktree `/Users/dex/Desktop/Code/Research/MULTISENSOR-LMB-FILTERS`
- 既有 Zotero AA Fusion 全文综述 `/Users/dex/Desktop/Code/Research/aa_fusion_review/AA_FUSION_REVIEW_CN.md`
- AA 消费端实现、AA regression test、AA/GA 1-trial 快速诊断

排除范围:

- 不做新的 Zotero 全文重读；沿用 2026-06-12 已完成的全文综述。
- 不做 N>=10/50 的统计主表；本轮仅做快速机理定位。
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

Interpretation:

- GA fixed 已经显著优于 AA fixed，说明差距不是动态权重造成的。
- AA Balanced/Cardinality-critical 可以把 cardinality dispersion 拉到接近 GA Balanced，但 localization disagreement 仍是 GA 的约 2.7 倍。
- AA 动态权重改善 network cardinality，却让 local RMSE 变差。这符合 AA 保留多模式/保守线性池的代价。
- spatial-KLA hybrid 把 localization disagreement 从 4.12 降到 1.64，和 GA Balanced 的 1.51 同级；Cardinality spatial-KLA AA 的 OSPA 1.7929 几乎等于 GA Balanced 的 1.7867。

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
| GA Balanced reference | KLA/GA single Gaussian | GA/KLA Bernoulli | 1.7867 | 1.5052 | 0.0913 | 2.2477 | 4.3123 | 0.2263 |

Interpretation:

- 这是本轮第一个达到 GA 同级的 AA-family 方法。
- 与 GA Balanced 相比，Cardinality spatial-KLA AA 的 OSPA 只高 0.0062，cardinality dispersion 完全相同，local E-OSPA 和 local CardErr 还更好；loc disagreement 仍略高。
- 这支持更精确的理论结论: 当前场景里 AA 不应承担 spatial mode intersection；AA 更适合承担 existence/PHD 层面的 first-moment/cardinality 融合。

## 理论解释

AA 文献的核心动机是 conservative/reliable fusion: 在未知互相关、漏检、异构滤波器和分布式网络下，AA 用线性池避免过度自信，并通过 PHD/LPHD consistency 保持 first-moment 意义上的鲁棒性。这个动机和我们的失败结果并不矛盾。

当前场景对 AA 不友好，原因在于:

1. 当前压力更像 false-alarm / clutter / partial-FOV mode selection 问题，而不是单纯 missed detection 问题。GA 的几何乘积会压制未被多个邻居共同支持的 modes；AA 会保留它们。
2. AA 的闭式结果常常是 mixture。对 Gaussian/LMB 来说，AA 保留的是 component union；但我们的状态输出最终只取 MAP cardinality 和每个 Bernoulli 的 top Gaussian。也就是说，AA 的优势需要一个好的 mixture fit / mode selection 才能体现在输出指标上。
3. Part V 的 FI target-wise 思路要求 target-wise association 基本可靠。我们的 distributed LMB 场景里有 partial FOV、局部邻域、通信丢包和 label/track 不确定性，FI/PD 权重只改变传感器权重，不能自动解决错误 mode 被保留下来的问题。
4. branch-decoupled AA 对 cardinality 有用，但它本质是 heuristic，不是严格 Bernoulli-AA。strict-AA 消融表明，严格使用同一组权重时 Cardinality-critical 的 existence refinement 不再产生额外收益。
5. hybrid 实验证明，把 spatial branch 换成 KLA 后 AA-family 方法立刻接近 GA。也就是说，AA-baseline 的主失败源不是 Bernoulli existence 线性平均，而是 spatial density 的线性 mixture 与当前 MAP/top-Gaussian 输出机制不匹配。

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

这保留 AA 的 existence 线性平均，但 spatial branch 使用 KLA 单 Gaussian。1-trial 中 Cardinality spatial-KLA AA 达到 OSPA `1.7929`，基本等于 GA Balanced `1.7867`。

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

## Verification Record

Independence status: self-check only. 本轮没有独立 sub-agent/human verifier；检查来自 regression test、Octave 1-trial run、`git diff --check` 和 evidence lint。

Commands run:

```bash
octave --quiet --eval "test_aa_lmb_track_merging"

octave --quiet --eval "addpath('RUN/GA'); [reportPath, summary] = runMultisensorFilters_formation_4plus4_TieredLinkAblation(1, 1, true, struct(), true, 'fidFiaExistenceRefinement', struct(), [1 3 4], struct('targetFormationLifeSpan', 24)); ..."

octave --quiet --eval "addpath('RUN/AA'); [reportPath, summary] = runAaBalancedCardinalityValidation(1, 1, true, ..., [1 5 6]); ..."

octave --quiet --eval "addpath('RUN/AA'); [reportPath, summary] = runAaBalancedCardinalityValidation(1, 1, true, struct('maximumNumberOfGmComponents', 1, ...), ..., [3]); ..."
```

Known limitation: all experimental numbers above are 1-trial diagnostics. They are enough to identify failure modes and screen bad directions, but not enough for paper claims.

## Risk and Escalation

若把本轮结论误当成统计主结论，主要风险是过早放弃某些 AA regime 或错误宣称 AA 无法使用。升级到 paper-facing 之前需要至少:

- N>=10 的 tuned AA vs GA paired validation。
- 低 PD / missed-detection-dominated / partial visibility 更强的场景。
- 对 effective target-wise fusion graph 的日志化诊断，而不是只看最终 OSPA。

## Reproducibility

核心复现命令:

```bash
octave --quiet --eval "test_aa_lmb_track_merging"

octave --quiet --eval "addpath('RUN/GA'); [reportPath, summary] = runMultisensorFilters_formation_4plus4_TieredLinkAblation(1, 1, true, struct(), true, 'fidFiaExistenceRefinement', struct(), [1 3 4], struct('targetFormationLifeSpan', 24));"

octave --quiet --eval "addpath('RUN/AA'); [reportPath, summary] = runAaBalancedCardinalityValidation(1, 1, true, struct('saveMat', false, 'saveCheckpoints', false, 'progressEverySteps', 0), struct(), true, [1 5 6]);"

octave --quiet --eval "addpath('RUN/AA'); [reportPath, summary] = runAaBalancedCardinalityValidation(1, 1, true, struct('saveMat', false, 'saveCheckpoints', false, 'progressEverySteps', 0, 'maximumNumberOfGmComponents', 1, 'existenceThreshold', 0.08), struct(), true, [3]);"

octave --quiet --eval "addpath('RUN/AA'); [reportPath, summary] = runAaBalancedCardinalityValidation(1, 1, true, struct('saveMat', false, 'saveCheckpoints', false, 'progressEverySteps', 0, 'maximumNumberOfGmComponents', 1, 'existenceThreshold', 0.08), struct(), true, [4]);"

octave --quiet --eval "addpath('RUN/AA'); [reportPath, summary] = runAaBalancedCardinalityValidation(1, 1, true, struct('saveMat', false, 'saveCheckpoints', false, 'progressEverySteps', 0), struct(), true, [7 8]);"
```

Validation:

```bash
git diff --check
python3 /Users/dex/.codex/skills/auto-research/scripts/evidence_lint.py docs/AA_BASELINE_FAILURE_DIAGNOSTIC_CN.md
```

## Open Issues

- 需要多 seed 验证 spatial-KLA AA hybrid 是否稳定达到 GA 同级，而不是 seed=2 偶然。
- 需要日志化每个 Bernoulli 的 target-wise spatial/existence effective sensor set、weight entropy、mode count，才能把机理从指标层推进到可控策略。
- FI target-wise AA 目前用的是轻量 surrogate，不等价于 Part V 的完整 CT-FI multi-rate formulation。
- 当前 hybrid 是最小实现，尚未做 regime-aware switching 或 effective graph guard。

## Recommendation

不要继续把“纯 AA 调到 GA 一样好”作为主目标。短期可以把 tuned AA (`maxGM=1`, `existenceThreshold=0.08`) 作为 pure-AA baseline 的更公平版本；真正可用的主线应转向 target-wise hybrid average fusion。当前最小可行形态已经成立: spatial branch 使用 KLA，existence/PHD branch 保留 AA；下一步应补 N>=10 paired validation 和 target-wise effective information graph，让 hybrid 从工程修补变成可解释方法。
