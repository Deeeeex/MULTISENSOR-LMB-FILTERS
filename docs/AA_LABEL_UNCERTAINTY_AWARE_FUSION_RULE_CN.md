# Label/Uncertainty-Aware AA-KLA 融合规则草案

日期: 2026-06-22

## Question

如何把当前 AA-existence + KLA-spatial hybrid 从场景调参推进到可泛化的方法设计，使剩余 consensus Loc gap 不再靠 `existenceThreshold`、bridge prior 或 existence gate 搜索解决，而是由 label consensus、spatial dispersion 和 posterior uncertainty 共同决定融合行为。

Decision target: 当前 label/uncertainty-aware 原型是否值得进入 N5 failure-block falsification、N50 paired validation 和 N50 ablation。

## Scope

包含:

- 当前分支 `/Users/dex/Desktop/Code/Research/MULTISENSOR-LMB-FILTERS`。
- `multisensorLmb/aaLmbTrackMerging.m` 的 AA existence 与 KLA spatial 消费端。
- `multisensorLmb/computeAdaptiveFusionWeights.m` 的 branch-specific adaptive weights。
- `RUN/AA/runAaBalancedCardinalityValidation.m` 的 top-time 和 label-level attribution。
- N50 tuned hybrid、N50 design ablation、N5 negative probes 与 label-level attribution reports。

排除:

- 不声明该草案已经优于 GA Balanced；当前 N50 consensus Loc 仍未过关。
- 不把本规则写成 strict Bernoulli-AA 的闭式最优解。
- 不沿 seeds 12-16 搜索固定阈值、power 或 bridge coefficient。
- 不把 N1 结果外推成统计结论；当前 N1 只作为 gate/falsification。

## Risk Tier

L2。该文档会影响后续算法实现和 paper-facing 方法叙事，但当前只新增内部研究 spec，不构成最终性能 claim。

## Claims

| ID | Claim | Confidence | Evidence IDs | Caveats |
| --- | --- | --- | --- | --- |
| C1 | 当前 tuned AA-existence + KLA-spatial hybrid 的核心收益来自 spatial-KLA，而不是 FID-FIA existence refinement 或阈值搜索。 | High | E1, E2, E3 | 当前证据来自 24-step diagnostic scenario。 |
| C2 | 剩余 Loc gap 不能用单因子修补解释；强 existence gate、threshold=0.20 和 bridge-aware prior 都在 N5 failure block 变差。 | High | E4, E5, E6 | N5 block 不是 N50 全局因果证明，但足以排除这些作为主方法。 |
| C3 | label-level attribution 显示 failure block 同时存在 label-set split 和 same-label spatial spread。 | High | E7, E8 | 仍是 attribution，不是修复。 |
| C4 | 因此下一步方法应连续建模 label support、spatial agreement 和 posterior uncertainty，而不是用固定场景阈值做 hard switch。 | Medium-High | E4-E8 | 需要实现和实验验证。 |
| C5 | overlap-weighted spatial KLA + between-posterior uncertainty inflation + support-aware existence tempering 已实现为默认关闭的 experimental arm，并通过 synthetic regression。 | High | E9, E10, E11 | 这只证明代码边界行为，不证明场景性能。 |
| C6 | 当前 full rule 未通过 N1 gate: OSPA 改善但 Loc、Card、E-OSPA 和 CardErr 显著变差，不能进入 N5/N50。 | High | E12, E13 | N1 是 gate，不是统计结论。 |
| C7 | spatial-overlap only 也未通过 N1 gate: 它避免了 full rule 的 cardinality collapse，但 OSPA/Loc 均略差于同 seed tuned baseline。 | Medium-High | E14, E15 | 该 report 通过 adaptive override 运行，arm name 仍显示 Tuned spatial-KLA AA，需读取 config 字段确认。 |
| C8 | covariance-inflation only 也未通过 N1 gate: 它是更干净的 uncertainty-propagation 假设，但未改善 Loc/OSPA，并带来约 1.5x runtime。 | Medium-High | E16, E17 | 单 seed gate 只用于否定进入 N5/N50，不作统计 claim。 |
| C9 | naive label-lifecycle decoupling 也未通过 N1 gate: 分离 output threshold 与 pruning threshold 是合理抽象，但直接保留全部低置信 label 会扩大 cardinality clutter 和 runtime。 | Medium-High | E18-E21 | 这否定的是 naive lifecycle arm，不否定更强的 label-consensus objective。 |
| C10 | mature-label lifecycle 修复了 naive lifecycle 的 runtime/cardinality 膨胀，但 N1 仍未通过: Loc/E-OSPA/RMSE 微小改善，OSPA/Card/CardErr 变差。 | Medium-High | E22, E23, E24 | 这说明仅用 trajectory age 作为 label survival 证据仍太弱。 |
| C11 | output-history lifecycle 在 N1 和 N5 上近似无副作用，OSPA/Loc/E-OSPA/RMSE 有极小 paired 改善且 Card/CardErr 持平，但量级太小，不足以上 N50。 | Medium-High | E22, E23, E25-E27 | 可作为低风险 primitive 保留；不能作为达成目标的主方法。 |

## 方法设计

### 输入

对 local fusion problem 中的某个 Bernoulli label `ell`，邻域传感器 `s=1..S_k` 给出:

```text
(r_s^ell, p_s^ell(x), alpha_x_s, alpha_r_s)
```

其中 `r_s^ell` 是 local existence，`p_s^ell(x)` 是 moment-matched Gaussian `(mu_s, Sigma_s)`，`alpha_x_s` 和 `alpha_r_s` 分别是当前 spatial/existence branch weights。缺失或已剪枝的 label 以 `r_s^ell=0` 处理，不强行制造 position。

### 1. Label support

定义连续支持质量:

```text
m_s = alpha_r_s * r_s^ell
M = sum_s m_s
Neff_label = M^2 / sum_s m_s^2
```

`M` 表示该 label 在邻域内的 weighted support，`Neff_label` 表示支持是否集中在少数 sensor。这里不设固定 support threshold；后续只用连续分数调制 existence odds。

### 2. Uncertainty-normalized spatial agreement

对任意两个 local posterior，用 Gaussian overlap 衡量同一 label 的空间一致性:

```text
BC_sj = exp(-1/8 * d_sj' * S_sj^{-1} * d_sj)
        * |Sigma_s|^(1/4) |Sigma_j|^(1/4) / |S_sj/2|^(1/2)

d_sj = mu_s - mu_j
S_sj = (Sigma_s + Sigma_j) / 2
```

这是 Bhattacharyya coefficient 的 Gaussian 形式，天然按 posterior covariance 归一化。传感器距离在 covariance 尺度下小则 overlap 高，距离在 uncertainty 尺度下大则 overlap 低；不需要按当前场景手设米级阈值。

每个 sensor 的 label-spatial agreement:

```text
A_s = sum_{j != s} alpha_x_j r_j^ell BC_sj / sum_{j != s} alpha_x_j r_j^ell
```

若只有一个有效 posterior，则 `A_s=1`，避免在低可见性 regime 下硬惩罚单传感器。

### 3. Overlap-weighted KLA spatial branch

当前 spatial-KLA 使用:

```text
beta_s proportional to alpha_x_s
```

候选规则改为:

```text
beta_s proportional to alpha_x_s * r_s^ell * A_s
```

然后执行 Gaussian KLA:

```text
K = sum_s beta_s Sigma_s^{-1}
h = sum_s beta_s Sigma_s^{-1} mu_s
Sigma_KLA = K^{-1}
mu_KLA = Sigma_KLA h
```

这不是 threshold gate。它是连续的 robust KLA: low-existence 或与其他 posterior overlap 很低的 local posterior 会自然降权；如果多个 posterior 一致，`A_s` 接近 1，规则退化回当前 spatial-KLA。

### 4. Between-posterior uncertainty inflation

为了避免 KLA 在 disagreement regime 下过度自信，输出 covariance 加上 posterior 间分散项:

```text
mu_bar = sum_s beta_s mu_s
B = sum_s beta_s (mu_s - mu_bar)(mu_s - mu_bar)'
Sigma_out = Sigma_KLA + B
```

这一步不改变 point estimate 的鲁棒均值，但把 same-label spatial spread 显式反映到 uncertainty。它适合后续 eOSPA / covariance-aware gating，也避免把 disagreement 伪装成高置信单 Gaussian。

### 5. Support-aware existence tempering

保留 AA existence 的线性均值作为 base:

```text
r_AA = sum_s alpha_r_s r_s^ell
```

再用 label support 和 spatial agreement 构造连续 tempering factor:

```text
Q = M * mean_s(A_s weighted by m_s)
logit(r_out) = logit(r_AA) + log(max(Q, eps))
```

含义:

- support 高且空间一致时，`Q` 接近 1，existence 基本不变；
- support 低或 spatial overlap 低时，existence odds 被温和降低；
- 不使用 hard delete，也不依赖固定场景阈值。

这个 tempering 要作为新 arm 的可选机制，与当前 AA existence baseline 做 ablation；不能直接替换主线。

## 与已有失败证据的关系

### 对 label-set split

当不同 local neighborhoods 输出不同 label 假设时，每个 label 的 `M`、`Neff_label` 和 `Q` 会下降。规则不会直接按 label 数硬删，而是让低支持 label 的 existence odds 下降。这样可适配不同 sensor 数和拓扑密度。

### 对 same-label spatial spread

当 label 一致但位置分散时，`BC_sj` 下降，离群 posterior 的 `A_s` 下降。spatial branch 会转向 overlap 更高的 posterior 子集，同时 `B` 把剩余 disagreement 写进 covariance。

### 对 missed-detection regime

如果只有少数 sensor 看见目标，但它们的 posterior 内部一致，`A_s` 不会因为缺少其他 sensor 被硬惩罚；existence 仍由 AA base 和 support tempering 连续控制。因此该规则比固定 existence gate 更适合 partial visibility。

N1 gate 暴露出一个重要修正: `Q = M * agreement` 的 support tempering 会把 absolute support mass 引入 existence odds，在 partial visibility / local-neighborhood setting 下过强，导致 CardErr 从 `0.066250` 恶化到 `2.046250`。因此 support mass 不应直接作为 default existence tempering；后续只能作为诊断或在有明确 missed-detection model 的条件下进入 odds correction。

另一个上游修正假设是把 estimate extraction 的 output threshold 与 recursive Bernoulli pruning threshold 分离。这个抽象是合理的，因为 `existenceThreshold=0.18` 同时用于输出和剪枝会让局部 missed detection 永久删除 label。但 naive arm 只把 pruning threshold 降回 `0.01`，没有给低置信 label 引入跨邻域 consensus 约束，N1 gate 显示它把 label survival split 转成了 cardinality clutter: Card 从 `0.018750` 恶化到 `0.040000`，CardErr 从 `0.066250` 恶化到 `0.097500`，runtime 约 `1.598x`。

两个更收紧的 lifecycle 变体进一步说明问题边界:

- mature-label lifecycle: 低于 output threshold 但高于 pruning threshold 的 label 只有在 `trajectoryLength >= 1` 时才递推。它避免了 naive arm 的大幅 runtime/cardinality 膨胀，但 N1 上 OSPA/Card/CardErr 仍略差。
- output-history lifecycle: 低置信 label 只有在最近一次 MAP 输出中过时不超过 `labelPruningMaxOutputGap=1` 时才递推。它在 N1/N5 上基本无副作用，但收益只有 `1e-4` 量级，不足以解释或修复 N50 Loc gap。

## 实现计划

当前已把 label-uncertainty fusion 拆成独立方法开关，而不是把多个机制耦合成一个场景调参臂:

```matlab
arms(...).name = 'Uncertainty-inflated spatial-KLA AA';
cfg.aaSpatialFusionMode = 'kla';
cfg.useAaLabelUncertaintyFusion = true;
cfg.useAaLabelSpatialOverlapWeights = false;
cfg.useAaLabelUncertaintyInflation = true;
cfg.useAaLabelExistenceTempering = false;
```

另外新增了一个 label-lifecycle arm，用于验证“输出/递推解耦”这个上游假设:

```matlab
arms(...).name = 'Label-lifecycle spatial-KLA AA';
cfg.aaSpatialFusionMode = 'kla';
cfg.labelPruningThreshold = 0.01;
```

后续把 naive lifecycle 拆成两个更明确的时序保护臂:

```matlab
arms(...).name = 'Mature-label lifecycle spatial-KLA AA';
cfg.labelPruningThreshold = 0.01;
cfg.labelPruningMinTrajectoryLength = 1;

arms(...).name = 'Output-history lifecycle spatial-KLA AA';
cfg.labelPruningThreshold = 0.01;
cfg.labelPruningProtectionMode = 'last-output';
cfg.labelPruningMaxOutputGap = 1;
```

代码位置:

- `aaLmbTrackMerging.m`: 消费 `useAaLabelUncertaintyFusion`，并用 `useAaLabelSpatialOverlapWeights`、`useAaLabelUncertaintyInflation`、`useAaLabelExistenceTempering` 分别控制 overlap reweight、between-posterior covariance inflation 和 existence odds tempering。
- `applyLmbLabelLifecycleThresholds.m`: 将 output threshold 与 recursive pruning threshold 分离；默认 `labelPruningThreshold` 缺省时等于 `existenceThreshold`，保持旧行为；可用 `trajectory-age` 或 `last-output` 保护低置信 label。
- `runParallelUpdateLmbFilter.m`: MAP 输出只看超过 output threshold 的 Bernoulli；递推对象按 pruning threshold 保留，并把 MAP 输出 label 的 `lastOutputTime` 写回递推对象。
- `common/generateMultisensorModel.m` / `common/generateModel.m`: 为 Bernoulli object 增加 `lastOutputTime` 字段，避免 birth 和 surviving objects 拼接时结构不一致。
- `computeAdaptiveFusionWeights.m`: 不需要先改；该规则使用已有 branch weights 作为 base。
- `runAaBalancedCardinalityValidation.m`: arm 12 明确命名为 `Uncertainty-inflated spatial-KLA AA`；arm 13/14 明确命名为 lifecycle variants；report 记录上述开关，便于 N1/N5 gate 复现。

最小 regression:

1. 两个一致 posterior 时，输出与普通 spatial-KLA 数值一致或接近。已通过 E11。
2. 一个 spatial outlier 与两个一致 posterior 同时存在时，outlier 权重下降，均值接近一致 posterior。已通过 E11。
3. low-support false label 的 existence odds 下降但不是硬删。已通过 E11。
4. covariance inflation 可在不启用 overlap reweight 的条件下独立使用。已通过 E11。
5. 单有效 posterior 不会被 overlap 逻辑压掉。已通过 E11。
6. lifecycle 默认保持旧单阈值行为，打开 `labelPruningThreshold` 后低于输出阈值但高于剪枝阈值的 label 只递推不输出。已通过 E19。
7. output-history lifecycle 只保护最近输出过的低置信 label；过期或从未输出的低置信 label 不递推。已通过 E23。

实验 gate:

1. N1 paired sanity: 与 Tuned spatial-KLA AA 和 GA Balanced/GA final 比较，不接受只改善一个 seed 上单个指标而显著破坏 local metrics 的 arm。当前 full rule 未通过该 gate。
2. N5 failure-block falsification: 只用作 falsification，不在这里调参。若 label-set split 和 same-label spread rows 同时改善，才进入 N50。
3. N50 paired validation: 目标仍是 OSPA、Loc、Card、E-OSPA、hOspa、RMSE、CardErr 均优于两个 GA modes。
4. N50 ablation: 只有在某个方法假设通过 N1 和 N5 falsification 后才运行；当前 full、overlap-only、inflation-only、naive lifecycle、mature lifecycle 都不满足进入条件；output-history lifecycle 虽然 N5 near-neutral，但效果太小，不满足上 N50 的门槛。

## Evidence Ledger

| ID | Type | Source or artifact | What it supports | Strength |
| --- | --- | --- | --- | --- |
| E1 | experiment | `RUN/AA/AA_BALANCED_CARDINALITY_VALIDATION_N50_SEED1_20260621_183039.md` | C1, current tuned N50 result | strong |
| E2 | experiment | `RUN/GA/GA_TIERED_LINK_ABLATION_N50_SEED1_20260621_183039.md` | C1, GA N50 reference | strong |
| E3 | experiment | `RUN/AA/AA_BALANCED_CARDINALITY_VALIDATION_N50_SEED1_20260621_215745.md` | C1, N50 design ablation | strong |
| E4 | experiment | `RUN/AA/AA_BALANCED_CARDINALITY_VALIDATION_N5_SEED11_20260622_095821.md` | C2, existence-gated spatial-KLA negative probe | medium |
| E5 | experiment | `RUN/AA/AA_BALANCED_CARDINALITY_VALIDATION_N5_SEED11_20260622_104136.md` | C2, threshold=0.20 negative probe | medium |
| E6 | experiment | `RUN/AA/AA_BALANCED_CARDINALITY_VALIDATION_N5_SEED11_20260622_104752.md` | C2, bridge-aware prior negative probe | medium |
| E7 | experiment | `RUN/AA/AA_BALANCED_CARDINALITY_VALIDATION_N5_SEED11_20260622_110215.md` | C3, label-level attribution table | strong |
| E8 | command log | `RUN/AA/AA_LABEL_FAILURE_ATTRIBUTION_N5_SEED11_20260622.log` | C3, reproduction command output | medium |
| E9 | code | `multisensorLmb/aaLmbTrackMerging.m` | C5, implemented overlap-weighted KLA, covariance inflation and existence tempering switches | strong |
| E10 | code | `RUN/AA/runAaBalancedCardinalityValidation.m` | C5, experimental arm and report config fields | strong |
| E11 | command | `octave --quiet --eval "test_aa_lmb_track_merging"` output: tests 1-9 passed | C5, synthetic regression | strong |
| E12 | experiment | `RUN/AA/AA_BALANCED_CARDINALITY_VALIDATION_N1_SEED1_20260622_112156.md` | C6, full rule N1 negative gate | strong |
| E13 | command log | `RUN/AA/AA_LABEL_UNCERTAINTY_N1_SEED1_20260622.log` | C6, full rule command output | medium |
| E14 | experiment | `RUN/AA/AA_BALANCED_CARDINALITY_VALIDATION_N1_SEED1_20260622_112431.md` | C7, spatial-overlap only N1 negative gate | medium |
| E15 | command log | `RUN/AA/AA_LABEL_OVERLAP_SPATIAL_N1_SEED1_20260622.log` | C7, spatial-overlap only command output | medium |
| E16 | experiment | `RUN/AA/AA_BALANCED_CARDINALITY_VALIDATION_N1_SEED1_20260622_113133.md` | C8, covariance-inflation only N1 negative gate | strong |
| E17 | command log | `RUN/AA/AA_LABEL_INFLATION_ONLY_N1_SEED1_20260622.log` | C8, covariance-inflation only command output | medium |
| E18 | code | `multisensorLmb/applyLmbLabelLifecycleThresholds.m` and `multisensorLmb/runParallelUpdateLmbFilter.m` | C9, output/pruning lifecycle split implementation | strong |
| E19 | command | `octave --quiet --eval "test_lmb_label_lifecycle_thresholds"` output: tests 1-3 passed | C9, lifecycle synthetic regression | strong |
| E20 | experiment | `RUN/AA/AA_BALANCED_CARDINALITY_VALIDATION_N1_SEED1_20260622_113821.md` | C9, naive lifecycle N1 negative gate | strong |
| E21 | command log | `RUN/AA/AA_LABEL_LIFECYCLE_N1_SEED1_20260622.log` | C9, lifecycle command output | medium |
| E22 | code | `common/generateMultisensorModel.m`, `common/generateModel.m`, `multisensorLmb/applyLmbLabelLifecycleThresholds.m`, `multisensorLmb/runParallelUpdateLmbFilter.m`, `RUN/AA/runAaBalancedCardinalityValidation.m` | C10-C11, mature/output-history lifecycle implementation | strong |
| E23 | command | `octave --quiet --eval "test_lmb_label_lifecycle_thresholds"` output: tests 1-5 passed | C10-C11, lifecycle regression including output-history mode | strong |
| E24 | experiment | `RUN/AA/AA_BALANCED_CARDINALITY_VALIDATION_N1_SEED1_20260622_114437.md` and `RUN/AA/AA_MATURE_LABEL_LIFECYCLE_N1_SEED1_20260622.log` | C10, mature-label lifecycle N1 near-neutral negative gate | strong |
| E25 | experiment | `RUN/AA/AA_BALANCED_CARDINALITY_VALIDATION_N1_SEED1_20260622_114808.md` and `RUN/AA/AA_OUTPUT_HISTORY_LIFECYCLE_N1_SEED1_20260622.log` | C11, output-history lifecycle N1 no-regression gate | strong |
| E26 | experiment | `RUN/AA/AA_BALANCED_CARDINALITY_VALIDATION_N5_SEED11_20260622_115033.md` | C11, output-history lifecycle N5 near-neutral gate | strong |
| E27 | command log | `RUN/AA/AA_OUTPUT_HISTORY_LIFECYCLE_N5_SEED11_20260622.log` | C11, output-history lifecycle N5 command output | medium |

## Verification Record

Independence status: self-check only. 本文档尚未经过独立 verifier；当前实现和 N1 gate 均由同一 worker lane 完成，不能作为 paper-facing 结论。

已检查:

- 现有 label-level attribution report 中确实包含 `Unique labels`、`Full labels`、`Mean label cover` 和 `Max label spread`。
- `test_aa_lmb_track_merging` 通过，覆盖 target-wise weights、strict AA、existence-gated KLA、label-uncertainty synthetic cases、inflation-only 和 single-supported-posterior case。
- N1 full rule gate 失败: Tuned baseline OSPA/Loc/Card 为 `1.682637/1.474567/0.018750`；full rule 为 `1.419895/2.728531/0.063750`，local E-OSPA/CardErr 为 `4.529137/2.046250`。
- N1 spatial-overlap only gate 失败: OSPA/Loc/Card 为 `1.709002/1.489258/0.018750`。
- N1 covariance-inflation only gate 失败: Tuned baseline OSPA/Loc/Card 为 `1.682637/1.474567/0.018750`；inflation-only 为 `1.727916/1.540082/0.017500`，local E-OSPA/RMSE/CardErr 为 `2.066921/4.185329/0.067500`。
- `test_lmb_label_lifecycle_thresholds` 通过，覆盖默认旧行为、递推保留低于输出阈值的 label、adaptiveFusion 覆盖剪枝阈值。
- N1 naive lifecycle gate 失败: Tuned baseline OSPA/Loc/Card 为 `1.682637/1.474567/0.018750`；lifecycle 为 `1.745000/1.484851/0.040000`，local E-OSPA/RMSE/CardErr 为 `2.073722/4.157186/0.097500`，runtime 相对 tuned 为 `1.598x`。
- mature-label lifecycle N1 gate 未通过: Tuned baseline OSPA/Loc/Card 为 `1.682637/1.474567/0.018750`；mature-label 为 `1.684729/1.474432/0.020000`，local E-OSPA/RMSE/CardErr 为 `2.028008/4.144330/0.067500`。
- output-history lifecycle N1 no-regression: OSPA/Loc/Card 为 `1.682562/1.474512/0.018750`，local E-OSPA/RMSE/CardErr 为 `2.028839/4.144903/0.066250`。
- output-history lifecycle N5 near-neutral: Tuned OSPA/Loc/Card 为 `1.702915/1.529716/0.034250`；output-history 为 `1.702857/1.529653/0.034250`，local E-OSPA/RMSE/CardErr 为 `2.032737/3.588080/0.086250`，paired OSPA/Loc reductions 仅 `0.000058/0.000063`。

待检查:

- label-set split 的修复需要一个跨邻域 label-consensus objective；output-history lifecycle 只能作为低风险 survival primitive，收益太小。
- 是否需要对 uncertainty consistency 另设 NEES/NIS 类指标；当前 OSPA/eOSPA/RMSE gate 不支持 covariance inflation 作为主线性能改进。

## Risk and Escalation

主要风险:

- 把一个尚未实验的 method spec 写成 performance claim。
- overlap weighting 在 missed-detection dominated regime 下压低真实单传感器观测。
- covariance inflation 可能改善 uncertainty consistency，但 N1 OSPA/eOSPA/RMSE 已显示它不是当前主线 Loc gap 的直接解。
- support-mass existence tempering 在 local-neighborhood setting 下造成 cardinality collapse。
- naive lifecycle decoupling 会保留过多低置信 label，造成 cardinality clutter 和 runtime 上升。
- output-history lifecycle 低风险但收益太小；不能把 near-neutral N5 外推成 N50 success。

升级条件:

- 若进入 paper-facing claim，需要 N50 paired validation、N50 ablation 和独立 verifier。
- 当前 full rule 已在 N1 gate 失败，不应进入 N5/N50；应回到 rule 设计而不是调参。

## Reproducibility

现有 evidence 可复现命令:

```bash
octave --quiet RUN/AA/runAaLocFailureAttributionN5.m
octave --quiet --eval "test_aa_lmb_track_merging"
octave --quiet --eval "test_lmb_label_lifecycle_thresholds"
octave --quiet --eval "addpath('RUN/AA'); aaControls=struct('saveMat',false,'saveCheckpoints',false,'progressEverySteps',0,'existenceThreshold',0.18); [reportPath, summary]=runAaBalancedCardinalityValidation(1,1,true,aaControls,struct(),true,[9 12]); disp(summary.consensus); disp(summary.local.meanAcrossSensors);"
octave --quiet --eval "addpath('RUN/AA'); aaControls=struct('saveMat',false,'saveCheckpoints',false,'progressEverySteps',0,'existenceThreshold',0.18); [reportPath, summary]=runAaBalancedCardinalityValidation(1,1,true,aaControls,struct(),true,[9 13]); disp(summary.consensus); disp(summary.local.meanAcrossSensors);"
octave --quiet --eval "addpath('RUN/AA'); aaControls=struct('saveMat',false,'saveCheckpoints',false,'progressEverySteps',0,'existenceThreshold',0.18); [reportPath, summary]=runAaBalancedCardinalityValidation(1,1,true,aaControls,struct(),true,[9 14]); disp(summary.consensus); disp(summary.local.meanAcrossSensors);"
octave --quiet --eval "addpath('RUN/AA'); aaControls=struct('saveMat',false,'saveCheckpoints',false,'progressEverySteps',0,'existenceThreshold',0.18); [reportPath, summary]=runAaBalancedCardinalityValidation(5,11,true,aaControls,struct(),true,[9 14]); disp(summary.consensus); disp(summary.local.meanAcrossSensors);"
octave --quiet --eval "addpath('RUN/AA'); aaControls=struct('saveMat',false,'saveCheckpoints',false,'progressEverySteps',0,'existenceThreshold',0.18); overrides=struct('useAaLabelUncertaintyFusion',true,'useAaLabelUncertaintyInflation',false,'useAaLabelExistenceTempering',false); [reportPath, summary]=runAaBalancedCardinalityValidation(1,1,true,aaControls,struct(),true,[9],overrides); disp(summary.consensus); disp(summary.local.meanAcrossSensors);"
python3 /Users/dex/.codex/skills/auto-research/scripts/evidence_lint.py docs/AA_LABEL_UNCERTAINTY_AWARE_FUSION_RULE_CN.md
```

后续实现后应新增:

```bash
octave --quiet --eval "test_aa_lmb_track_merging"
octave --quiet RUN/AA/runAaLabelUncertaintyDiagnosticN1.m
octave --quiet RUN/AA/runAaLabelUncertaintyDiagnosticN5.m
```

## Open Issues

- `Q = M * agreement` 的 log-odds tempering 已被 N1 gate 证明过强；后续需要把 absolute support mass 从 default existence correction 中移除或换成 model-normalized risk。
- `BC_sj` 对高维 covariance determinant 的数值稳定性需要 regularization。
- 对 label-set split 的根因可能在 distributed pruning / birth management，但 output-history 只带来 `1e-4` 量级收益；真正的下一步应建模跨邻域 label-consensus，而不是继续只改单滤波器 lifecycle。
- N50 consensus Loc 仍未被证明可通过该规则改善。

## Recommendation

当前 `Uncertainty-inflated spatial-KLA AA`、`Mature-label lifecycle spatial-KLA AA` 和 `Output-history lifecycle spatial-KLA AA` experimental arms 已实现。full rule、spatial-overlap only、covariance-inflation only、naive lifecycle 和 mature lifecycle 都未通过 N1 gate；output-history lifecycle 通过了 no-regression N1/N5，但收益太小，不进入 N50。下一步应停止在单个 local filter 内继续修 lifecycle，转向跨邻域 label-consensus objective，让 label survival 同时考虑时序证据、邻域支持和 cardinality budget。
