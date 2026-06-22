# Label/Uncertainty-Aware AA-KLA 融合规则草案

日期: 2026-06-22

## Question

如何把当前 AA-existence + KLA-spatial hybrid 从场景调参推进到可泛化的方法设计，使剩余 consensus Loc gap 不再靠 `existenceThreshold`、bridge prior 或 existence gate 搜索解决，而是由 label consensus、spatial dispersion 和 posterior uncertainty 共同决定融合行为。

Decision target: 后续是否值得实现一个新的 AA-family arm，并在 1-trial gate、N5 failure-block falsification、N50 paired validation 和 N50 ablation 中验证。

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
- 不在本文档中新增实验结果；本文档是方法 spec 和后续验证合同。

## Risk Tier

L2。该文档会影响后续算法实现和 paper-facing 方法叙事，但当前只新增内部研究 spec，不构成最终性能 claim。

## Claims

| ID | Claim | Confidence | Evidence IDs | Caveats |
| --- | --- | --- | --- | --- |
| C1 | 当前 tuned AA-existence + KLA-spatial hybrid 的核心收益来自 spatial-KLA，而不是 FID-FIA existence refinement 或阈值搜索。 | High | E1, E2, E3 | 当前证据来自 24-step diagnostic scenario。 |
| C2 | 剩余 Loc gap 不能用单因子修补解释；强 existence gate、threshold=0.20 和 bridge-aware prior 都在 N5 failure block 变差。 | High | E4, E5, E6 | N5 block 不是 N50 全局因果证明，但足以排除这些作为主方法。 |
| C3 | label-level attribution 显示 failure block 同时存在 label-set split 和 same-label spatial spread。 | High | E7, E8 | 仍是 attribution，不是修复。 |
| C4 | 因此下一步方法应连续建模 label support、spatial agreement 和 posterior uncertainty，而不是用固定场景阈值做 hard switch。 | Medium-High | E4-E8 | 需要实现和实验验证。 |
| C5 | 一个可实现的候选规则是 overlap-weighted spatial KLA + between-posterior uncertainty inflation + support-aware existence tempering。 | Medium | E9, E10 | 目前是理论/工程 spec，尚无性能数据。 |

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

## 实现计划

建议新增一个明确命名的 experimental arm，而不是改写当前 tuned arm:

```matlab
arms(...).name = 'Label-uncertainty spatial-KLA AA';
cfg.aaSpatialFusionMode = 'kla';
cfg.useAaLabelUncertaintyFusion = true;
cfg.useAaLabelExistenceTempering = true;
```

代码位置:

- `aaLmbTrackMerging.m`: 消费 `useAaLabelUncertaintyFusion`，在 KLA spatial 前计算 `A_s` 和 `beta_s`，并输出 `Sigma_out = Sigma_KLA + B`。
- `computeAdaptiveFusionWeights.m`: 不需要先改；该规则使用已有 branch weights 作为 base。
- `runAaBalancedCardinalityValidation.m`: 新增 arm，并在 report 中记录 `labelAgreementMean`、`labelSupportNeff`、`uncertaintyInflationTrace` 的 summary diagnostics。

最小 regression:

1. 两个一致 posterior 时，输出与普通 spatial-KLA 数值一致或接近。
2. 一个 spatial outlier 与两个一致 posterior 同时存在时，outlier 权重下降，均值接近一致 posterior。
3. 单有效 posterior 时不被 hard penalty。
4. low-support false label 的 existence odds 下降但不是硬删。

实验 gate:

1. N1 paired sanity: 与 Tuned spatial-KLA AA 和 GA Balanced/GA final 比较，不接受只改善一个 seed 上单个指标而显著破坏 local metrics 的 arm。
2. N5 failure-block falsification: 只用作 falsification，不在这里调参。若 label-set split 和 same-label spread rows 同时改善，才进入 N50。
3. N50 paired validation: 目标仍是 OSPA、Loc、Card、E-OSPA、hOspa、RMSE、CardErr 均优于两个 GA modes。
4. N50 ablation: 至少比较 current tuned, overlap-weighted spatial only, covariance inflation only, existence tempering only, full rule。

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
| E9 | code | `multisensorLmb/aaLmbTrackMerging.m` | C5, location where KLA spatial branch can consume robust beta weights | medium |
| E10 | code | `RUN/AA/runAaBalancedCardinalityValidation.m` | C5, report and arm infrastructure for a new experimental rule | medium |

## Verification Record

Independence status: self-check only. 本文档尚未经过独立 verifier；它把已有实验结果转化为方法 spec，但未实现或验证新 arm。

已检查:

- 现有 label-level attribution report 中确实包含 `Unique labels`、`Full labels`、`Mean label cover` 和 `Max label spread`。
- 当前 branch HEAD 为 `dc9431a`，远端 `origin/codex/aa-target-wise-fix` 同步到同一提交。

待检查:

- 新 rule 的 regression tests。
- N1/N5/N50 paired performance。
- 是否需要对 covariance inflation 做 OSPA/eOSPA 之外的指标补充。

## Risk and Escalation

主要风险:

- 把一个尚未实验的 method spec 写成 performance claim。
- overlap weighting 在 missed-detection dominated regime 下压低真实单传感器观测。
- covariance inflation 改善 uncertainty consistency 但不改善 point Loc。

升级条件:

- 若进入 paper-facing claim，需要 N50 paired validation、N50 ablation 和独立 verifier。
- 若 full rule 只在 seeds 12-16 上好，不应上 N50；应回到 rule 设计而不是调参。

## Reproducibility

现有 evidence 可复现命令:

```bash
octave --quiet RUN/AA/runAaLocFailureAttributionN5.m
python3 /Users/dex/.codex/skills/auto-research/scripts/evidence_lint.py docs/AA_LABEL_UNCERTAINTY_AWARE_FUSION_RULE_CN.md
```

后续实现后应新增:

```bash
octave --quiet --eval "test_aa_lmb_track_merging"
octave --quiet RUN/AA/runAaLabelUncertaintyDiagnosticN1.m
octave --quiet RUN/AA/runAaLabelUncertaintyDiagnosticN5.m
```

## Open Issues

- `Q` 的 log-odds tempering 是否过强，需要先用 synthetic regression 确认边界行为；不能在 N5 failure block 上调。
- `BC_sj` 对高维 covariance determinant 的数值稳定性需要 regularization。
- 对 label-set split 的根因可能在 distributed pruning / birth management，而不只在 local fusion rule。
- N50 consensus Loc 仍未被证明可通过该规则改善。

## Recommendation

下一步应实现 `Label-uncertainty spatial-KLA AA` experimental arm，但只按上述无场景阈值规则实现，不做 seeds 12-16 参数搜索。先跑 synthetic regression 和 N1 sanity；若没有同时保持 OSPA/Card/local metrics，再回到规则设计，不进入 N50。
