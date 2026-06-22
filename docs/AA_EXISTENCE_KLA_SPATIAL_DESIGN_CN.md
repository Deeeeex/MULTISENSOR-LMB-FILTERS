# AA-existence + KLA-spatial hybrid 设计说明

日期: 2026-06-22

## Question

如何把 AA Fusion 用在当前 distributed LMB / partial-FOV / tiered packet-drop 场景里，使它不是简单复刻 pure AA spatial mixture 的失败路径，而是形成一个可解释、可消融、可继续研究的 AA-existence + KLA-spatial hybrid。

## Scope

包含范围:

- `multisensorLmb/aaLmbTrackMerging.m` 中的 AA existence 与 spatial 消费端。
- `RUN/AA/runAaBalancedCardinalityValidation.m` 中的 AA / spatial-KLA AA arms。
- N10 与 N50 tuned hybrid 对照结果。
- N50 design ablation 结果。

排除范围:

- 不把 hybrid 写成严格 Bernoulli-AA 的理论闭式形式。
- 不在本文档中重新综述全部 AA Fusion 文献；文献脉络见 `/Users/dex/Desktop/Code/Research/aa_fusion_review/AA_FUSION_REVIEW_CN.md`。
- 不把当前 N50 结果写成“所有指标严格优于 GA Balanced”；consensus Loc 仍有小幅缺口。

## Risk Tier

L2。该设计会影响研究路线和后续论文叙事，但当前只作为本地研究分支的算法说明与实验计划，不直接形成投稿主张。

## Claims

| ID | Claim | Confidence | Evidence IDs | Caveats |
| --- | --- | --- | --- | --- |
| C1 | Pure AA spatial mixture 在当前场景的主失败源是 spatial mode union 与 MAP/top-Gaussian 输出不匹配。 | High | E1, E2 | 主要基于当前 4+4 formation 场景。 |
| C2 | AA-existence + KLA-spatial hybrid 能保留 AA 的 existence/cardinality 优势，同时用 KLA spatial mode intersection 压制孤立 modes。 | High | E3, E4, E5, E7 | 仍是 heuristic hybrid，不是 strict-AA。 |
| C3 | Tuned spatial-KLA AA 在 N50 上优于两个 GA mode 的 OSPA、cardinality、local E-OSPA、hOspa、local RMSE 和 local CardErr。 | High | E5, E6 | consensus Loc 仍略弱于 GA Balanced。 |
| C4 | 当前 `spatialStructureStrength=0.75` 和 `spatialDecouplingStrength=1.0` 相对基础 spatial-KLA hybrid 只带来小幅 OSPA/Loc/Card 改善。 | High | E7, E8 | tuned local E-OSPA/RMSE/CardErr 略差于基础 spatial-KLA。 |
| C5 | FID-FIA existence refinement 不适合当前 AA-existence + KLA-spatial hybrid；N50 ablation 中它显著破坏 Loc、Card 和 local tracking。 | High | E7, E8 | 该结论限于当前 FID-FIA 参数和 24-step diagnostic scenario。 |

## 定位

当前方法不是严格 Bernoulli-AA 的替代定义，而是一个面向当前 distributed LMB 场景的 heuristic hybrid:

- existence branch: 保留 AA 的 Bernoulli existence 线性平均，用来利用 AA 在 first-moment / missed-detection robustness 上的优势。
- spatial branch: 改用 KLA/GA 单 Gaussian 融合，用几何池完成 spatial mode intersection，压制 partial-FOV、false alarm 和 clutter 带来的孤立 spatial modes。
- weight branch: 默认使用 branch-decoupled AA weights；strict-AA 模式保留为 regression 和理论边界检查，而不是当前主结果。

## 算法

对目标 `i`、传感器 `s=1..S`，local LMB Bernoulli 为 `(r_s^i, p_s^i(x))`。

1. 计算动态质量分数:
   - covariance / concentration score；
   - realized link quality；
   - existence confidence；
   - optional structure prior；
   - optional FID-FIA existence score。
2. 得到两组归一化权重:
   - `alpha_x^i`: spatial branch weights；
   - `alpha_r^i`: existence branch weights。
3. existence branch 使用 AA:
   - `r^i = sum_s alpha_r^i r_s^i`。
4. spatial branch 使用 KLA:
   - 先把每个 local mixture moment-match 为 `(mu_s^i, Sigma_s^i)`；
   - `Sigma_i^{-1} = sum_s alpha_x^i (Sigma_s^i)^{-1}`；
   - `Sigma_i^{-1} mu_i = sum_s alpha_x^i (Sigma_s^i)^{-1} mu_s^i`。
5. 输出 LMB Bernoulli:
   - existence 为 AA 结果；
   - spatial density 为一个 KLA-fused Gaussian；
   - 低 existence Bernoulli 由 `existenceThreshold=0.18` pruning。

当前 tuned N50 配置:

```matlab
existenceThreshold = 0.18;
aaSpatialFusionMode = 'kla';
emaAlpha = 0.0;
minWeight = 0.0;
spatialEmaAlpha = 0.0;
existenceEmaAlpha = 0.0;
spatialMinWeight = 0.0;
existenceMinWeight = 0.0;
spatialDecouplingStrength = 1.0;
existenceDecouplingStrength = 0.15;
spatialStructureStrength = 0.75;
existenceStructureStrength = 0.08;
```

## 理论边界

严格 Bernoulli-AA 要求同一组权重同时进入 existence 和 spatial density:

```text
f_AA(X) = sum_s alpha_s f_s(X)
```

对 Bernoulli 分布，这会给出 `r = sum_s alpha_s r_s`，且 spatial density 与 `alpha_s r_s p_s(x)` 成比例。该形式的主要意义是 conservative linear opinion pool；在未知互相关时不会像几何池那样过度自信。

当前 hybrid 刻意不沿用严格 AA 的 spatial density，因为本仓库的输出链路最终会做 MAP cardinality 和 top-Gaussian selection。纯 AA spatial mixture 会保留 component union；在 partial-FOV 和 false-alarm regime 下，这些被保留的 modes 很容易在不同 sensor neighborhood 中被不同地选为 top component，表现为高 consensus Loc disagreement。

KLA spatial branch 的作用是做 mode intersection:

```text
p_KLA(x) proportional to prod_s p_s(x) ^ alpha_x_s
```

在 Gaussian moment-matched 近似下，它等价于 precision 加权平均。它不是 AA 的 conservative spatial union，而是用共同支持压制孤立 spatial mode。因此该 hybrid 的理论说法应写成:

> AA 用于 Bernoulli existence / first moment 的保守融合；KLA 用于 spatial density 的 mode intersection；branch-decoupled weights 是工程启发式，用来把两类风险分开建模。

不要写成:

> 这是严格 AA-LMB 的闭式最优形式。

## 当前证据

N10 上，Tuned spatial-KLA AA 同时优于 GA Balanced 和 GA final 的全部记录指标:

| Method | Trials | OSPA | Loc | Card | E-OSPA | RMSE | CardErr |
| --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| GA Balanced | 10 | 1.732144 | 1.468054 | 0.083000 | 2.197304 | 4.223026 | 0.212000 |
| GA final | 10 | 1.802067 | 1.825416 | 0.075500 | 2.133102 | 4.511292 | 0.149500 |
| Tuned spatial-KLA AA | 10 | 1.663109 | 1.456257 | 0.035000 | 1.9986 | 3.7166 | 0.084000 |

N50 上，Tuned spatial-KLA AA 已经压过 GA final，并在除 consensus Loc 外的全部指标上压过 GA Balanced:

| Method | Trials | OSPA | Loc | Card | E-OSPA | hOspa | RMSE | CardErr |
| --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| GA Balanced | 50 | 1.761137 | 1.462915 | 0.084125 | 2.2133 | 0.4869 | 4.2284 | 0.2040 |
| GA final | 50 | 1.847668 | 1.958378 | 0.081275 | 2.1799 | 0.4895 | 4.6951 | 0.1510 |
| Tuned spatial-KLA AA | 50 | 1.682607 | 1.472837 | 0.035350 | 2.029641 | 0.4856 | 3.682880 | 0.088000 |

因此当前设计是可行的，但还不是“全指标严格胜出”的最终形态。N50 Loc gap 很小，主要来自少数 seed block；后续要用 effective graph diagnostics 找出这些 realization 下哪些 target 的 spatial branch 选错了 sensor set。

## N50 ablation 结果

N50 ablation 固定同一 seed schedule、`existenceThreshold=0.18` 和 24-step diagnostic scenario，比较四个 arms:

| Arm | OSPA | Loc | Card | E-OSPA | hOspa | RMSE | CardErr |
| --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| Balanced AA | 3.422478 | 4.237960 | 0.116775 | 3.641417 | 0.4833 | 4.396212 | 0.221925 |
| Balanced spatial-KLA AA | 1.686615 | 1.476160 | 0.035375 | 2.024428 | 0.4856 | 3.678611 | 0.087925 |
| Cardinality spatial-KLA AA | 2.329792 | 6.549674 | 0.209250 | 2.442527 | 0.4934 | 5.542318 | 0.314250 |
| Tuned spatial-KLA AA | 1.682607 | 1.472837 | 0.035350 | 2.029641 | 0.4856 | 3.682880 | 0.088000 |

结论:

- `Balanced AA -> Balanced spatial-KLA AA` 是决定性变化: OSPA 从 `3.422478` 降到 `1.686615`，Loc 从 `4.237960` 降到 `1.476160`，Card 从 `0.116775` 降到 `0.035375`。这证明 pure AA spatial mixture 是主失败源，spatial-KLA 是当前设计的核心增益。
- `Cardinality spatial-KLA AA` 失败: OSPA `2.329792`、Loc `6.549674`、Card `0.209250`，local metrics 也显著变差。当前 FID-FIA existence refinement 不应放进 AA-existence + KLA-spatial 主线。
- `Tuned spatial-KLA AA` 相对 `Balanced spatial-KLA AA` 的收益很小但方向一致: OSPA 改善 `0.004008`，Loc 改善 `0.003323`，Card 改善 `0.000025`。代价是 local E-OSPA/RMSE/CardErr 略差。因此 tuned 配置可以作为当前 N50 主配置，但需要把它描述为小幅 retuning，而不是新的主机制。
- 与 GA Balanced 相比，tuned 配置仍有 consensus Loc gap: `1.472837` 对 `1.462915`。当前证据支持“AA-existence + KLA-spatial hybrid 是合理且整体更优的方向”，但不支持“所有指标严格优于 GA Balanced”。

## Evidence Ledger

| ID | Type | Artifact | What it supports |
| --- | --- | --- | --- |
| E1 | experiment | `RUN/AA/AA_BALANCED_CARDINALITY_VALIDATION_N1_SEED1_20260621_015912.md` | pure AA baseline spatial disagreement |
| E2 | experiment | `RUN/AA/AA_BALANCED_CARDINALITY_VALIDATION_N1_SEED1_20260621_022111.md` | pure AA maxGM/pruning probe |
| E3 | experiment | `RUN/AA/AA_BALANCED_CARDINALITY_VALIDATION_N1_SEED1_20260621_022944.md` | first spatial-KLA AA hybrid evidence |
| E4 | experiment | `RUN/AA/AA_BALANCED_CARDINALITY_VALIDATION_N10_SEED1_20260621_150248.md` | tuned spatial-KLA AA N10 all-metric win |
| E5 | experiment | `RUN/AA/AA_BALANCED_CARDINALITY_VALIDATION_N50_SEED1_20260621_183039.md` | tuned spatial-KLA AA N50 result |
| E6 | experiment | `RUN/GA/GA_TIERED_LINK_ABLATION_N50_SEED1_20260621_183039.md` | GA Balanced/final N50 reference |
| E7 | experiment | `RUN/AA/AA_BALANCED_CARDINALITY_VALIDATION_N50_SEED1_20260621_215745.md` | N50 design ablation report |
| E8 | code | `multisensorLmb/aaLmbTrackMerging.m` | strict-AA and spatial-KLA AA consumption path |
| E9 | command log | `RUN/AA/AA_TUNED_DESIGN_ABLATION_N50_20260621_215745.log` | N50 design ablation command log |

## Verification Record

Independence status: self-check only. 本文档由当前分支代码、N10/N50 reports 和 block-screen logs 汇总，没有独立 reviewer。

已验证:

- `git diff --check` 通过。
- `docs/AA_BASELINE_FAILURE_DIAGNOSTIC_CN.md` 通过 evidence lint。
- `docs/AA_EXISTENCE_KLA_SPATIAL_DESIGN_CN.md` 通过 evidence lint。

待验证:

- 若要形成 paper-facing all-metric claim，需要继续解决 N50 consensus Loc gap。

## Risk and Escalation

主要风险是把 heuristic hybrid 写成 strict-AA 理论结果，或把 N50 的 near-win 写成全指标严格胜出。后续论文表述应保留三个边界:

- branch-decoupled weights 是 heuristic；
- spatial branch 的 KLA 是针对 mode intersection 的工程选择；
- N50 consensus Loc 尚需 effective graph guard 或 regime-aware switching 进一步处理。

N50 ablation 显示 tuned spatial structure 有小幅 OSPA/Loc/Card 收益，但 local E-OSPA/RMSE/CardErr 略差。当前可以保留 Tuned spatial-KLA AA 作为主配置，同时在论文或报告中把基础 spatial-KLA hybrid 作为紧邻 ablation，对收益幅度保持克制。

## Reproducibility

核心命令:

```bash
octave --quiet --eval "test_aa_lmb_track_merging"
sh RUN/AA/launchAaTunedHybridVsGaN50.sh
sh RUN/AA/launchAaTunedDesignAblationN50.sh
```

N50 ablation 入口固定 `numberOfTrials=50`、`baseSeed=1`、`existenceThreshold=0.18`、arms `[3 7 8 9]`。

## Open Issues

- consensus Loc 尚未在 N50 上严格优于 GA Balanced。
- 当前 branch-decoupled AA 是 heuristic；论文中必须明确命名，不能把它等同于 strict-AA。
- 需要记录 target-wise effective graph: 每个 Bernoulli 在 spatial/existence branch 中实际使用的 sensor set、weight entropy、mode count 和 pruning 前后 existence。
- 若要形成强 paper claim，需要补一个 regime-aware guard: 当 target-wise spatial graph 的共同支持不足时回退到更保守的 spatial selection 或提高 pruning。

## Recommendation

把当前方法作为 `heuristic AA-existence + KLA-spatial hybrid` 继续推进，而不是继续追求 pure AA 在当前 false-alarm / partial-FOV regime 下直接击败 GA。N50 ablation 已经支持该设计的主机制: spatial-KLA 是核心，FID-FIA existence refinement 应排除，tuned spatial structure 只是小幅 retuning。下一步研究点不是再做全局参数 sweep，而是针对 N50 Loc gap 做 target-wise effective graph guard 或 regime-aware switching。
