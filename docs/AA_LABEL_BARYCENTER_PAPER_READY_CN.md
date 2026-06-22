# AA Label-Barycenter Fusion 论文写作准备包

日期: 2026-06-22

## Question

如何把当前 AA Fusion 工作整理成 ready for paper writing 的状态: 明确问题动机、方法定义、理论推导、实验结果、claim 边界和后续写作入口，使它可以直接拆成论文的 Introduction、Method、Theory、Experiments 和 Limitations。

## Scope

包含:

- 分支 `codex/aa-target-wise-fix` 上已经实现并验证的 AA target-wise weight consumer fix、centralized cross-local label-consensus projection、neighborhood label-barycenter projection 和 reference-only ablation。
- 主要代码路径: `multisensorLmb/aaLmbTrackMerging.m`、`multisensorLmb/computeAdaptiveFusionWeights.m`、`multisensorLmb/applyCrossLocalLabelConsensusProjection.m`、`multisensorLmb/runDistributedLmbFilter.m`、`RUN/AA/runAaBalancedCardinalityValidation.m`。
- 主要证据: AA N1/N5/N50 sanity、N50 neighborhood validation、centralized/reference-only ablation、GA N50 reference 对照。
- 论文写作层面的 contribution、method narrative、理论性质、图表计划和风险边界。

排除:

- 不把本文档当作最终投稿正文；它是写作前的技术包。
- 不虚构外部 citation key；正式论文引用需要再从 Zotero 或 `docs/paper/els-cas-templates/paper-refs.bib` 核对。
- 不把 centralized projection 的 consensus=0 写成 distributed online fusion 的性能结论。
- 不声明当前 output-level post-pass 已经是递归滤波内部的最终 online AA 算法。

## Risk Tier

L2。该文档会直接影响论文叙事和下一步算法实现，但当前仍是 self-check research branch 的写作准备包；方法证据来自本地代码、报告和同一 worker lane 的验证，尚未经过 independent verifier。

## Paper-Ready Positioning

候选英文标题:

```text
Neighborhood Label-Barycenter Average Fusion for Distributed LMB Tracking under Unreliable Communication
```

候选中文标题:

```text
面向不可靠通信分布式 LMB 跟踪的邻域标签重心平均融合
```

一句话 contribution:

```text
本文把 AA-existence + KLA-spatial hybrid 的剩余误差从阈值和权重调参问题重写为跨 local filters 的 label canonicalization 与 matched posterior barycenter 问题，并提出一个基于邻域迭代的 label-barycenter operator，在不读取全局 label set 的条件下同时降低跨节点 disagreement 与 truth-referenced local tracking error。
```

建议主线:

1. 失败诊断: AA consumer-side target-wise weights 修复后，单纯继续调 `existenceThreshold`、support count 或 bridge prior 不能解释剩余 Loc gap；failure block 同时包含 label-set split 和 same-label spatial spread。
2. 方法转向: 把目标从场景阈值搜索改为结构化 operator: reference label set selection、Hungarian label canonicalization、matched posterior moment barycenter、neighborhood iterative propagation。
3. 论文主方法: `Neighborhood label-barycenter spatial-KLA AA`。它只使用 `neighborMap{s}` 中的 local outputs，迭代 `crossLocalConsensusIterations=3` 轮。
4. upper-bound 诊断: `Cross-local label-consensus spatial-KLA AA`。它全局读取所有 local outputs 并把同一结果写给所有 sensors，因此 consensus 指标归零是构造性质，只能用作 upper-bound 和 ablation 诊断。
5. 必要组件证据: reference-only ablation 使用同一 reference selection 和 label canonicalization，但不做 matched posterior barycenter；N50 显示它显著弱于 full barycenter，说明收益不是简单复制 medoid output。

可以写成论文的核心贡献:

| Contribution | Paper wording | Evidence |
| --- | --- | --- |
| Ctr-1 failure diagnosis | AA-LMB 的剩余误差主要来自跨 local filters 的 label-space misalignment 与 matched states dispersion，而不是单个 threshold 的失配。 | `docs/AA_LABEL_UNCERTAINTY_AWARE_FUSION_RULE_CN.md`, `docs/AA_BASELINE_FAILURE_DIAGNOSTIC_CN.md` |
| Ctr-2 method | 提出 neighborhood label-barycenter AA: 每个节点在邻域内选 median-cardinality medoid reference，匹配邻居 estimates，并对 matched posteriors 做 moment barycenter。 | `multisensorLmb/applyCrossLocalLabelConsensusProjection.m` |
| Ctr-3 theory | 证明 reference cardinality 的 median robustness、matched mixture 的 moment projection、稳定匹配条件下的 distributed moment consensus 收敛，以及 AA existence pooling 的 convex-hull boundedness。 | `docs/AA_LABEL_BARYCENTER_THEORY_CN.md` |
| Ctr-4 evidence | N50 paired validation 显示 neighborhood full 同时改善 consensus disagreement 和 local E-OSPA/RMSE/CardErr；reference-only ablation 隔离出 barycenter 的空间收益。 | `RUN/AA/AA_BALANCED_CARDINALITY_VALIDATION_N50_SEED1_20260622_174819.md` |

## Method

### Problem Formulation

在时刻 `k`，sensor `s` 的 LMB estimate 输出为:

```text
X_s(k) = { (ell_{s,i}, mu_{s,i}, Sigma_{s,i}) }_{i=1..n_s(k)}
```

其中 `ell` 是 label，`mu` 和 `Sigma` 是 moment-matched Gaussian state。传统 AA existence branch 可以线性 pool existence probability；但如果不同 local filters 对同一物理目标给出不同 labels，或者相同 label 下的 spatial states 分散，后续 average fusion 会出现两个问题:

- label-set split: 多个 local filters 维护不同 label identities，导致 network outputs 互相不一致。
- same-label spatial spread: label 看似一致但 states 分散，KLA/AA spatial update 会留下 Loc gap 或过度自信。

因此本文的方法不继续搜索固定阈值，而是定义一个 label-aligned posterior barycenter operator。

### Centralized Diagnostic Operator

Centralized 版本用于 upper bound 和 ablation。对每个时刻 `k`:

1. 计算所有 local outputs 的 cardinality `n_s(k)`，取 median cardinality。
2. 只在 cardinality 最接近 median 的 sensors 中选 reference candidates。
3. 对每个 candidate 计算它到其他 local outputs 的平均 set distance，选择 medoid reference。
4. 用 Hungarian assignment 将每个 local output 匹配到 reference label set。
5. 对每个 reference label 的 matched states 做 moment barycenter:

```text
mu_bar = (1/m) * sum_i mu_i
Sigma_bar = (1/m) * sum_i [Sigma_i + (mu_i - mu_bar)(mu_i - mu_bar)']
```

6. 把同一组 `(L_ref, mu_bar, Sigma_bar)` 写回所有 sensors 的 output estimate。

该版本的价值是证明 label canonicalization + posterior barycenter 的潜在上界，并通过 reference-only ablation 判断 barycenter 是否真的贡献 spatial improvement。

### Neighborhood Label-Barycenter Operator

论文主方法应以 neighborhood 版本为主。每个 sensor `s` 只使用通信邻域 `N_s = neighborMap{s}` 中的 outputs:

```text
Input: {X_j(k): j in N_s}, iterations H
for h = 1..H:
  for each sensor s:
    choose reference sensor r_s from N_s by median-cardinality medoid
    set L_ref,s = labels(X_{r_s})
    for each neighbor j in N_s:
      match X_j to L_ref,s by Hungarian assignment
    for each reference label ell in L_ref,s:
      collect matched states G_{s,ell}
      compute moment barycenter (mu_bar,s,ell, Sigma_bar,s,ell)
    output Y_s(k) = {(ell, mu_bar,s,ell, Sigma_bar,s,ell)}
  replace X_s(k) by Y_s(k)
```

当前 prototype 是 output-level iterative operator，而不是递归 filter 内部的 message update。它仍然重要，因为它已经把 centralized global read/write 限制为 graph-local read/write，并允许多跳信息通过 H 轮邻域迭代传播。

### Relation to AA and KLA

当前 AA branch 仍保留 existence 的 arithmetic pooling 语义；spatial branch 使用 KLA 或 label-aligned moment barycenter。写作时可以把方法定位为:

- existence side: AA-compatible linear pooling，保持 `[0,1]` convex-hull boundedness。
- spatial side: label-aligned barycenter/projection layer，用于修复 AA-LMB output label misalignment 和 same-label spatial spread。
- branch relationship: 这是 heuristic/branch-decoupled AA family，而不是严格单一权重同时进入 existence 和 spatial branch 的 strict-AA closed form。

若后续要强化理论纯度，可以新增 strict-AA mode；当前 paper claim 更适合写成 method-level label-barycenter average fusion，而不是严格 Bernoulli-AA 最优解。

## Theory Derivation

### Proposition 1: Centralized Projection Guarantees Output Consensus

命题: 若 projection 在时刻 `k` 输出同一组 estimate `Y(k)` 给所有 sensors，则任意满足 `d(Y,Y)=0` 的 pairwise output disagreement metric 在该时刻为 0。

证明: projection 最后一步把每个 sensor 的 labels、means 和 covariances 都替换为同一个 `Y(k)`。因此任意 pair `(a,b)` 有 `Y_a(k)=Y_b(k)`，由 metric identity 得 `d(Y_a(k),Y_b(k))=0`。对所有 sensor pairs 求均值仍为 0。

论文边界: 这只解释 centralized diagnostic 的 consensus=0；不能据此声称 truth-referenced tracking error 必然下降。

### Proposition 2: Median-Cardinality Medoid Has Majority Robustness

命题: 若超过半数 local filters 在时刻 `k` 输出同一 cardinality `n*`，则 reference selection 的 candidate cardinality 必为 `n*`。

证明: 一维样本中严格多数值为 `n*` 时，median 为 `n*`。实现先选择 `abs(n_s - median(n))` 最小的 sensors，因此距离最小值为 0，且只由 `n_s=n*` 的 sensors 达到。后续 medoid 只在这些 sensors 中选择，所以 reference cardinality 为 `n*`。

论文边界: 该性质不保证 label identity 正确，只保证 reference cardinality 不会被少数 cardinality outliers 直接拖走。

### Proposition 3: Matched Moment Barycenter Is the First-Two-Moment Projection of the Empirical Mixture

给定 reference label `ell` 的 matched posterior group:

```text
G_ell = { (mu_i, Sigma_i) }_{i=1..m}
```

定义:

```text
mu_bar = (1/m) * sum_i mu_i
Sigma_bar = (1/m) * sum_i [Sigma_i + (mu_i - mu_bar)(mu_i - mu_bar)']
```

性质 3a: `mu_bar` 是最小化 `sum_i ||z - mu_i||^2` 的唯一解。

证明: 对 `J(z)=sum_i ||z-mu_i||^2` 求导，得到 `2m z - 2 sum_i mu_i = 0`，因此 `z=mu_bar`。Hessian 为 `2m I`，正定。

性质 3b: `(mu_bar, Sigma_bar)` 是等权 Gaussian mixture `(1/m) sum_i N(mu_i,Sigma_i)` 的前两阶矩。

证明: 一阶矩为 `E[x]=(1/m)sum_i mu_i`。由 total covariance decomposition:

```text
Cov[x] = E_i[Cov[x|i]] + Cov_i(E[x|i])
       = (1/m) * sum_i Sigma_i
         + (1/m) * sum_i (mu_i - mu_bar)(mu_i - mu_bar)'
```

这正是实现中的 covariance formula。

论文边界: 这是 AA/moment-barycenter 解释，不是 covariance intersection 或 KLA 的保守 unknown-correlation 证明。若主审关注 covariance consistency，需要补 NEES/NIS 或 conservative barycenter ablation。

### Proposition 4: Stable Distributed Moment Consensus Converges to Centralized Moment Barycenter

假设在一个短时间窗口内 label matching 固定且正确，通信图连通，权重矩阵 `W` primitive 且 doubly-stochastic。令每个 node 对 matched label 维护 moment vector:

```text
q_s = (r_s, mu_s, M2_s),   M2_s = Sigma_s + mu_s mu_s'
```

执行:

```text
q_s^{h+1} = sum_j W_{sj} q_j^h
```

则 `q_s^h` 对所有 nodes 收敛到 `(1/S) sum_s q_s^0`。

证明: average consensus 标准结论给出 `W^h -> (1/S) 11'`。堆叠 moment vectors 后有 `q^h = W^h q^0`，因此每个 node 的极限是全局平均。由于 `q` 包含一阶矩和二阶矩，极限对应 centralized moment barycenter 的 moment statistics。

论文边界: 当前实现还不是该 recursive message form；它是 output-level neighborhood iteration。该命题可作为下一版 online method 的理论目标，而不是当前 prototype 的完整收敛证明。

### Proposition 5: AA Existence Pooling Preserves Convex-Hull Boundedness

若 `r_j in [0,1]`、`W_{sj} >= 0` 且 `sum_j W_{sj}=1`，则:

```text
r_s^+ = sum_j W_{sj} r_j
```

满足 `r_s^+ in [min_j r_j, max_j r_j]`。

证明: `r_s^+` 是 `r_j` 的凸组合，必落在输入值的 convex hull 内。

论文含义: existence branch 可以保持 AA 的 conservative linear-pool 解释；方法创新主要发生在 label alignment 和 matched spatial posterior barycenter。

## Claims

| ID | Claim | Confidence | Evidence IDs | Writing boundary |
| --- | --- | --- | --- | --- |
| C1 | AA-LMB 剩余误差应被表述为 label canonicalization + matched posterior barycenter 问题，而不是继续调 threshold 的问题。 | High | E1, E2 | 不把所有负结果写成形式化定理，只写成 failure diagnosis 和 design motivation。 |
| C2 | Neighborhood label-barycenter AA 是当前最适合进入论文的主方法候选。 | High | E3, E4, E7 | 明确它是 output-level iterative prototype，递归 online 化仍需后续实现。 |
| C3 | Centralized cross-local projection 可作为 upper bound 和 method diagnostic；其 consensus=0 是构造性质。 | High | E5, E6 | 不把 centralized consensus=0 当主结果。 |
| C4 | Reference-only ablation 显示收益不只是复制 medoid output，matched posterior barycenter 对 local E-OSPA/RMSE 有必要贡献。 | High | E4, E5 | CardErr 持平，说明该 ablation 主要隔离 spatial/posterior averaging。 |
| C5 | N50 neighborhood validation 显示 full method 相对 tuned AA 显著改善 consensus disagreement 和 local tracking metrics。 | High | E4 | 当前是 paired deterministic trials，仍需 independent verifier 或独立复跑。 |
| C6 | 与 GA references 对比时，Neighborhood label-barycenter AA 在 local E-OSPA/RMSE/CardErr 上均低于两个 GA reference rows。 | Medium-High | E4, E8 | AA 和 GA 来自相邻但不同 validation scripts；正式论文需统一 experimental table 的描述口径。 |
| C7 | 理论可支撑的部分是结构性质、moment projection 和稳定匹配下的 consensus target；性能提升仍由实验支持。 | High | E6, E9 | 不写成 truth-error 必然下降的 theorem。 |

## Experimental Results

### Main N50 Comparison

所有数值为 50 deterministic paired trials, seeds `2`--`51`。AA neighborhood rows 来自 `RUN/AA/AA_BALANCED_CARDINALITY_VALIDATION_N50_SEED1_20260622_174819.md`；GA references 来自 `RUN/GA/GA_TIERED_LINK_ABLATION_N50_SEED1_20260621_183039.md`。

| Arm | Consensus OSPA | Loc. disag. | Card. disp. | Local E-OSPA | Local RMSE | Local CardErr | Runtime note |
| --- | ---: | ---: | ---: | ---: | ---: | ---: | --- |
| GA +structure-aware decoupled KLA | 1.761137 | 1.462915 | 0.084125 | 2.213284 | 4.228361 | 0.203975 | GA script, 56.44s |
| GA +FID-FIA existence refinement | 1.847668 | 1.958378 | 0.081275 | 2.179948 | 4.695098 | 0.151025 | GA script, 125.59s |
| Tuned spatial-KLA AA | 1.682607 | 1.472837 | 0.035350 | 2.029641 | 3.682880 | 0.088000 | AA script, 62.25s |
| Neighborhood label-barycenter AA | 0.309818 | 0.216372 | 0.021200 | 1.681483 | 3.449035 | 0.077200 | AA script, 102.35s, 1.647x tuned |
| Neighborhood reference-only AA | 1.030062 | 0.891585 | 0.021200 | 1.911286 | 3.662955 | 0.077200 | AA script, 95.58s, 1.542x tuned |

Interpretation:

- Neighborhood full 相对 tuned AA 把 network OSPA disagreement 从 `1.682607` 降到 `0.309818`，local E-OSPA 从 `2.029641` 降到 `1.681483`，RMSE 从 `3.682880` 降到 `3.449035`，CardErr 从 `0.088000` 降到 `0.077200`。
- Neighborhood reference-only 的 CardErr 与 full 相同，但 E-OSPA 和 RMSE 明显弱于 full，说明 label canonicalization 可以修 cardinality/support 一部分，state barycenter 是 spatial improvement 的关键。
- Neighborhood full 的 runtime 为 tuned AA 的 `1.647x`。这比 GA FID-FIA 的 `2.232x` GA reference 低，但与 GA runtime 不是同一脚本直接 paired denominator；论文中应分开报告或重新统一计时。

### Paired Improvements Relative to Tuned AA

| Arm | Metric | Reduction | Wins | Sign-test p |
| --- | --- | ---: | ---: | ---: |
| Neighborhood label-barycenter AA | OSPA | 81.59% | 50/50 | 1.776e-15 |
| Neighborhood label-barycenter AA | Loc. disag. | 85.31% | 50/50 | 1.776e-15 |
| Neighborhood label-barycenter AA | Card. disp. | 40.03% | 50/50 | 1.776e-15 |
| Neighborhood label-barycenter AA | E-OSPA | 17.15% | 50/50 | 1.776e-15 |
| Neighborhood label-barycenter AA | RMSE | 6.35% | 48/50 | 2.267e-12 |
| Neighborhood label-barycenter AA | CardErr | 12.27% | 46/50 | 6.981e-11 |
| Neighborhood reference-only AA | RMSE | 0.54% | 27/50 | 0.6718 |

Interpretation:

- Full method 在全部 consensus metrics 和 local E-OSPA 上 50/50 wins；RMSE 也有 48/50 wins。
- Reference-only 的 RMSE reduction 不显著，说明 barycenter 不是可省略的装饰组件。

### Centralized Upper-Bound Ablation

| Arm | Consensus OSPA | Loc. disag. | Card. disp. | Local E-OSPA | Local RMSE | Local CardErr |
| --- | ---: | ---: | ---: | ---: | ---: | ---: |
| Cross-local label-consensus AA | 0.000000 | 0.000000 | 0.000000 | 1.645476 | 3.343985 | 0.071200 |
| Reference-only label-consensus AA | 0.000000 | 0.000000 | 0.000000 | 1.781249 | 3.463664 | 0.071200 |

Interpretation:

- 两个 centralized rows 的 consensus 全为 0，是输出同一 estimate 的构造结果。
- Full barycenter 相对 reference-only 在 local E-OSPA 上好 `8.25%`，RMSE 好 `3.58%`，CardErr 持平。
- 这个表适合放在 appendix 或 method-diagnostic subsection，用来证明 barycenter 组件必要；不适合作为 main performance headline。

### N1/N5 Sanity

| Setting | Tuned AA local E-OSPA/RMSE/CardErr | Neighborhood full | Neighborhood reference-only |
| --- | --- | --- | --- |
| N5 seed11 | 2.032799 / 3.588145 / 0.086250 | 1.691451 / 3.450998 / 0.073000 | 1.918293 / 3.716909 / 0.073000 |
| N50 seed1 | 2.029641 / 3.682880 / 0.088000 | 1.681483 / 3.449035 / 0.077200 | 1.911286 / 3.662955 / 0.077200 |

Interpretation: N5 和 N50 的方向一致，降低了单一 N50 seed block 偶然性的风险；但仍建议 independent verifier 或独立复跑。

## Paper Writing Assets

### Abstract Skeleton

```text
Distributed LMB tracking under heterogeneous packet loss suffers not only from sensor reliability imbalance but also from cross-node label misalignment. We diagnose a residual failure mode of AA-LMB fusion where local filters maintain split label sets or spatially dispersed states for matched labels. To address this, we propose a neighborhood label-barycenter average-fusion operator. Each node selects a median-cardinality medoid reference in its communication neighborhood, aligns neighboring estimates by Hungarian matching, and replaces matched posteriors with a moment barycenter. We show that the operator preserves AA existence boundedness, admits a moment-projection interpretation, and converges to the centralized moment barycenter under stable matching and average-consensus assumptions. In 50 paired distributed LMB trials under tiered packet loss, the neighborhood label-barycenter operator reduces network OSPA disagreement by 81.59% and local E-OSPA by 17.15% relative to the tuned AA baseline, while a reference-only ablation shows that spatial gains require the barycenter component.
```

正式英文摘要需要补两点: 引用背景与更精确地说明 output-level prototype / online method 的边界。

### Introduction Logic

1. Distributed LMB fusion under packet loss needs more than scalar reliability weighting.
2. GA/KLA adaptive weighting gives conservative and useful baselines, but AA-LMB has a different failure mode because label support and spatial posteriors can split across local filters.
3. Empirical diagnosis shows threshold or bridge-prior search is fragile; the structural issue is label-space canonicalization.
4. Proposed solution: neighborhood label-barycenter average fusion.
5. Contributions: diagnosis, method, theory, paired evidence and ablation.

### Method Section Outline

1. Distributed LMB output notation.
2. Failure mode: label-set split and same-label spatial spread.
3. Centralized diagnostic projection.
4. Neighborhood label-barycenter operator.
5. Reference-only ablation definition.
6. Complexity: Hungarian matching per neighborhood per time step, H iterations; current N50 runtime `1.647x` tuned AA.

### Theory Section Outline

1. Output consensus identity for centralized diagnostic.
2. Median-cardinality medoid robustness.
3. Moment barycenter as empirical mixture moment projection.
4. Stable distributed moment consensus target.
5. AA existence convex-hull boundedness.
6. Boundary paragraph: structural properties do not prove truth-error monotonicity.

### Experiments Section Outline

1. Scenario: 8 sensors, two 4-sensor formations, 100-step tiered heterogeneous packet loss, fixed pDrop levels `[0, 0.1, 0.2, 0.5]`, trials seeds `2`--`51`.
2. Baselines: tuned spatial-KLA AA, GA structure-aware decoupled KLA, GA FID-FIA existence refinement.
3. Main result: N50 neighborhood label-barycenter vs tuned AA and GA references.
4. Ablation: neighborhood full vs neighborhood reference-only; centralized upper-bound full vs reference-only.
5. Runtime and limitations.

### Figure and Table Plan

| Asset | Content | Source |
| --- | --- | --- |
| Figure 1 | Pipeline: local outputs -> neighborhood reference selection -> Hungarian matching -> moment barycenter -> local consensus output. | Can be generated from `docs/AA_LABEL_BARYCENTER_THEORY_CN.md` algorithm. |
| Figure 2 | Failure mode schematic: label-set split vs same-label spatial spread. | Use diagnostic examples from AA reports if available. |
| Table 1 | N50 main comparison: GA references, tuned AA, neighborhood full, reference-only. | `RUN/AA/AA_BALANCED_CARDINALITY_VALIDATION_N50_SEED1_20260622_174819.md`, `RUN/GA/GA_TIERED_LINK_ABLATION_N50_SEED1_20260621_183039.md` |
| Table 2 | Paired reductions and sign-test p values. | Same AA N50 report. |
| Table 3 | Centralized upper-bound and reference-only ablation. | `RUN/AA/AA_BALANCED_CARDINALITY_VALIDATION_N50_SEED1_20260622_145052.md` |
| Proposition box | Moment barycenter and stable consensus properties. | This document and `docs/AA_LABEL_BARYCENTER_THEORY_CN.md` |

## Evidence Ledger

| ID | Type | Source or artifact | Supports |
| --- | --- | --- | --- |
| E1 | design doc | `docs/AA_LABEL_UNCERTAINTY_AWARE_FUSION_RULE_CN.md` | Failure diagnosis; move away from threshold/search tuning. |
| E2 | progress doc | `docs/AA_RESEARCH_PROGRESS_CN.md` | Current goal status, N1/N5/N50 checkpoint summary. |
| E3 | code | `multisensorLmb/applyCrossLocalLabelConsensusProjection.m` | Centralized and neighborhood projection implementation. |
| E4 | experiment | `RUN/AA/AA_BALANCED_CARDINALITY_VALIDATION_N50_SEED1_20260622_174819.md` | Main N50 neighborhood result and paired reductions. |
| E5 | experiment | `RUN/AA/AA_BALANCED_CARDINALITY_VALIDATION_N50_SEED1_20260622_145052.md` | Centralized full vs reference-only ablation. |
| E6 | theory doc | `docs/AA_LABEL_BARYCENTER_THEORY_CN.md` | Propositions, evidence package, limitations. |
| E7 | regression | `test_cross_local_label_consensus_projection.m` and command `octave --quiet --eval "test_cross_local_label_consensus_projection"` | Projection behavior, reference-only, neighborhood-local regression. |
| E8 | GA reference | `RUN/GA/GA_TIERED_LINK_ABLATION_N50_SEED1_20260621_183039.md` | GA comparison rows. |
| E9 | code hook | `multisensorLmb/runDistributedLmbFilter.m` | Projection hook is currently output-level after local estimates are produced. |

## Verification Record

Independence status: self-check only. 本文档由同一 worker lane 基于当前代码、已有报告和理论文档整理，尚未经过 independent verifier。

已检查:

- `git status --short --branch` 确认当前分支为 `codex/aa-target-wise-fix`，tracked workspace 在写入本文档前无未提交改动；存在多份未跟踪历史日志，本文档不依赖它们作为新增提交内容。
- `docs/AA_RESEARCH_PROGRESS_CN.md` 确认当前推荐方向是 label canonicalization + posterior barycenter，而不是继续搜索 `existenceThreshold` 或 bridge prior。
- `multisensorLmb/applyCrossLocalLabelConsensusProjection.m` 确认 neighborhood modes 只通过 `neighborMap{s}` 选择 localSensorIdx，并支持 `neighborhood-barycenter` 与 `neighborhood-reference-only`。
- `test_cross_local_label_consensus_projection.m` 确认 regression 覆盖 centralized barycenter、reference-only 和 neighborhood-locality。
- `RUN/AA/AA_BALANCED_CARDINALITY_VALIDATION_N50_SEED1_20260622_174819.md` 确认 N50 neighborhood full、reference-only 和 tuned AA 的 summary rows、paired reductions 和 runtime。
- `RUN/AA/AA_BALANCED_CARDINALITY_VALIDATION_N50_SEED1_20260622_145052.md` 确认 centralized upper-bound full vs reference-only ablation。
- `RUN/GA/GA_TIERED_LINK_ABLATION_N50_SEED1_20260621_183039.md` 确认 GA N50 reference rows。

## Risk and Escalation

主要风险:

- Claim inflation: 把 output-level projection 写成 recursive online distributed filter。
- Metric inflation: 把 centralized consensus=0 写成方法主性能，而不是构造性 upper bound。
- Theory inflation: 把 moment barycenter 的 least-squares/moment projection 性质写成 truth-error 必然下降。
- Generalization risk: 当前实验是一个 tiered packet-loss scenario；需要跨场景或 independent verifier 才能强说泛化。
- Runtime risk: neighborhood full N50 runtime 是 tuned AA 的 `1.647x`，需要在论文中明确成本或优化。
- Citation risk: 外部相关工作引用需要从 Zotero 或现有 `.bib` 核对后再写入正式稿。

升级条件:

- 在投稿级主文中把方法称为 online/distributed recursive AA 前，必须把 operator 下沉到滤波递归内部，并做 N1/N5/N50 ablation。
- 在写强泛化 claim 前，需要至少一次 independent verifier 或独立复跑。
- 如果新 online 版本不能复现 local E-OSPA/RMSE 改善，应把当前方法降级为 post-processing/diagnostic projection。

## Reproducibility

关键复现命令:

```bash
octave --quiet --eval "test_cross_local_label_consensus_projection"
octave --quiet --eval "addpath('RUN/AA'); aaControls=struct('saveMat',false,'saveCheckpoints',false,'progressEverySteps',0,'existenceThreshold',0.18); [reportPath, summary]=runAaBalancedCardinalityValidation(50,1,true,aaControls,struct(),true,[9 18 19]); disp(summary.consensus); disp(summary.local.meanAcrossSensors);"
octave --quiet --eval "addpath('RUN/AA'); aaControls=struct('saveMat',false,'saveCheckpoints',false,'progressEverySteps',0,'existenceThreshold',0.18); [reportPath, summary]=runAaBalancedCardinalityValidation(50,1,true,aaControls,struct(),true,[16 17]); disp(summary.consensus); disp(summary.local.meanAcrossSensors);"
python3 /Users/dex/.codex/skills/auto-research/scripts/evidence_lint.py docs/AA_LABEL_BARYCENTER_PAPER_READY_CN.md
git diff --check
```

主要 artifact:

- `RUN/AA/AA_BALANCED_CARDINALITY_VALIDATION_N50_SEED1_20260622_174819.md`
- `RUN/AA/AA_NEIGHBORHOOD_LABEL_BARYCENTER_N50_SEED1_20260622_174817.log`
- `RUN/AA/AA_BALANCED_CARDINALITY_VALIDATION_N50_SEED1_20260622_145052.md`
- `RUN/AA/AA_CROSS_LOCAL_LABEL_CONSENSUS_ABLATION_N50_SEED1_20260622_145045.log`
- `RUN/GA/GA_TIERED_LINK_ABLATION_N50_SEED1_20260621_183039.md`

## Open Issues

- Recursive online implementation: 当前 neighborhood prototype 是 output-level post-pass，需要把 label proposal、matching、existence AA、moment consensus 和 lifecycle guard 下沉到 filter update loop。
- Stable matching: Proposition 4 依赖 fixed/correct matching；真实 birth/death、partial FOV、packet drop 下 matching 会变化。
- Conservative covariance: Moment barycenter 不保证 unknown-correlation consistency，需要比较 KLA/precision barycenter 或加入 covariance consistency diagnostics。
- Unified runtime table: AA 和 GA runtimes 来自不同 validation scripts，正式论文最好统一 measurement denominator。
- Broader validation: 当前结果足以启动写作，但投稿级泛化还需要 independent verifier、跨 seed block 或跨 scenario evidence。
- Citations: 相关工作需要从 Zotero AA Fusion 分类和现有 bibliography 中补入真实 citation keys。

## Recommendation

论文主线应写成“neighborhood label-barycenter AA 修复 distributed AA-LMB 的 label-space misalignment 和 matched posterior dispersion”，而不是“又一个 tuned AA 权重配置”。主结果放 neighborhood N50: consensus OSPA/Loc/Card 从 tuned `1.682607/1.472837/0.035350` 降到 `0.309818/0.216372/0.021200`，local E-OSPA/RMSE/CardErr 从 `2.029641/3.682880/0.088000` 降到 `1.681483/3.449035/0.077200`。

Centralized cross-local projection 应放在 method diagnostic 或 appendix，定位为 upper bound。Reference-only ablation 是关键证据: centralized full 比 reference-only local E-OSPA 好 `8.25%`、RMSE 好 `3.58%`；neighborhood full 的 RMSE reduction 是 `6.35%` 且 wins `48/50`，reference-only RMSE reduction 只有 `0.54%` 且 wins `27/50`。这组对照直接支撑“posterior barycenter 是必要方法组件”。

下一步如果要进入正式 paper draft，建议按以下顺序推进:

1. 先写 method paper draft skeleton，不急着再做 threshold search。
2. 同步实现 recursive online label-barycenter arm，保持与当前 output-level operator 同构。
3. 做 independent verifier 或独立复跑 N50。
4. 统一 AA/GA runtime denominator，并补一张 runtime/cost table。
5. 用 Zotero 和 `.bib` 核对相关工作引用，再把本文档拆进正式 manuscript。
