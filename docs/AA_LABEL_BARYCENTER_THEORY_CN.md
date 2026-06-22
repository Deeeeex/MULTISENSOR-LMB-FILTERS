# Cross-local Label Barycenter AA 设计与理论边界

日期: 2026-06-22

## Question

如何把已经通过 N50 的 `Cross-local label-consensus spatial-KLA AA` 从一个 centralized/output-level prototype，整理成可发表方法的算法设计与理论边界: 哪些性质已经由当前实现直接保证，哪些性质由 N50 ablation 支持，哪些仍必须在 online/distributed 版本中重新验证。

## Scope

包含:

- `multisensorLmb/applyCrossLocalLabelConsensusProjection.m` 中的 reference label selection、Hungarian matching 和 moment-matched barycenter。
- `multisensorLmb/runDistributedLmbFilter.m` 中默认关闭的 output-level projection hook。
- `RUN/AA/runAaBalancedCardinalityValidation.m` 中的 arm 16/17。
- N1/N5/N50 full-barycenter vs reference-only ablation。
- 下一步 online/distributed label-barycenter AA 的理论目标和验收不变量。

排除:

- 不把当前 projection 声明为最终 distributed recursive LMB filter。
- 不把 consensus OSPA/Loc/Card 归零当成独立性能 claim。
- 不证明当前 method 在任意数据分布上优于 GA；当前证明只覆盖结构性质，性能优势由 paired experiments 支持。
- 不继续搜索 `existenceThreshold`、support count 或 bridge prior。

## Risk Tier

L2。该文档会影响后续算法叙事和 online/distributed 实现，但当前只是本地研究分支的设计证明和证据包，不是最终投稿文本。

## Claims

| ID | Claim | Confidence | Evidence IDs | Caveats |
| --- | --- | --- | --- | --- |
| C1 | 当前 projection 是一个输出空间投影: 同一时刻所有 local estimates 被替换为同一组 reference labels 和 consensus states，因此任意只比较 local outputs 之间差异的 consensus metric 都会按构造归零。 | High | E1, E2, E6 | 这是结构性质，不是 tracking performance 证明。 |
| C2 | reference label set 的 cardinality 不是任意固定阈值给出的，而是从 observed local cardinalities 的中位数附近选 medoid；若严格多数 local filters 有同一 cardinality，reference cardinality 必取该值。 | High | E1 | 只说明 cardinality selection 的鲁棒性，不说明 label identities 一定正确。 |
| C3 | 给定匹配组后，当前 barycenter state 是 matched empirical mixture 的一阶矩，covariance 是 matched mixture 的二阶中心矩；均值同时是 matched states 的最小二乘中心。 | High | E1 | 使用等权 moment matching；未引入 covariance-quality weights。 |
| C4 | reference-only ablation 在 N1/N5/N50 上均弱于 full barycenter projection 的 local E-OSPA/RMSE，说明经验收益不是只来自统一 label set，还来自 matched posterior barycenter。 | High | E3-E6 | CardErr 持平，说明该 ablation 主要隔离 spatial/posterior averaging。 |
| C5 | paper-facing 下一步不应包装 centralized projection，而应实现 online neighborhood label-barycenter AA: 用局部消息交换逼近同一 label-space projection，并在稳定 matching 假设下收敛到 centralized moment barycenter。 | Medium-High | E1, E7 | 收敛命题依赖固定匹配、连通图和 doubly-stochastic weights；这些需要实现后验证。 |

## Algorithm Design

### 当前 prototype

对时刻 `k`，每个 sensor `s` 的输出 estimate 记为:

```text
X_s(k) = { (ell_{s,i}, mu_{s,i}, Sigma_{s,i}) }_{i=1..n_s(k)}
```

当前 centralized projection 做四步:

1. 取所有 `n_s(k)` 的中位数，保留 cardinality 最接近中位数的 candidates。
2. 对每个 candidate `r`，计算它到所有 local outputs 的平均 set distance，选平均距离最小的 medoid 作为 reference sensor。
3. 用 Hungarian assignment 把每个 `X_s(k)` 的 state estimates 匹配到 reference label set `L_ref(k)`。
4. 对每个 reference label 的 matched states 做 moment matching，得到 consensus state，并把同一组 `(L_ref, mu_bar, Sigma_bar)` 写回所有 sensors 的 output estimate。

该算法把剩余问题从阈值搜索改成两个结构子问题:

- label canonicalization: 哪一组 labels 应作为同一时刻跨 local filters 的规范 label set。
- posterior barycenter: 已匹配到同一 reference label 的 local posteriors 应如何合成一个 shared posterior。

### Online/distributed 目标形式

下一步 paper-facing 方法应把上面的 global pass 改成 graph-local recurrence。每个 node `s` 只和邻居 `j in N_s` 交换 compact label messages:

```text
m_{s,ell}(k) = (ell, r_{s,ell}, mu_{s,ell}, Sigma_{s,ell}, age, support)
```

每个 fusion round 包含:

1. Neighborhood label proposal: 从 `N_s` 的 labels 中生成本地 reference/prototype label set。
2. Label matching: 用 OSPA/Mahalanobis-style cost 把邻居 labels 匹配到本地 prototypes。
3. Existence AA update: 对 matched label 做线性 existence pooling。
4. Moment consensus update: 对 matched state 的一阶矩和二阶矩做局部 convex averaging。
5. Lifecycle guard: 对低支持或未稳定匹配的 labels 延迟输出或降权，而不是用全局固定阈值硬删。

一个可验证的在线版本可以先从 moment-consensus 形式开始:

```text
q_{s,ell}^{(h+1)} = sum_{j in N_s} W_{sj} q_{j,pi_{j->s}(ell)}^{(h)}
```

其中 `q` 包含 `(r, mu, M2)`，`M2 = Sigma + mu mu'`，`W` 是通信图上的 row/doubly-stochastic 权重，`pi_{j->s}` 是邻居 label 到本地 prototype 的匹配。

## Theory Notes

### Proposition 1: output consensus is guaranteed by construction

命题: 若 projection 在时刻 `k` 输出同一组 `Y(k)` 给所有 sensors，则任意满足 `d(Y,Y)=0` 的 pairwise output disagreement metric 在该时刻为 0。

证明: projection 的最后一步把每个 `projectedEstimates{s}` 的 `labels{k}`、`mu{k}` 和 `Sigma{k}` 都设为同一个 `Y(k)`。因此任意 sensor pair `(a,b)` 有 `Y_a(k)=Y_b(k)`。由 metric identity，`d(Y_a(k),Y_b(k))=d(Y(k),Y(k))=0`。对所有 pair 求均值仍为 0。

边界: 这只证明 network disagreement 指标归零，不证明 `Y(k)` 接近 truth。论文 claim 必须依赖 local E-OSPA/RMSE/CardErr 和 GA reference 对照。

### Proposition 2: median-cardinality medoid resists minority cardinality outliers

命题: 若超过半数 local filters 在时刻 `k` 输出相同 cardinality `n*`，则当前 reference selection 的候选 cardinality 包含且只包含 `n*`，最终 reference cardinality 为 `n*`。

证明: 一维样本中严格多数值等于 `n*` 时，样本中位数为 `n*`。当前实现先选择 `abs(n_s - median(n))` 最小的 sensors。距离最小值为 0，且只有 `n_s=n*` 的 sensors 达到该值。因此 medoid 只会在 cardinality 为 `n*` 的 sensors 中选，最终 reference cardinality 为 `n*`。

边界: 如果没有严格多数，median cardinality 只是稳健中心，不保证等于 truth cardinality。

### Proposition 3: moment barycenter is the matched empirical mixture projection

给定 reference label `ell` 的 matched group:

```text
G_ell = { (mu_i, Sigma_i) }_{i=1..m}
```

当前实现输出:

```text
mu_bar = 1/m * sum_i mu_i
Sigma_bar = 1/m * sum_i [ Sigma_i + (mu_i - mu_bar)(mu_i - mu_bar)' ]
```

命题 3a: `mu_bar` 是最小化 `sum_i ||z - mu_i||^2` 的唯一解。

证明: 对目标函数 `J(z)=sum_i ||z-mu_i||^2` 求导，得 `2m z - 2 sum_i mu_i = 0`，所以 `z=mu_bar`。Hessian 为 `2m I`，正定，因此唯一最小。

命题 3b: `(mu_bar, Sigma_bar)` 是等权 Gaussian mixture `1/m sum_i N(mu_i,Sigma_i)` 的前两阶矩。

证明: 混合分布的一阶矩为 `E[x]=1/m sum_i mu_i=mu_bar`。二阶中心矩由 total covariance decomposition 得到:

```text
Cov[x] = E_i[Cov[x|i]] + Cov_i(E[x|i])
       = 1/m * sum_i Sigma_i + 1/m * sum_i (mu_i-mu_bar)(mu_i-mu_bar)'
```

这正是实现中的 `Sigma_bar`。

边界: 这是 AA/moment barycenter，不是 covariance-intersection/KLA 的保守交集。若 unknown cross-correlation 是主要风险，online 版本需要比较 moment-barycenter 与 precision/KLA-barycenter。

### Proposition 4: stable distributed moment consensus converges to the centralized moment barycenter

命题: 假设在一个时间窗口内 label matching 固定且正确，通信图连通，`W` primitive 且 doubly-stochastic。若每个 node 对 matched label 的 moment vector `q_s=(r_s, mu_s, M2_s)` 执行

```text
q_s^{h+1} = sum_j W_{sj} q_j^h
```

则 `q_s^h` 对所有 nodes 收敛到 `1/S * sum_s q_s^0`。这等于 centralized 等权 moment barycenter 的 moment statistics。

证明: average consensus 标准结论。`W` primitive 且 doubly-stochastic 时，`W^h -> (1/S) 11'`。堆叠所有 nodes 的 moment vectors，有 `q^h = W^h q^0`，因此每个 node 的极限为全局平均。因为 `q` 直接包含一阶矩和二阶矩，极限 moment statistics 与 centralized moment matching 的输入平均一致。

边界: 关键假设是 stable matching。真实 distributed LMB 中 label birth/death、partial FOV 和 packet drop 会让 matching 变化；这正是后续算法和 ablation 要验证的部分。

### Proposition 5: AA existence update preserves convex-hull boundedness

命题: 若 `r_j in [0,1]` 且 `W_{sj} >= 0`, `sum_j W_{sj}=1`，则 `r_s^+ = sum_j W_{sj} r_j` 仍在 `[min_j r_j, max_j r_j]` 内。

证明: `r_s^+` 是 `r_j` 的凸组合，凸组合落在输入值的凸包内。

含义: online 版本可以保持 AA existence 的 conservative linear-pool 解释，不需要引入场景固定的 hard existence gate。

## Evidence Ledger

| ID | Type | Source or artifact | What it supports | Strength |
| --- | --- | --- | --- | --- |
| E1 | code | `multisensorLmb/applyCrossLocalLabelConsensusProjection.m` | C1-C3, reference selection, matching and moment matching implementation | strong |
| E2 | code | `multisensorLmb/runDistributedLmbFilter.m` | C1, projection is called after all local distributed estimates are produced | strong |
| E3 | command | `octave --quiet --eval "test_cross_local_label_consensus_projection"` output: tests 1-3 passed | C4, projection and reference-only regression | strong |
| E4 | experiment | `RUN/AA/AA_BALANCED_CARDINALITY_VALIDATION_N1_SEED1_20260622_143214.md` and `RUN/AA/AA_BALANCED_CARDINALITY_VALIDATION_N5_SEED11_20260622_143447.md` | C4, N1/N5 reference-only ablation | medium-high |
| E5 | experiment | `RUN/AA/AA_BALANCED_CARDINALITY_VALIDATION_N50_SEED1_20260622_145052.md` | C4, N50 full vs reference-only ablation report | strong |
| E6 | command log | `RUN/AA/AA_CROSS_LOCAL_LABEL_CONSENSUS_ABLATION_N50_SEED1_20260622_145045.log` | C1, C4, command output for N50 ablation | strong |
| E7 | design doc | `docs/AA_LABEL_UNCERTAINTY_AWARE_FUSION_RULE_CN.md` | C5, current recommendation to move from centralized projection to online/distributed label-barycenter AA | medium |

## Verification Record

Independence status: self-check only. 本文档由同一 worker lane 基于当前代码和已生成报告整理，尚未经过 independent verifier。

已检查:

- `applyCrossLocalLabelConsensusProjection.m` 确认 projection mode 包含 `barycenter` 和 `reference-only`，默认是 `barycenter`。
- `selectReferenceSensor` 先按 median cardinality 选 candidate，再按平均 set distance 选 medoid。
- `buildConsensusStates` 对每个 reference state 使用 Hungarian matching 收集 matched local states。
- `momentMatchStates` 输出均值 `mean(mu_i)`，covariance 为 `mean(Sigma_i + delta_i delta_i')`，并做对称化和轻量 regularization。
- `runDistributedLmbFilter.m` 确认 projection hook 位于所有 local filters 完成之后，因此当前实现是 output-level pass。
- N50 ablation 报告确认 full barycenter local E-OSPA/RMSE/CardErr 为 `1.645476/3.343985/0.071200`，reference-only 为 `1.781249/3.463664/0.071200`。

未检查:

- 尚未实现 Proposition 4 对应的 online/distributed moment consensus。
- 尚未验证 stable matching 在 partial-FOV / tiered packet-drop 下能保持足够长的时间窗口。
- 尚未比较 moment-barycenter 与 KLA/precision barycenter 的 online 版本。

## Risk and Escalation

主要风险:

- 把 centralized projection 的构造性 consensus=0 写成 distributed fusion 性能提升。
- 把 least-squares barycenter 的结构证明误写成 truth-referenced error 必然下降。
- 在线版本中 label matching 不稳定时，moment consensus 可能把错误 tracks 合并。
- Moment barycenter 对 unknown cross-correlation 不如 KLA/CI 保守；如果 covariance consistency 变成主风险，需要新增 NEES/NIS 或 covariance-aware check。

升级条件:

- paper-facing method claim 需要 online/distributed 实现、N1/N5/N50 ablation、以及 independent verifier。
- 如果在线版本无法复现 N50 local metric 改善，应把当前 centralized projection 降级为 upper-bound diagnostic，不作为主方法。

## Reproducibility

当前 prototype 与 ablation 可复现命令:

```bash
octave --quiet --eval "test_cross_local_label_consensus_projection"
octave --quiet --eval "addpath('RUN/AA'); aaControls=struct('saveMat',false,'saveCheckpoints',false,'progressEverySteps',0,'existenceThreshold',0.18); [reportPath, summary]=runAaBalancedCardinalityValidation(50,1,true,aaControls,struct(),true,[16 17]); disp(summary.consensus); disp(summary.local.meanAcrossSensors);"
python3 /Users/dex/.codex/skills/auto-research/scripts/evidence_lint.py docs/AA_LABEL_BARYCENTER_THEORY_CN.md
```

online/distributed 版本实现后至少应新增:

```bash
octave --quiet --eval "test_aa_online_label_barycenter_consensus"
octave --quiet --eval "addpath('RUN/AA'); aaControls=struct('saveMat',false,'saveCheckpoints',false,'progressEverySteps',0,'existenceThreshold',0.18); [reportPath, summary]=runAaBalancedCardinalityValidation(1,1,true,aaControls,struct(),true,[16 <online-arm>]); disp(summary.consensus); disp(summary.local.meanAcrossSensors);"
```

## Open Issues

- 当前 reference selection 使用等权 sensor medoid；online 版本是否应引入 link quality、posterior covariance 或 label age 权重仍未定。
- 当前 barycenter 使用等权 moment matching；是否需要 branch weights 或 covariance-quality weights 需要专门 ablation。
- 当前 matching cost 是 Euclidean position distance；online 版本可能需要 Mahalanobis/OSPA hybrid cost 来适配不同尺度和 covariance。
- Label birth/death 的稳定 matching 条件还没有实现机制，需要把 lifecycle guard 与 label prototype consensus 联合设计。
- 如果目标是严格 AA family，moment-barycenter spatial update 的理论定位需要写成 output/projection layer；如果目标是 hybrid family，则可以把 spatial branch 定位为 label-aligned barycenter operator。

## Recommendation

把当前结果定位为一个通过 N50 的 method-level upper-bound prototype: 它证明跨 local filters 的 label canonicalization 加 matched posterior barycenter 是值得推进的方向，也通过 reference-only ablation 排除了“只是复制 medoid output”的解释。

下一步不要继续做场景阈值搜索。应实现一个 online/distributed label-barycenter AA arm，并围绕三个 ablation 证明方法合理性:

1. label canonicalization only: 统一 labels，但不做 state barycenter。
2. state barycenter only: 在稳定 labels 下做 moment/KLA barycenter。
3. full online label-barycenter AA: label proposal、matching、existence AA、moment consensus 和 lifecycle guard 同时启用。

只有当 online 版本在 N1/N5 通过 gate，并在 N50 上保持 local E-OSPA/RMSE/CardErr 优势时，才应把它升级为 paper-facing algorithm。
