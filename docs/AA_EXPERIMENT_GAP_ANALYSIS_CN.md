# AA 实验结果不完善点梳理

最后更新: 2026-06-27 CST

## 总体判断

当前实验结果足够支撑一篇边界清晰的 TAES Regular Paper: 本文方法是一个作用在 active output tracks 上的 graph-local label/moment correspondence projection，用 reference selection、Hungarian assignment 和 matched first-two-moment barycenter 修复 target-wise AA/KLA scalar-weight routing 之后仍可能存在的 component-correspondence 问题。

但当前证据还不能支撑更强的说法: 它不是递归在线 LMB 更新的完整验证，不保证所有 close crossing / birth / death 下 assignment 总正确，也不证明 equal moment barycenter 具有 covariance consistency 或 reliability optimality。因此，实验“不完善”的核心不是主线证据不够，而是泛化维度和方法边界还没有完全展开。

2026-06-27 maneuver/crossing 补充实验的状态应按这个边界理解: N1/N5 smoke 已经证明 fixed crossing scenario、crossing-window metrics 和 three-arm mechanism split 能跑通；N50 report `RUN/AA/AA_BALANCED_CARDINALITY_VALIDATION_N50_SEED71_20260627_163915.md` 已完成并写入 `evidence_sources.json`，通过 crossing-window gate。Crossing-window 里 full method 的 E-OSPA/RMSE reductions 为 `11.08% / 16.99%`，reference-only 为 `4.98% / 5.38%`，支持 fixed crossing window 下的 full-vs-reference separation。但它仍只是一个预指定 crossing-window boundary check，不能写成 all target dynamics、birth/death 或 recursive-online guarantee。

## 已经比较稳的部分

| 证据块 | 当前状态 | 可以支撑的 claim |
| --- | --- | --- |
| Primary N50 | 已接入 generated fragments 和 independent verifier | full method 相对 fixed spatial-KLA AA baseline 同时降低 network disagreement 和 local tracking error。 |
| Reference-only ablation | 已进入主表和机制解释 | label copying 不能解释主要 RMSE 收益，matched posterior barycenter 是必要组件。 |
| Held-out baseSeed=11 N50 | 已接入正文 evidence path | 同一固定方法在非开发 seed 下保留 full-vs-reference RMSE separation。 |
| Harsh packet-loss N50 | 已作为 response-ready / concise Discussion evidence | 更严 packet-loss profile 下机制仍保持方向性收益，不是只对默认通信剖面有效。 |
| topology-ring / partial-FOV N50 | 已接入 scenario-family gate | sparse topology 和 sensing-geometry perturbation 下仍有 fixed-parameter robustness evidence。 |
| full-topology N50 | 已接入但只作为 boundary | ideal full-neighborhood 下三臂等价，只能说明 zero-disagreement ceiling，不是额外增益。 |
| maneuver/crossing N50 | 已接入 crossing-window gate | 预指定 crossing window 下 full-vs-reference RMSE/E-OSPA separation 仍存在，是 assignment-stability boundary evidence。 |

## 实验结果里仍不完善的点

### 1. Target dynamics 证据仍只覆盖一个预指定 crossing window

当前主证据覆盖了主 formation、held-out seed、harsh packet-loss、ring topology、partial FOV、full topology ceiling，以及一个固定 maneuver/crossing assignment-stability N50。这个 crossing N50 很有价值，因为它直接检验了本文理论中最敏感的 assignment-stability 边界，并且在预指定 crossing window 中 full 仍优于 reference-only。但它仍只是一个固定 crossing-window protocol，不覆盖 birth/death、长时间机动、多次交叉、强局部 posterior 偏置或递归反馈后的错误 label persistence。

Paper-facing 处理: 可以写成 one pre-specified maneuver/crossing window 下的 assignment-stability boundary evidence。不能写成 all target dynamics、crossing-robust、birth/death robust 或 assignment-guaranteed。

Research 机会: 继续设计 assignment-margin guarded projection。核心不是调 `H` 或 threshold，而是引入方法层面的 guard: assignment margin、covariance overlap、track age / support history 或 multi-step consistency，并用 harder birth/death/crossing scenarios 测试 no-guard vs guarded。

### 2. 当前方法是 output-level projection，不是递归在线滤波器

当前实现和稿件都把方法限定在 active output tracks 的 label/moment projection。这个边界使实验更干净，但也意味着还没有验证 projection 反馈进下一步 LMB filtering 后会不会导致 label persistence、wrong merge、track loss 或 birth/death 处理错误。

Paper-facing 处理: 保持 active-output scope。不要把方法写成 validated recursive LMB fusion。

Research 机会: 做 recursive guarded projection。必须包含 three-arm failure control: output-only、recursive no-guard、recursive with fixed guard。若 no-guard 失败而 guarded 成功，会形成更强的后续方法贡献。

### 3. Equal barycenter 还不是 covariance/reliability-aware 方法

当前 barycenter 是 matched first-two-moment least-squares representative。它能解释为什么 full method 比 reference-only 更好，但它没有使用 sensor reliability、measurement noise mismatch 或 covariance calibration 信息。若某些 sensor posterior 明显低质量，equal barycenter 可能过度信任差 posterior。

Paper-facing 处理: 可以说 first-two-moment barycenter 修复 correspondence 后的 state representative，但不能说 covariance consistent 或 reliability optimal。

Research 机会: 新增 reliability-weighted barycenter arm，配合 covariance-mismatch fixed scenario。关键是把它作为新方法 arm，而不是悄悄替换当前 equal barycenter 后继续叫同一个方法。

### 4. Network consensus 指标有构造性成分

Projection 本身会直接减少跨 sensor output disagreement，因此 network OSPA / localization disagreement 的大幅下降是预期中的机制结果。真正更有说服力的是 local E-OSPA / RMSE / CardErr 是否也改善，以及 full 是否优于 reference-only。

Paper-facing 处理: 主 claim 应依赖 local metrics、reference-only ablation、held-out replication 和 verifier，而不是只展示 consensus 归零或大幅下降。

Research 机会: 引入更难的 windowed metrics。例如 crossing window local RMSE/E-OSPA、wrong-assignment count、identity switch count，这比 whole-run average 更能检验 correspondence 方法。

### 5. Spatial RMSE 收益是稳定但不夸张的

主 N50 的 full RMSE reduction 是 `6.35%`，held-out 是 `6.64%`，harsh-loss 是 `7.98%`，partial-FOV 是 `6.50%`，topology-ring 更强一些为 `13.12%`。这说明方法有稳定空间收益，但不是数量级式性能革命。

Paper-facing 处理: 不要把稿件写成 broad tracking accuracy breakthrough。更准确的说法是: 在 target-wise AA routing 已修复后，残余 correspondence mismatch 仍会伤害 local tracking；graph-local label-barycenter projection 能以固定参数提供一致、可解释的增益。

Research 机会: 如果想提升主指标幅度，应从方法机制扩展入手，例如 covariance/reliability weighting 或 guarded recursive feedback，而不是对当前 formation 数据继续搜索阈值。

### 6. Full-topology 结果没有区分度

full-topology N50 中三臂 network disagreement 全为 `0.000000`，local E-OSPA/RMSE/CardErr 也完全相同。这不是坏结果，但它不能作为方法提升证据。

Paper-facing 处理: 只能写成 ideal full-neighborhood zero-disagreement equivalence boundary。不能写成在 full topology 下又取得了额外 improvement。

Research 机会: 该结果反而说明真正有价值的场景是 sparse / lossy / partially observable network，而不是完全通信 ceiling。

### 7. Runtime 仍是 prototype-level evidence

主 N50 中 full method runtime overhead 约 `1.644x`，reference-only 约 `1.534x`，主要来自 assignment 和 output rewriting。当前运行环境是 Octave/MATLAB-compatible prototype，不是优化实现。

Paper-facing 处理: 可以报告 paired relative overhead，但不要把绝对秒数写成硬件无关 benchmark。

Research 机会: 若 reviewer 关注实时性，可以补一个 complexity-focused implementation note 或优化 Hungarian matching / neighborhood batching，但这不是当前科学主线的最强贡献点。

## 对论文写作的直接结论

1. 当前稿件可以继续保持 content-ready 状态；实验不完善点主要是 future-risk boundary，不是当前主线的致命缺口。
2. Abstract、Introduction 和 Conclusion 必须保留 output-level、fixed-design、no per-scenario search、active-track projection 这些边界词。
3. Results 里应强调 full-vs-reference separation，而不是只强调 network disagreement 的大幅下降。
4. Discussion 的最佳结构是: 先说明 mechanism evidence，再说明 formation-family / crossing / covariance / recursive limits。
5. 若要继续增强方法，而不是当前 TAES 证据包，最值得做的是基于 crossing 结果设计 guarded projection 和 recursive no-guard/guarded 对照，而不是继续调现有 formation 参数。

## 后续 research 优先级

| 优先级 | 方向 | 为什么值得做 | 成功标志 |
| --- | --- | --- | --- |
| P1 | Maneuver / crossing assignment stress | 直接检验当前理论最敏感的 assignment-stability 边界 | crossing-window full 仍优于 reference-only，或暴露 guard 必要性。 |
| P1 | Assignment-margin guarded projection | 如果 crossing 暴露 ambiguity，这是最自然的方法层创新 | no-guard mixed/fails，guarded fixed method 恢复 local RMSE/E-OSPA。 |
| P2 | Covariance/reliability-weighted barycenter | 处理 equal barycenter 不能区分 posterior quality 的问题 | mismatch scenario 下 reliability arm 优于 equal full，而不是只调参。 |
| P2 | Recursive guarded online deployment | 把当前 output-level module 推进到真正滤波闭环 | recursive no-guard failure control + guarded success。 |
| P3 | Runtime optimization | 提高工程可用性和 reviewer 信心 | 降低 overhead，同时不改变 fixed-design metric conclusion。 |

## 不建议继续做的事

- 不继续围绕当前数据搜索 `H`、existence threshold、projection cutoff、barycenter weights 或 label rules。
- 不把 N1/N5 smoke 的偶然改善写进主文。
- 不把 full-topology ceiling 当成 gain evidence。
- 不把 equal barycenter 包装成 reliability-optimal。
- 不在没有 recursive validation 的情况下扩展成 online LMB feedback claim。
