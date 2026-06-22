# AA Fusion 大目标进度

最后更新: 2026-06-22 23:44:28 CST

## 当前结论

当前最有希望的方向不是继续搜索 `existenceThreshold`、support count 或 bridge prior，而是把 AA 的剩余问题重写成跨 local filters 的 label canonicalization 与 posterior barycenter 问题。

论文写作准备包已整理到 `docs/AA_LABEL_BARYCENTER_PAPER_READY_CN.md`，包含 paper-ready positioning、method outline、理论推导、N50 实验表、ablation 解释、图表计划和 claim 边界。

已经验证的两个原型是:

1. `Cross-local label-consensus spatial-KLA AA`: centralized/output-level upper-bound prototype。
2. `Neighborhood label-barycenter spatial-KLA AA`: 只使用 `neighborMap{s}` 的 output-level neighborhood iterative prototype。

核心流程:

```text
local outputs -> median-cardinality medoid reference label set
              -> Hungarian matching
              -> matched posterior barycenter
              -> shared labels/states written back to every local output
```

重要边界: centralized 版本会按构造让 consensus OSPA/Loc/Card 归零，不能单独作为论文 claim；neighborhood 版本不再全局读取 label set，N50 上 consensus 指标不为零但大幅优于 tuned baseline。两者仍是 output-level projection，不是递归滤波内部的最终 online AA 算法。

## Goal 状态

| 目标项 | 状态 | 证据 | 备注 |
| --- | --- | --- | --- |
| 开 AA 分支并修复 AA consumer-side 权重/label 问题 | 已完成并 push | branch `codex/aa-target-wise-fix`；commits `c3d26a5`, `d12fb92`, `7de3c0c` | 当前分支已推到 origin。 |
| 避免搜索式调参，转向方法层面设计 | 已执行 | `docs/AA_LABEL_UNCERTAINTY_AWARE_FUSION_RULE_CN.md` | 文档明确否定阈值搜索，转向 label-consensus/barycenter。 |
| N1 sanity: 新方法必须强于 tuned AA | 已完成 | `RUN/AA/AA_BALANCED_CARDINALITY_VALIDATION_N1_SEED1_20260622_121119.md` | local E-OSPA/RMSE/CardErr: tuned `2.0289/4.1450/0.06625` -> projection `1.6343/3.7789/0.06000`。 |
| N5 sanity: 不是单 seed 偶然 | 已完成 | `RUN/AA/AA_BALANCED_CARDINALITY_VALIDATION_N5_SEED11_20260622_121330.md` | local E-OSPA/RMSE/CardErr: tuned `2.032799/3.588145/0.086250` -> projection `1.660378/3.290166/0.068000`。 |
| N50 validation: AA 新方法优于 GA 两个 mode | 已完成 | `RUN/AA/AA_BALANCED_CARDINALITY_VALIDATION_N50_SEED1_20260622_122813.md`; `RUN/GA/GA_TIERED_LINK_ABLATION_N50_SEED1_20260621_183039.md` | projection local E-OSPA/RMSE/CardErr `1.645476/3.343985/0.071200`，低于 GA structure-aware `2.213284/4.228361/0.203975` 和 GA FID-FIA `2.179948/4.695098/0.151025`。 |
| N50 ablation: 证明设计组件合理 | 已完成 | `RUN/AA/AA_BALANCED_CARDINALITY_VALIDATION_N50_SEED1_20260622_145052.md`; `RUN/AA/AA_CROSS_LOCAL_LABEL_CONSENSUS_ABLATION_N50_SEED1_20260622_145045.log` | full barycenter local E-OSPA/RMSE/CardErr `1.645476/3.343985/0.071200`，reference-only `1.781249/3.463664/0.071200`。 |
| 算法设计与理论证明 | 已建立，online 化第一步通过 N50 | `docs/AA_LABEL_BARYCENTER_THEORY_CN.md` | 已整理 output-level projection 的可证明性质、reference-only ablation 解释、online/distributed label-barycenter AA 的收敛条件和边界；neighborhood iterative prototype 已完成 N50。 |
| Neighborhood online 化 sanity/validation | N1/N5/N50 已通过 | `RUN/AA/AA_BALANCED_CARDINALITY_VALIDATION_N1_SEED1_20260622_171542.md`; `RUN/AA/AA_BALANCED_CARDINALITY_VALIDATION_N5_SEED11_20260622_172034.md`; `RUN/AA/AA_BALANCED_CARDINALITY_VALIDATION_N50_SEED1_20260622_174819.md` | N50 consensus OSPA/Loc/Card `0.309818/0.216372/0.021200`，local E-OSPA/RMSE/CardErr `1.681483/3.449035/0.077200`，均低于两个 GA reference；runtime 约 `1.647x` tuned。 |
| Paper-writing ready 技术包 | 已完成 | `docs/AA_LABEL_BARYCENTER_PAPER_READY_CN.md` | 已把方法、理论、实验结果、ablation、图表计划和 claim 边界整理成可拆入 manuscript 的中文材料。 |
| 文档维护 | 已建立，持续维护 | 本文件；`docs/AA_LABEL_UNCERTAINTY_AWARE_FUSION_RULE_CN.md`; `docs/AA_LABEL_BARYCENTER_THEORY_CN.md` | 当前 checkpoint 已回填 N50 validation、N50 ablation、recommendation、theory boundary 和 neighborhood N50。 |

## 已完成的负结果

这些结果帮助排除“针对当前数据调参”的路径:

| 方向 | 结果 | 结论 |
| --- | --- | --- |
| full label-uncertainty rule | N1 未通过 | support-mass existence tempering 过强，导致 cardinality collapse。 |
| spatial-overlap only | N1 未通过 | 没有改善 OSPA/Loc。 |
| covariance-inflation only | N1 未通过 | 更干净但不改善 Loc/RMSE，runtime 上升。 |
| naive lifecycle split | N1 未通过 | 保留低置信 label 导致 clutter/runtime 上升。 |
| mature-label lifecycle | N1 未通过 | 收益太小且 Card/CardErr 变差。 |
| output-history lifecycle | N1/N5 near-neutral | 基本无副作用，但只有 `1e-4` 级收益，不值得 N50。 |
| support-consensus lifecycle | N1 未通过 | 单一 Neff survival gate 不足以修复 Loc gap。 |

## N50 ablation 结果

目的: 证明 full projection 的收益不只是“把所有节点复制成同一个 medoid output”，而是来自 matched posterior barycenter。

| Arm | 含义 | 预期判据 |
| --- | --- | --- |
| `Cross-local label-consensus spatial-KLA AA` | reference label set + Hungarian matching + posterior barycenter | 应在 local E-OSPA/RMSE 上优于 reference-only。 |
| `Reference-only label-consensus spatial-KLA AA` | 只复制 medoid reference output，不做 barycenter averaging | 若弱于 full projection，说明 barycenter 是必要组件。 |

N1/N5/N50 ablation 都支持该假设:

- N1: full `1.6343/3.7789/0.060000` vs reference-only `1.8111/3.9361/0.060000`。
- N5: full `1.6604/3.2902/0.068000` vs reference-only `1.7976/3.4685/0.068000`。
- N50: full `1.645476/3.343985/0.071200` vs reference-only `1.781249/3.463664/0.071200`。
- N50 paired deltas show reference-only is worse by `8.25%` E-OSPA and `3.58%` RMSE; CardErr is unchanged.

Neighborhood iterative prototype 的 N50 ablation 也支持该假设:

- tuned baseline local `2.029641/3.682880/0.088000`。
- neighborhood full `1.681483/3.449035/0.077200`。
- neighborhood reference-only `1.911286/3.662955/0.077200`。
- full 相对 tuned 的 paired local reductions 为 `17.15%` E-OSPA、`6.35%` RMSE、`12.27%` CardErr；wins 为 `50/50`、`48/50`、`46/50`。
- reference-only 的 RMSE reduction 只有 `0.54%`，wins `27/50`，说明 state barycenter 是空间性能收益的必要组件。

## 剩余风险

- 当前最佳 N50 结果仍来自 output-level projection；neighborhood iterative prototype 已通过 N50，但还不是递归滤波内部 online method。
- consensus 指标归零是构造结果；paper-facing claim 必须依赖 local metrics、GA reference 对照和 ablation。
- 还需要独立 verifier 或至少独立复跑，避免同一 worker lane 自证。
- 当前 ablation 证明了 barycenter 组件有用，理论文档也给出稳定 matching 下 online moment-consensus 收敛到 centralized moment barycenter 的条件；当前实现是 output-level neighborhood iterative prototype，不是递归滤波内部的最终 online method。

## 下一步

1. 把 output-level iterative prototype 下沉到递归滤波内部的 online label message / moment consensus。
2. 给 online 版本设计新的 method-level ablation: label canonicalization only、state barycenter only、iterative local consensus。
3. 找 independent verifier 或独立复跑 N50，降低 self-check 风险。
