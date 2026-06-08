# 严格组件消融实验运行指南

## 目的

这个实验专门回应 reviewer 对现有 Table 4 归因不足的意见。它复用现有
tiered-link 4+4 formation 场景、固定 seed 和 paired-trial 机制，只改变
`adaptiveFusion` 配置，因此同一 trial 内的所有方法共享 truth、measurements
和通信 realization。

默认运行前 10 个 arm：

| # | Variant | 归因目的 |
|---:|:--|:--|
| 1 | Fixed Metropolis | 固定拓扑权重 baseline |
| 2 | Cov only | posterior concentration 单独作用 |
| 3 | Link only | realized communication quality 单独作用 |
| 4 | Exist only | existence confidence 单独作用 |
| 5 | Cov + Link | covariance-link backbone |
| 6 | Cov + Link + Exist, shared weights | existence confidence 对 shared scalar weight 的增益 |
| 7 | Cov + Link + Exist, branch-decoupled | spatial/existence 分支解耦的增益 |
| 8 | + structure-aware | weak structure-aware prior 的增益 |
| 9 | + EMA/floor | 时间平滑和最终权重下界的稳定化增益 |
| 10 | + FID-FIA existence only | 只在 existence branch 引入 FID-FIA 的增益 |

构造器还提供第 11 个可选 arm：

`Balanced + FID-FIA existence`

它直接从当前无稳定化 Balanced 配置派生，只在 existence branch 增加
FID-FIA。六个 EMA/floor 字段均保持为 0。这个 arm 用于评估论文中的下一
operating point，不应替代 arm 10 作为旧十行因果链中的单变量对照。

## 关键实验约束

1. Arm 2-8 都关闭 EMA 和 final weight floor。
2. Arm 9 只新增 `EMA=0.7` 和 `floor=0.05`。
3. Arm 10 保留 arm 9 的稳定化配置，只新增 existence-branch FID-FIA。
4. 不要把旧 Table 4 的 covariance-link 数值直接拼入新表。新表必须整组重跑，
   因为这里为了严格归因，arm 2-8 使用未稳定化配置。
5. 主表优先报告 network disagreement 三项指标，同时保留 local E-OSPA、
   local RMSE 和 local cardinality error，防止“节点更一致但整体偏离 truth”。

## 运行命令

先做配置测试，不运行滤波：

```matlab
addpath('RUN/GA');
addpath('tests');
test_rigorous_ablation_arms;
```

做 1-trial 全表烟雾实验：

```matlab
addpath('RUN/GA');
[reportPath, summary] = ...
    runMultisensorFilters_formation_4plus4_RigorousAblation(1, 1, true, true);
```

正式 paired 50-trial：

```matlab
addpath('RUN/GA');
[reportPath, summary] = ...
    runMultisensorFilters_formation_4plus4_RigorousAblation(50, 1, true, true);
```

同时加入第 11 个当前配置 FID-FIA extension：

```matlab
addpath('RUN/GA');
[reportPath, summary] = ...
    runMultisensorFilters_formation_4plus4_RigorousAblation( ...
        50, 1, true, true, struct(), struct(), 1:11);
```

只运行当前无 EMA/floor 的 FID-FIA extension：

```matlab
addpath('RUN/GA');
[reportPath, summaryPath, summary] = ...
    runFidFiaExistenceNoStabilizationValidation(50, 1);
```

调试时只运行指定 arm，例如固定权重、Cov+Link、shared、decoupled、
structure-aware、稳定化和 FID-FIA：

```matlab
selectedArms = [1, 5, 6, 7, 8, 9, 10];
[reportPath, summary] = ...
    runMultisensorFilters_formation_4plus4_RigorousAblation( ...
        1, 1, true, true, struct(), struct(), selectedArms);
```

## 结果解释顺序

- `Cov only` 对 `Fixed Metropolis`：covariance concentration 是否单独有效。
- `Link only` 对 `Fixed Metropolis`：当前提升是否主要来自 link quality。
- `Exist only` 对 `Fixed Metropolis`：existence confidence 是否有独立信号。
- Shared three-factor 对 `Cov + Link`：新增 existence confidence 是否有效。
- Branch-decoupled 对 shared：分支解耦是否有效。
- Structure-aware 对 branch-decoupled：弱结构先验是否有效。
- EMA/floor 对 structure-aware：稳定化是否有效。
- FID-FIA existence only 对 EMA/floor：existence-only information geometry 是否有效。

如果某一步的均值差异很小，正文不能只依据单个均值宣称该模块必要。应同时检查
paired trial 差值、win count、置信区间以及 local tracking metrics，再决定是保留
“贡献”表述，还是降级为稳定化/部署选项。
