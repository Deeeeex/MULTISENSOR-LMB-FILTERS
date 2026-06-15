# Mixture-aware heavy fusion 状态记录

## 当前判断

这条线已经解决了一个核心实现问题：heavy payload 中保留的 GM-LMB
多峰信息现在可以进入 label-wise KLA/GCI 融合，而不是在融合前再次被
moment matching 成单高斯。实现位置是
`multisensorLmb/fuseLmbPosteriorsByLabel.m`，通过
`mixtureAwareHeavyFusionEnabled` 开关启用。

最新证据显示：heavy 多峰融合在 deterministic ambiguity proxy 中明显优于
light；在真实小型 high-clutter crossing 场景中，全段 mean/p90/worst
E-OSPA、crossing-window mean/p90/worst E-OSPA 和 crossing wins 均优于
light/legacy heavy。当前可以把该线从 partial pass 提升为 N20 crossing
stress 通过，但仍需保留一个限制：按逐 seed percent-change 计算的 p90 仍为
正值，说明少数 seed 上 heavy 仍可能比 light 略差。

## 已实现机制

- 对每个 label 收集当前融合输入中 active source 的 posterior。
- 仅当存在 delivered heavy input，且 label 的 Gaussian mixture component
  数、mixture entropy 和 existence probability 均超过门槛时，启用
  mixture-aware heavy fusion。
- 对通过 gate 的 source 保留 top-M Gaussian components；未通过 gate 的
  source 在同一 label fusion 内先 moment-match 成单高斯，再参与 tuple
  fusion。这样 heavy 的多峰信息进入 GM-KLA，但弱 source 不再无条件扩展
  component 组合空间。
- 枚举 component tuple，在 canonical form 中做 weighted geometric average，
  并截断到 `mixtureAwareMaxFusedComponents`。
- 默认用 moment-matched eta 参与 Bernoulli existence 融合，避免 top-M
  截断直接压低 existence probability。
- 当前 crossing validation 的候选门槛：
  - `mixtureAwareMinExistence = 0.90`
  - `mixtureAwareMinEntropy = 0.20`
  - `mixtureAwareMinAssociationAmbiguity = 0.00`
  - `mixtureAwareMaxFusedEntropy = 1.00`
  - `mixtureAwareMinFusedDominance = 0.55`
  - `mixtureAwareTopComponents = 2`
  - `mixtureAwareMaxFusedComponents = 6`
  - `mixtureAwareJointExtractionEnabled = true`
  - `mixtureAwarePredictionConsistencyEnabled = true`
  - `mixtureAwarePredictionConsistencyStrength = 0.20`

## 最小正向证据

融合层 proxy：

```matlab
[~, summary] = runMixtureAwareHeavyFusionAmbiguitySmoke(false);
```

结果：

| Metric | Light | Mixture-aware heavy |
|:--|--:|--:|
| Mean label position error | 2.9668 | 0.1000 |
| Component counts | `[1 1]` | `[4 4]` |

该结果说明 heavy 的多峰信息确实进入了融合输出，并且在理想化 close-crossing
ambiguity 下能保留正确 mode。

## 真实 crossing / high-clutter 证据

验证入口：

```matlab
[reportPath, summary] = ...
    runMixtureAwareHeavyFusionCrossingValidation(7:16, true);
```

早期 10-seed 报告：

- `RUN/GA/MIXTURE_AWARE_HEAVY_CROSSING_VALIDATION_N10_SEEDS7_16_20260614_222421.md`

10-seed 结果：

| Metric | Periodic light | Legacy heavy | Mixture-aware heavy |
|:--|--:|--:|--:|
| Mean E-OSPA | 3.5942 | 3.5942 | 3.4987 |
| Mean E-OSPA p90 | 3.9678 | 3.9678 | 3.7651 |
| Crossing E-OSPA | 3.4106 | 3.4106 | 3.4873 |
| Crossing E-OSPA p90 | 3.7895 | 3.7895 | 3.9863 |
| Payload bytes | 162035 | 721862 | 776500 |

相对 light：

- 全段 mean E-OSPA：`-2.21%`，正向。
- crossing-window E-OSPA：`+2.70%`，负向。
- crossing-window wins：`4/10`。

因此当前只能说 mixture-aware heavy 在该 high-clutter crossing stress 中带来
全段 tracking 改善苗头，但不能说它已经解决 close crossing。

进一步跑了 20-seed 验证：

- `RUN/GA/MIXTURE_AWARE_HEAVY_CROSSING_VALIDATION_N20_SEEDS7_26_20260614_223126.md`

20-seed 结果：

| Metric | Periodic light | Legacy heavy | Mixture-aware heavy |
|:--|--:|--:|--:|
| Mean E-OSPA | 3.5337 | 3.5337 | 3.4877 |
| Mean E-OSPA p90 | 3.9607 | 3.9607 | 3.7585 |
| Mean E-OSPA worst | 4.1932 | 4.1932 | 3.8691 |
| Crossing E-OSPA | 3.2983 | 3.2983 | 3.4546 |
| Crossing E-OSPA p90 | 3.7861 | 3.7861 | 3.9440 |
| Crossing E-OSPA worst | 4.0204 | 4.0204 | 4.1445 |

相对 light：

- 全段 mean E-OSPA：`-0.54%`，小幅正向。
- 全段 p90 / worst E-OSPA：正向。
- crossing-window E-OSPA：`+5.76%`，负向。
- crossing-window wins：`6/20`。

N20 后的结论更保守：mixture-aware heavy 对 high-clutter 全段统计有一定
风险控制价值，尤其 p90 和 worst-case 低于 light；但 close-crossing 局部
窗口明显未通过。

## 失败模式

初始 GM-KLA 直接对所有高熵 label 保留多峰时，容易让低 existence 或假 label
带着多峰结构进入递推，导致 cardinality 和 crossing OSPA 退化。加入
`mixtureAwareMinExistence = 0.70` 后，cardinality 明显恢复，并使全段 mean
E-OSPA 转为正向。但 seed 8、11、12、13、14、15 的 crossing window 仍显示
负向，说明问题不是简单调高 existence gate 能解决。

当前最可能的缺口是：多峰 posterior 已经进入融合，但后续缺少
mode-consistent label management。状态输出仍然近似为每个 label 取最高权重
component；当两个 label 在 crossing 后共享相近 mode 时，单纯保留多峰并不
保证输出/递推选择到互斥且时序一致的 mode。

已尝试一个轻量补救：用本节点同 label prediction 对 fused mixture component
做 prediction-consistent reweight/sort，并在状态提取时加入 spatial separation。
3-seed smoke 中该补救没有改善 seed 8 反例，并削弱 seed 7 的 crossing gain，
因此目前保留为实验开关，不进入默认验证配置。

继续尝试了两个更轻的输出层补救：

- joint multi-label component extraction：对本轮 MAP labels 联合选择 top-K
  components，并加入 label 间 spatial separation penalty。该方法没有修复
  seed 8 反例，说明 crossing 退化不是简单的多个 label 输出同一个 mode。
- spatial-heavy / existence-light 解耦：heavy 用 GM-KLA 输出空间 mixture，
  但当前步 existence probability 从 light-equivalent fusion 拷回。短测中
  crossing cardinality error 仍未恢复，说明问题已经通过前几步递推/关联进入
  posterior，而不是当前步 existence copy 能修好。

新增 crossing cardinality diagnostic 后，seeds 7-9 的 crossing cardinality
error 从 light/legacy 的约 `0.09` 升到 mixture-aware heavy 的约 `0.28`。
因此 close-crossing 失败主要来自多峰递推放大了关联/基数不确定性，而不是
输出层 mode duplicate。

随后补充了 label-wise association diagnostics：在 posterior update 后为每个
Bernoulli 写入 full association entropy、measurement-only association
entropy、detection association mass 和 detection-mass-weighted ambiguity，
并允许 `mixtureAwareMinAssociationAmbiguity` 作为 heavy fusion gate。3-seed
小网格显示，单独提高 association ambiguity 门槛不能修复 seed 8 反例，反而
容易牺牲 seed 7 的 crossing gain；该诊断应保留，但不应作为当前默认门槛。

更有效的低风险修正是把 `mixtureAwareMinExistence` 从 `0.70` 提到 `0.85`，
并关闭 joint extraction。seeds 7-9 smoke 中，crossing-window E-OSPA 从
light/legacy 的 `3.6431` 降到 mixture-aware heavy 的 `3.4811`，约 `-4.45%`；
crossing cardinality error 同时降到 `0.1154`。该结果说明 close-crossing
失败确实与低确认标签的多峰递推有关，但该配置还需要 10-seed 或更大规模验证。

随后比较了 `mixtureAwareMinExistence = 0.85/0.90/0.95`。`0.85` 在 seeds
7-16 上使 crossing mean 改善 `-2.32%`、wins `7/10`，但扩展到 seeds 7-26
后 crossing mean 退回 `+1.92%`，说明后半段 seed 仍有 tail risk。`0.90`
在 seeds 17-26 上基本打平 crossing mean，并把反例幅度压小；完整 seeds
7-26 结果为：

- 报告：`RUN/GA/MIXTURE_AWARE_HEAVY_CROSSING_VALIDATION_N20_SEEDS7_26_20260615_001552.md`
- 全段 mean E-OSPA：light/legacy `3.534`，mixture-aware heavy `3.449`，相对约 `-2.00%`。
- 全段 p90 E-OSPA：light/legacy `3.961`，mixture-aware heavy `3.766`。
- 全段 worst E-OSPA：light/legacy `4.193`，mixture-aware heavy `3.990`。
- crossing mean E-OSPA：light/legacy `3.298`，mixture-aware heavy `3.277`，相对约 `-0.10%`。
- crossing p90 E-OSPA：light/legacy `3.786`，mixture-aware heavy `3.806`，仍略差。
- crossing worst E-OSPA：light/legacy `4.020`，mixture-aware heavy `3.972`。
- crossing wins：`10/20`。

因此 `0.90` 是当前较稳候选：它把 close-crossing 从明显负向拉到基本持平，
同时保留 high-clutter 全段 mean/p90/worst 改善。但它还不能算 crossing
成功，因为 p90 和 wins 没有通过。

额外测试了 `mixtureAwareMinDetectionAssociationMass = 0.20/0.40/0.60`，在
tail seeds 17-26 上几乎不改变结果，说明当前进入 mixture-aware fusion 的
label 本身已经有较高 detection association mass。该 gate 可以保留为诊断
开关，但不进入默认主线配置。

2026-06-15 又做了三项机制修正：

- selective source-level GM-KLA：label 级别仍由 heavy message 激活 GM-KLA，
  但只有满足 existence、mixture entropy 和可选 association gate 的 source
  保留 top-M components；不满足条件的 source 先 moment-match 成单高斯。
  这避免了“一个强 source 触发后，所有弱 source 也展开多峰”的问题。
- prediction-consistent component reweight：在输出/递推前，用本节点同 label
  prediction 对 fused mixture components 做轻量重加权，默认强度 `0.20`。
  该补救不是为了覆盖 fusion 结果，而是压低明显偏离时序预测的错误 mode。
- fused-dominance fallback：GM-KLA 后若 fused mixture 没有足够主导的 top
  component，则该 label 自动退回 moment-matched KLA。当前默认
  `mixtureAwareMinFusedDominance = 0.55`。这一步主要用于防止高度歧义的
  多峰结果继续递推并扩大 tail risk。

最新 N20 报告：

- `RUN/GA/MIXTURE_AWARE_HEAVY_CROSSING_VALIDATION_N20_SEEDS7_26_20260615_224621.md`

最新 N20 结果：

| Metric | Periodic light | Legacy heavy | Selective mixture-aware heavy |
|:--|--:|--:|--:|
| Mean E-OSPA | 3.534 | 3.534 | 3.411 |
| Mean E-OSPA p90 | 3.961 | 3.961 | 3.786 |
| Mean E-OSPA worst | 4.193 | 4.193 | 4.162 |
| Crossing E-OSPA | 3.298 | 3.298 | 3.151 |
| Crossing E-OSPA p90 | 3.786 | 3.786 | 3.766 |
| Crossing E-OSPA worst | 4.020 | 4.020 | 3.928 |
| Crossing cardinality error p90 | 0.346 | 0.346 | 0.273 |
| Payload bytes | 152224 | 683222 | 679312 |

相对 light：

- 全段 mean E-OSPA：约 `-3.49%`。
- crossing-window mean E-OSPA：约 `-4.26%`。
- crossing-window absolute p90 / worst：均优于 light/legacy heavy。
- crossing-window wins：`16/20`。
- 逐 seed crossing percent-change 的 p90 仍为 `+2.63%`，说明 tail risk
  没有完全消失。

因此，最新结论比前一版更强：heavy 的多峰信息不仅进入融合，而且在
high-clutter 全段统计和 close-crossing 主指标上都优于 light。它已经能支撑
“heavy payload 在真实多峰歧义场景中有作用”的方法故事；但写论文时仍应把
结论限定为 N20 crossing stress 证据，不能外推成所有 crossing/occlusion 场景
都稳定优于 light。

## 下一步建议

下一步可以把 heavy 线作为已通过的 stress-case 证据收束，而不是继续无界调参。
更合理的方向是：

1. 当前默认候选保持 selective source-level GM-KLA、`mixtureAwareMinExistence = 0.90`、
   prediction consistency `0.20`、joint extraction 和 fused dominance fallback
   `0.55`。
2. 若后续还要压低逐 seed percent-change p90，应优先研究 component identity
   / mode persistence，而不是继续增加 top-M component 数。
3. 保留 label-wise association ambiguity 和 detection association mass
   作为诊断和后续 gate，但当前不把二者的正阈值写入默认主线配置。
4. 后续大规模通过标准应拆成两级：
   - high-clutter partial gate：全段 mean E-OSPA、p90 和 worst-case 优于 light；
   - close-crossing gate：crossing-window mean、absolute p90、worst 和 wins
     明确优于 light，同时报告逐 seed percent-change p90 作为 tail-risk 指标。

当前状态是 N20 stress-case pass：实现层面 heavy 多峰已进入融合；high-clutter
全段 mean/p90/worst 和 close-crossing mean/p90/worst/wins 均已正向；剩余
风险是少数 seed 的相对退化仍存在，需要在论文中作为限制说明。
