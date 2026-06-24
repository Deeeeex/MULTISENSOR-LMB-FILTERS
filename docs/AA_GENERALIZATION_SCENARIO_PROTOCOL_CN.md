# AA generalization scenario protocol

最后更新: 2026-06-24 16:25 CST

## 目的

当前 TAES 稿件已经有三类 packet-loss evidence: main tiered profile、baseSeed=11 held-out replication、baseSeed=21 harsh-loss stress。下一步的审稿风险不应再通过搜索阈值或 packet-loss 参数来处理，而应检验方法是否在不同场景族下仍保持同一个机制解释:

```text
scalar AA weights route trust -> still need component correspondence
neighborhood reference selection -> canonicalize local labels
Hungarian assignment + moment barycenter -> recover spatial gain beyond label copying
```

本协议定义 topology、partial-FOV 和 idealized-full-topology 三个固定场景族。它们用于验证方法层面的泛化和边界，不用于选择 `H`、existence threshold、projection cutoff、barycenter weights、label rules 或 packet-loss profile。

## 固定方法参数

所有场景族沿用 TAES 主实验设置:

- Arms: `[9 18 19]`
  - `Tuned spatial-KLA AA`
  - `Neighborhood label-barycenter spatial-KLA AA`
  - `Neighborhood reference-only label-consensus spatial-KLA AA`
- `existenceThreshold = 0.18`
- `crossLocalConsensusIterations = 3`
- `aaSpatialFusionMode = kla`
- `saveMat = false`
- `saveCheckpoints = false`
- `progressEverySteps = 0`
- Packet-loss profile保持主实验的 tiered profile，除非协议另行说明。

## 场景族定义

| Family | scenarioLabel | 改变量 | 不变量 | 目的 |
| --- | --- | --- | --- | --- |
| `topology-ring` | `topology-ring-formation` | `neighborMapMode = ring`，每个节点只看自己和环上左右邻居 | target formation、FOV、packet loss、method parameters | 检验 sparse local topology 下多跳 neighborhood projection 是否仍有机制收益。 |
| `partial-fov35` | `partial-fov35-formation` | `sensorFovHalfAngleDeg = 35` | 4+4 topology、packet loss、method parameters | 检验更强 partial-observation stress 下 label reference 和 barycenter 是否仍优于 label copying alone。 |
| `full-topology` | `full-topology-formation` | `neighborMapMode = full` | target formation、FOV、packet loss、method parameters | 提供一个 idealized topology ceiling，帮助区分 topology bottleneck 与方法本身的 matching/barycenter behavior。 |

当前 `RUN/AA/runAaBalancedCardinalityValidation.m` 已新增第 9 个可选参数 `scenarioOverrides`。默认 `struct()` 保持已有 4+4 formation 结果完全不变；只有 launcher 显式传入 scenario 时才改变 topology/FOV。

## Launcher

通用入口:

```bash
RUN/AA/launchAaTaesScenarioFamilySmoke.sh
```

默认值:

- `AA_SCENARIO_FAMILY=topology-ring`
- `AA_SCENARIO_TRIALS=1`
- `AA_SCENARIO_BASE_SEED=31`
- `AA_ALLOW_CONCURRENT=0`

示例:

```bash
AA_SCENARIO_FAMILY=topology-ring AA_SCENARIO_TRIALS=1 RUN/AA/launchAaTaesScenarioFamilySmoke.sh
AA_SCENARIO_FAMILY=partial-fov35 AA_SCENARIO_TRIALS=5 RUN/AA/launchAaTaesScenarioFamilySmoke.sh
AA_SCENARIO_FAMILY=full-topology AA_SCENARIO_TRIALS=50 RUN/AA/launchAaTaesScenarioFamilySmoke.sh
```

每次启动会写 durable log 和 pid:

```text
RUN/AA/AA_TAES_SCENARIO_{family}_N{N}_BASESEED{baseSeed}_{timestamp}.log
RUN/AA/AA_TAES_SCENARIO_{family}_N{N}_BASESEED{baseSeed}_{timestamp}.pid
```

## 解释规则

每个场景族都按相同层次解释:

1. Network OSPA / localization disagreement / cardinality dispersion: 检查 graph-local label projection 是否仍降低跨传感器输出不一致。
2. Local E-OSPA / CardErr: 检查一致性收益是否转化为 finite-set tracking quality。
3. Local RMSE: 检查 matched posterior barycenter 是否带来空间定位收益。
4. Reference-only ablation: 若 full 与 reference-only 接近，说明该场景中主要收益可能来自 label-set canonicalization；若 full 显著优于 reference-only，继续支持 moment barycenter 机制。

Mixed 或 negative 结果不能触发参数回调搜索。应按如下方式处理:

- 若 topology-ring 变差: 讨论 sparse topology 下 medoid/reference propagation 的局限，考虑递归 message-passing 版本，而不是调 `H` 直到当前数据变好。
- 若 partial-fov35 变差: 讨论 FOV mismatch / missing support 下 assignment uncertainty 和 birth/death lifecycle guard。
- 若 full-topology ceiling 很强: 说明 topology bottleneck 是当前 output-level neighborhood operator 的主要限制之一。

## 当前状态

- Code support: 已完成。`runAaBalancedCardinalityValidation` 支持 `scenarioOverrides`，并在报告中记录 `scenarioLabel`、`neighborMapMode`、FOV 和 sensor-motion settings。
- Launcher: 已完成。`RUN/AA/launchAaTaesScenarioFamilySmoke.sh` 支持三类场景族和 durable log/pid handoff。
- Evidence: `topology-ring` 与 `partial-fov35` 已完成 N1 smoke；尚未形成 paper-grade N50。下一步补 N5 smoke，若报告结构和机制信号合理，再启动固定 N50。

## Topology-ring N1 smoke

Smoke command was run on 2026-06-24 with `AA_SCENARIO_FAMILY=topology-ring`, `AA_SCENARIO_TRIALS=1`, `AA_SCENARIO_BASE_SEED=31`, and trial seed `32`.

Artifacts:

- Report: `RUN/AA/AA_BALANCED_CARDINALITY_VALIDATION_N1_SEED31_20260624_155105.md`
- Launcher log: `RUN/AA/AA_TAES_SCENARIO_topology_ring_N1_BASESEED31_20260624_155103.log`

Run metadata recorded in the report:

- `scenarioLabel: topology-ring-formation`
- `neighborMapMode: ring`
- `pDropLevels: [0 0.1 0.2 0.5]`
- `pDropLevelCounts: [1 4 1 2]`
- Arms: `[9 18 19]`

Smoke result:

| Arm | Net OSPA | Loc. disag. | Card. disp. | E-OSPA | RMSE | CardErr |
| --- | ---: | ---: | ---: | ---: | ---: | ---: |
| Tuned spatial-KLA AA | 2.771921 | 2.682580 | 0.083750 | 2.580668 | 4.568272 | 0.116250 |
| Neighborhood label-barycenter | 1.348601 | 1.197544 | 0.030000 | 2.010548 | 4.035532 | 0.065000 |
| Neighborhood reference-only | 2.263993 | 2.099705 | 0.030000 | 2.410974 | 4.511156 | 0.065000 |

Interpretation:

- The new scenario override path is functional: the report records the ring topology and all three paper arms complete.
- The full method improves both network agreement and local tracking on this single sparse-topology trial.
- Reference-only improves cardinality and label-set coherence, but its RMSE gain is much smaller than the full method. This is a useful early signal for the matched-barycenter mechanism, not a paper-grade conclusion.
- No parameter should be changed in response to this N1 result. The next evidence step is a fixed N5/N50 run, not tuning.

## Partial-fov35 N1 smoke

Smoke command was run on 2026-06-24 with `AA_SCENARIO_FAMILY=partial-fov35`, `AA_SCENARIO_TRIALS=1`, `AA_SCENARIO_BASE_SEED=41`, and trial seed `42`.

Artifacts:

- Report: `RUN/AA/AA_BALANCED_CARDINALITY_VALIDATION_N1_SEED41_20260624_155707.md`
- Launcher log: `RUN/AA/AA_TAES_SCENARIO_partial_fov35_N1_BASESEED41_20260624_155706.log`

Run metadata recorded in the report:

- `scenarioLabel: partial-fov35-formation`
- `neighborMapMode: 4plus4`
- `sensorFovHalfAngleDeg: 35.000`
- `pDropLevels: [0 0.1 0.2 0.5]`
- `pDropLevelCounts: [1 4 1 2]`
- Arms: `[9 18 19]`

Smoke result:

| Arm | Net OSPA | Loc. disag. | Card. disp. | E-OSPA | RMSE | CardErr |
| --- | ---: | ---: | ---: | ---: | ---: | ---: |
| Tuned spatial-KLA AA | 2.327551 | 1.182761 | 0.457500 | 2.922353 | 4.461712 | 0.612500 |
| Neighborhood label-barycenter | 1.516556 | 0.126668 | 0.470000 | 2.734257 | 4.281871 | 0.600000 |
| Neighborhood reference-only | 1.884422 | 0.553296 | 0.470000 | 2.874641 | 4.460423 | 0.600000 |

Interpretation:

- The partial-FOV override path is functional: the report records the narrower 35 degree FOV and all three paper arms complete.
- The full method improves network OSPA, localization disagreement, local E-OSPA, RMSE, and CardErr on this single trial.
- Cardinality dispersion is slightly worse for both projection arms. This is a useful boundary signal: under narrow FOV, label canonicalization can improve spatial coherence while not fully resolving cardinality disagreement.
- Reference-only is almost tied with tuned AA on RMSE, while the full method improves RMSE by 4.03%. This continues to support matched-state barycentering as the spatial-gain mechanism, but only as an N1 smoke signal.
- No parameter should be changed in response to this N1 result. The next evidence step is fixed N5, followed by N50 only if the N5 structure justifies the compute.

## Paper-facing 使用边界

这些场景族的首要价值是降低审稿风险，而不是把每个结果都塞进主文。若 N50 结果稳定，可以在主文 Discussion 中用一句话概括，并把完整表格放入 supplement 或 response-ready fragment。若结果 mixed，应进入 limitations 和 future recursive-online design discussion。
