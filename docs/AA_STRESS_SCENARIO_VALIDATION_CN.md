# AA stress scenario validation protocol

最后更新: 2026-06-24 15:36 CST

## 目的

当前 TAES 稿件的主证据已经覆盖 baseSeed=1 的 50-trial tiered packet-loss validation、baseSeed=11 的 held-out 50-trial robustness check、reference-only ablation、GA contextual rows、runtime 和 independent verifier。剩余审稿风险主要不是当前场景的统计量，而是场景族覆盖仍集中在一个 tiered packet-loss formation profile。

本协议用于补充一个更强 packet-loss stress family。它不是新的调参入口，也不用于选择 `H`、existence threshold、projection cutoff、barycenter weights 或 trial-specific label rules。若 stress result 为 mixed 或 negative，应作为边界/limitations 证据处理，而不是回头改参数直到结果变好。

## 固定方法参数

与 TAES 主实验保持一致:

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
- No per-scenario search over projection cutoff, barycenter weights, label rules, or thresholds.

## Stress communication profile

The stress profile changes only the packet-loss level distribution:

```text
pDropLevels      = [0.2 0.35 0.5 0.7]
pDropLevelCounts = [1 3 2 2]
```

Compared with the main profile `[0 0.1 0.2 0.5] / [1 4 1 2]`, every sensor class is shifted to a harsher communication regime. The target formation, sensor motion, tracker controls, and method parameters remain the same.

## N1 smoke result

Smoke command was run on 2026-06-24 with `numberOfTrials=1`, `baseSeed=21`, and trial seed `22`.

Artifacts:

- Report: `RUN/AA/AA_BALANCED_CARDINALITY_VALIDATION_N1_SEED21_20260624_130131.md`
- Log: `RUN/AA/AA_TAES_STRESS_HARSH_N1_SMOKE_20260624_130130.log`

Smoke result:

| Arm | Net OSPA | Loc. disag. | Card. disp. | E-OSPA | RMSE | CardErr |
| --- | ---: | ---: | ---: | ---: | ---: | ---: |
| Tuned spatial-KLA AA | 2.101216 | 2.492763 | 0.093750 | 2.438530 | 3.951127 | 0.173750 |
| Neighborhood label-barycenter | 0.466321 | 0.348431 | 0.045000 | 2.066908 | 4.011443 | 0.135000 |
| Neighborhood reference-only | 1.222720 | 1.658173 | 0.045000 | 2.276603 | 3.956631 | 0.135000 |

Interpretation:

- The report structure is compatible with the existing evidence parser style: it contains per-trial network rows, local tracking rows, paired reduction tables, and runtime.
- The full method still strongly reduces network disagreement and improves local E-OSPA/CardErr on this single stress trial.
- RMSE is mixed in the smoke result: full barycenter is slightly worse than tuned AA, and reference-only is nearly tied. This is not a failure of the protocol; it is a useful warning that the harsher stress family may expose a localization boundary. A formal N50 run is needed before deciding whether this belongs in the manuscript, supplement, or limitation-only evidence package.

## Formal N50 result

Formal N50 completed on 2026-06-24 with `numberOfTrials=50`, `baseSeed=21`, and trial seeds `22..71`.

Artifacts:

- Report: `RUN/AA/AA_BALANCED_CARDINALITY_VALIDATION_N50_SEED21_20260624_130730.md`
- Log: `RUN/AA/AA_TAES_STRESS_HARSH_N50_BASESEED21_20260624_130728.log`
- Generated manifest: `docs/paper/taes/manuscript/generated/STRESS_HARSH_MANIFEST.md`
- Generated response-ready fragment: `docs/paper/taes/manuscript/generated/stress_harsh_section.tex`
- Generated manuscript sentence: `docs/paper/taes/manuscript/generated/stress_harsh_summary_sentence.tex`

Formal result:

| Arm | Net OSPA | Loc. disag. | Card. disp. | E-OSPA | RMSE | CardErr |
| --- | ---: | ---: | ---: | ---: | ---: | ---: |
| Tuned spatial-KLA AA | 2.224259 | 2.127170 | 0.087475 | 2.523393 | 4.235071 | 0.146025 |
| Neighborhood label-barycenter | 0.454475 | 0.299389 | 0.044100 | 2.092956 | 3.897040 | 0.110500 |
| Neighborhood reference-only | 1.262520 | 1.152782 | 0.044100 | 2.354646 | 4.130316 | 0.110500 |

Paired reductions relative to tuned AA:

| Metric | Full reduction | Full wins | Ref.-only reduction | Ref.-only wins |
| --- | ---: | ---: | ---: | ---: |
| Network OSPA | 79.57% | 50/50 | 43.24% | 50/50 |
| Local E-OSPA | 17.06% | 50/50 | 6.69% | 50/50 |
| Local RMSE | 7.98% | 48/50 | 2.47% | 38/50 |
| CardErr | 24.33% | 50/50 | 24.33% | 50/50 |

Interpretation:

- The formal N50 reverses the single-trial RMSE warning: under the harsher packet-loss family, full label-barycenter still improves RMSE and keeps a larger RMSE gain than reference-only.
- The packet-loss stress result should strengthen the manuscript as fixed-design robustness evidence, not as a new tuning target.
- This closes a broader packet-loss-profile check, but it does not close topology, partial-field-of-view, maneuver, or recursive-online implementation risks.

## Formal run launcher

Launcher:

```bash
RUN/AA/launchAaTaesHarshLossN50BaseSeed21.sh
```

Defaults:

- `AA_STRESS_TRIALS=50`
- `AA_STRESS_BASE_SEED=21`
- `AA_ALLOW_CONCURRENT=0`

The launcher writes a durable log and pid file:

```text
RUN/AA/AA_TAES_STRESS_HARSH_N{N}_BASESEED{baseSeed}_{timestamp}.log
RUN/AA/AA_TAES_STRESS_HARSH_N{N}_BASESEED{baseSeed}_{timestamp}.pid
```

For a quicker gate before N50:

```bash
AA_STRESS_TRIALS=5 RUN/AA/launchAaTaesHarshLossN50BaseSeed21.sh
```

## Manuscript evidence extraction

The TAES build now has a non-blocking stress evidence path:

```text
docs/paper/taes/manuscript/scripts/extract_stress_evidence.py
```

By default, this script does nothing except remove stale stress-generated files. After a formal harsh-loss N50 report exists, add its Markdown path to:

```text
docs/paper/taes/manuscript/evidence_sources.json
```

using the key:

```json
"stress_harsh_n50_report": "RUN/AA/<completed-harsh-loss-N50-report>.md"
```

Then run:

```bash
cd docs/paper/taes/manuscript
./build.sh
```

The build will generate:

- `generated/STRESS_HARSH_MANIFEST.md`
- `generated/stress_harsh_evidence.json`
- `generated/stress_harsh_section.tex`

The LaTeX table fragment is response-ready but is not imported by `main.tex` by default. The build also generates `stress_harsh_summary_sentence.tex`, which is imported into Discussion when `stress_harsh_n50_report` is configured. This keeps the current manuscript compact while still making the full stress table easy to use in a supplement or response package.

## Paper-facing decision rule

Use the same interpretation hierarchy as the main N50 evidence:

1. Network disagreement metrics test whether the graph-local label projection still improves cross-sensor output agreement.
2. Local E-OSPA/CardErr test whether agreement translates to finite-set tracking quality.
3. RMSE tests whether matched posterior barycenters improve localization rather than only cardinality/label coherence.
4. Reference-only is the mechanism control. If full and reference-only become similar under harsh loss, the result should be described as a boundary of matched-state averaging under severe missing information.

No method parameter should be changed in response to this stress result without opening a separate method-design branch and rerunning the full main/held-out evidence chain.
