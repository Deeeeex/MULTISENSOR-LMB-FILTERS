# Results And Ablation

## Paper-Ready Results Draft

This section reports the current evidence supporting the paper's main claims. The strongest evidence comes from the tiered heterogeneous packet-loss GA scenario introduced in Section 6, so that scenario is treated as the primary result. Ideal-communication and communication-robustness experiments are used as supporting studies. The AA route and secondary consistency modules are kept as appendix-only context so that the main text stays centered on GA/KLA fusion.

### 1. Main Result In The Tiered-Drop GA Scenario

The main claim is that communication-aware adaptive weighting substantially improves distributed consensus quality in the eight-sensor dual-formation GA-LMB scenario under tiered heterogeneous packet loss. Starting from Fixed Metropolis, the Balanced mode combines covariance, realized link quality, existence confidence, and weak structure-aware decoupled KLA; the Cardinality-critical mode adds an FID-FIA-informed cue only on the existence branch. The latest 20-trial main comparison also includes a FID-FIA baseline and the Balanced mode before the final existence-branch refinement. Fixed Metropolis yields consensus OSPA `2.453677`, consensus position disagreement `2.335508`, and consensus cardinality disagreement `0.714625`. The FID-FIA baseline yields `1.820229`, `1.647412`, and `0.126188`. The Balanced mode yields `1.785873`, `1.562521`, and `0.192938`. The Cardinality-critical mode yields `1.668961`, `1.528182`, and `0.061062`.

These reductions are sizeable in all three consensus metrics. Relative to Fixed Metropolis, the FID-FIA baseline lowers consensus cardinality disagreement by `82.34%`, making it a strong cardinality-consensus baseline. The Balanced mode lowers consensus OSPA by `27.22%` and consensus position disagreement by `33.10%`, making it the strongest pre-refinement spatial-consensus method. The Cardinality-critical mode lowers consensus OSPA by `31.98%`, consensus position disagreement by `34.57%`, and consensus cardinality disagreement by `91.46%`, making it the best arm on all three primary consensus metrics. This result is important because the method is not being evaluated in a benign homogeneous communication setting. The communication layer deliberately preserves the historical mean packet-loss rate while introducing persistent cross-node heterogeneity. The observed gain therefore supports the central paper claim that fusion weights should depend not only on posterior concentration and realized communication reliability, but also on existence decisiveness and target-pair information geometry.

The local truth-referenced metrics clarify the cardinality story. The Cardinality-critical mode has the lowest local cardinality error, `0.221563`, so its cardinality-consensus advantage is not merely agreement around a wrong target count. It also has the best local E-OSPA, `2.009084`. Its tradeoff is local RMSE, where it reaches `1.704538`, worse than the Balanced mode (`1.598561`) but still slightly better than the FID-FIA baseline (`1.715746`).

Computational cost is the third evaluation axis for this method family. A 3-trial runtime supplement in the same 100-step tiered-drop scenario times only the distributed LMB filtering/fusion call. Scenario generation, communication-model sampling, and metric evaluation are excluded. The result is:

| Arm | Filter runtime (s) | Runtime/step (s) | Relative to fixed |
|:----|-------------------:|-----------------:|------------------:|
| Fixed Metropolis | 47.334 +/- 0.732 | 0.473 | 1.000x |
| FID-FIA baseline | 137.830 +/- 3.479 | 1.378 | 2.913x |
| Balanced mode | 55.603 +/- 5.057 | 0.556 | 1.174x |
| Cardinality-critical mode | 143.107 +/- 2.950 | 1.431 | 3.023x |

This should be presented as a usage-oriented method family rather than a contradiction. The `Balanced mode` is the position-sensitive and low-overhead mode, recommended when RMSE/spatial stability, runtime, or energy budget is the limiting requirement. The `Cardinality-critical mode` is the cardinality-sensitive mode, recommended when target-number agreement, missed tracks, or false support dominate the application risk and the deployment can tolerate roughly `3x` fixed-weight filtering/fusion time.

### 2. Factor Ablation Of The Main Adaptive Design

The most important ablation is the factor-by-factor path from fixed fusion to the Balanced mode, followed by the FID-FIA-informed existence refinement. The first five rows diagnose the communication-aware backbone; the last row shows the branch-specific existence refinement that converts the Balanced mode into the cardinality-sensitive mode.

| Arm | Consensus OSPA | Consensus Position Disagreement | Consensus Cardinality Disagreement |
|:----|---------------:|---------------:|----------------------:|
| Fixed Metropolis | 2.624065 | 2.702602 | 0.878750 |
| Covariance-only adaptive | 2.211513 | 2.410976 | 0.589500 |
| Covariance-link adaptive | 1.877771 | 1.800945 | 0.245250 |
| Three-factor adaptive backbone | 1.874840 | 1.779820 | 0.244500 |
| Balanced mode | 1.862244 | 1.749608 | 0.244250 |
| Cardinality-critical mode | 1.668961 | 1.528182 | 0.061062 |

Several conclusions follow from this table.

First, covariance weighting is necessary but not sufficient. Adding covariance alone already improves the fixed-weight baseline, which confirms that posterior concentration is a useful first-order proxy for local posterior quality. However, the covariance-only arm still leaves a large gap relative to the two branch-decoupled operating modes, especially in cardinality disagreement.

Second, realized link quality is the dominant communication-aware factor under heterogeneous packet loss. Once link quality is added on top of covariance, consensus OSPA drops from `2.211513` to `1.877771`, consensus position disagreement drops from `2.410976` to `1.800945`, and consensus cardinality disagreement drops from `0.589500` to `0.245250`. This is the largest single incremental gain after the initial covariance correction. The interpretation is straightforward: under the tiered-drop model, two nodes with similar sensing models do not necessarily provide equally useful information, because one node's posterior may be delivered much more reliably than another's.

Third, existence confidence is the most effective third factor. Adding existence confidence on top of covariance and link quality yields a smaller but consistent improvement in all three consensus metrics. This matters because covariance and link quality alone do not explicitly express whether a node is decisive about target existence. The existence-confidence term adds exactly that missing dimension, and its most visible effect is on cardinality-related agreement.

Finally, before the FID-FIA existence refinement is added, the best result in this factor ablation comes from the Balanced mode rather than from a strong topology-driven redesign. The gain from Three-factor adaptive backbone to the Balanced mode is modest but consistent: OSPA improves from `1.874840` to `1.862244`, position disagreement improves from `1.779820` to `1.749608`, and cardinality disagreement improves from `0.244500` to `0.244250`. The final Cardinality-critical mode row then reduces the three consensus metrics to `1.668961`, `1.528182`, and `0.061062`. The correct interpretation is therefore not that topology or FID-FIA alone solves the problem. Rather, once the three-factor adaptive backbone is in place, a weak branch-specific structural modulation provides the spatial-side refinement, while FID-FIA is injected only into the existence/cardinality branch. This yields two operating modes: the Balanced mode for position-sensitive deployment and the Cardinality-critical mode for cardinality-sensitive deployment.

### 3. Ideal-Communication Supporting Evidence

An obvious question is whether the weak structure-aware decoupled refinement only helps because it compensates for packet loss. The ideal-communication experiment addresses this point by setting the communication level to `0` and comparing Ordinary GA, the Balanced mode, the FID-FIA baseline, and the Cardinality-critical mode under otherwise matched conditions.

The result remains positive, and the FID-FIA arms now make the supporting story sharper. The table reports mean values, with relative changes versus ordinary GA in parentheses.

| Arm | Consensus OSPA | Consensus Position Disagreement | Consensus Cardinality Disagreement | Local E-OSPA | Local RMSE |
|:----|---------------:|---------------:|---------------:|-------------:|-----------:|
| Ordinary GA | 1.704 | 1.532 | 0.162 | 1.963 | 1.445 |
| Balanced mode | 1.482 (-13.0%) | **1.238 (-19.2%)** | 0.132 (-18.2%) | 1.885 (-4.0%) | **1.372 (-5.1%)** |
| FID-FIA baseline | 1.532 (-10.1%) | 1.332 (-13.1%) | 0.054 (-66.5%) | 1.871 (-4.7%) | 1.499 (+3.8%) |
| Cardinality-critical mode | **1.433 (-15.9%)** | 1.309 (-14.6%) | **0.050 (-69.1%)** | **1.756 (-10.6%)** | 1.493 (+3.3%) |

This mirrors the main tiered-drop result: FID-FIA is especially useful for cardinality, while the Balanced mode remains strongest on ideal-communication local RMSE. Aggregated over sensors and trials, local E-OSPA is best for the Cardinality-critical mode (`1.755551`), while local RMSE remains best for the Balanced mode (`1.371501`).

This supporting study is useful for two reasons. First, it shows that the refinement is not merely repairing a damaged communication channel. Second, it shows that the consensus gains are not being purchased by catastrophic degradation in local tracking quality. The ideal-communication study therefore strengthens the interpretation of the method family as a better distributed fusion rule rather than only a packet-loss compensator.

### 4. Communication-Robustness Trend

A broader communication-level analysis has also been implemented by varying the communication level from `0` to `3` and comparing fixed and adaptive fusion modes through [analyzeCommunicationLevelImpact.m](/Users/dex/Desktop/Code/MULTISENSOR-LMB-FILTERS/analyzeCommunicationLevelImpact.m). At present, this study is best treated as supporting evidence rather than as a headline quantitative table, because the tiered-drop branch has much cleaner and more directly reproducible saved reports.

The present qualitative conclusion is nevertheless clear: as communication becomes more constrained and heterogeneous, adaptive weighting becomes more valuable. This trend is consistent with the factor ablation in the main scenario, where link-quality modeling provides the largest communication-specific gain. For the final paper version, the communication-level study should be rerun under the same reporting discipline as the main tiered-drop experiment so that a clean table of means and variances can be added.

### 5. Secondary Consistency Modules

Among the secondary modules, NIS-based consistency weighting is the most relevant one to report because it was explored seriously before the current four-factor main line was stabilized. The key comparison is `w/o NIS -> robust NIS -> plain NIS`. Across five trials, the corresponding consensus metrics are:

- OSPA: `1.909 -> 1.909 -> 2.008`
- RMSE: `2.934 -> 2.980 -> 3.173`
- cardinality disagreement: `0.267 -> 0.262 -> 0.300`

Two conclusions follow. Plain NIS is clearly harmful in the present setting. Robust NIS is more stable than plain NIS and slightly improves cardinality relative to `w/o NIS`, but it does not improve the full metric set and is effectively tied with the no-NIS baseline in OSPA. This is why NIS is no longer part of the main method story. It remains useful as appendix evidence because it documents that innovation consistency was explored, but the current evidence is weaker and more coupled than the evidence for covariance, link quality, existence confidence, and weak structure-aware decoupling.

### 6. Negative And Appendix-Only Ablations

Several additional modules were explored and are worth recording briefly because they justify the paper's selective emphasis.

Freshness weighting does not help in the current setting. Relative to the robust-NIS baseline, adding freshness changes OSPA from `1.909` to `1.910`, leaves RMSE essentially unchanged at `2.980`, and slightly worsens cardinality disagreement from `0.262` to `0.263`. The effect is negligible.

History weighting also fails to provide a convincing gain. In a twenty-trial comparison, OSPA changes from `1.811` to `1.814`, RMSE changes from `3.173` to `3.158`, and cardinality disagreement changes from `0.214` to `0.215`. This is too small and too mixed to justify a central role in the paper body.

A preliminary cardinality-consensus add-on was also tested in a one-trial pilot and degraded all three consensus metrics relative to the covariance-and-link baseline, moving OSPA from `1.909508` to `1.954855`, RMSE from `1.621662` to `1.791287`, and cardinality disagreement from `0.242500` to `0.285000`. Because this evidence is only a pilot and is uniformly negative, it is best treated as appendix-only context rather than as part of the main narrative.

Taken together, these negative ablations strengthen the paper rather than weaken it. They show that the design space was explored beyond the final chosen method, and they justify why the present paper is intentionally centered on the four effective ingredients: covariance, realized link quality, existence confidence, and weak structure-aware decoupling.

## Results Notes

- Keep the paper narrative centered on consensus metrics; they are the primary outcome for distributed fusion quality.
- Use local metrics mainly as supporting evidence that consensus gains do not come from local collapse.
- Treat runtime as a first-class efficiency metric for operating-mode selection, not as an optional implementation note.
- Present the AA route, NIS, freshness, and history studies as appendix material, not as co-equal method pillars.
- Avoid overclaiming statistical certainty from the current small trial counts; the wording should remain empirical and evidence-based.
