# Results And Ablation

## Paper-Ready Results Draft

This section reports the current evidence supporting the paper's main claims. The strongest evidence comes from the tiered heterogeneous packet-loss GA scenario introduced in Section 6, so that scenario is treated as the primary result. Ideal-communication and communication-robustness experiments are used as supporting studies. The AA route and secondary consistency modules are kept as appendix-only context so that the main text stays centered on GA/KLA fusion.

### 1. Main Result In The Tiered-Drop GA Scenario

The main claim is that communication-aware adaptive weighting substantially improves distributed consensus quality in the eight-sensor dual-formation GA-LMB scenario under tiered heterogeneous packet loss. Starting from Fixed Metropolis, the Balanced mode combines covariance, realized link quality, existence confidence, and weak structure-aware decoupled KLA; the Cardinality-critical mode adds an FID-FIA-informed cue only on the existence branch. The latest 20-trial main comparison also includes FID-FIA-weighted GA, used as the information-geometric baseline, and the Balanced mode before the final existence-branch refinement. Fixed Metropolis yields OSPA consensus error `2.453677`, matched localization disagreement `2.335508`, and cardinality dispersion `0.714625`. FID-FIA-weighted GA yields `1.820229`, `1.647412`, and `0.126188`. The Balanced mode yields `1.785873`, `1.562521`, and `0.192938`. The Cardinality-critical mode yields `1.668961`, `1.528182`, and `0.061062`.

These reductions are sizeable in all three network disagreement metrics. Relative to Fixed Metropolis, FID-FIA-weighted GA lowers cardinality dispersion by `82.34%`, making it a strong cardinality-consensus baseline. The Balanced mode lowers OSPA consensus error by `27.22%` and matched localization disagreement by `33.10%`, making it the strongest pre-refinement spatial-consensus method. The Cardinality-critical mode lowers OSPA consensus error by `31.98%`, matched localization disagreement by `34.57%`, and cardinality dispersion by `91.46%`, making it the best arm on all three primary network disagreement metrics. This result is important because the method is not being evaluated in a benign homogeneous communication setting. The communication layer deliberately preserves the historical mean packet-loss rate while introducing persistent cross-node heterogeneity. The observed gain therefore supports the central paper claim that fusion weights should depend not only on posterior concentration and realized communication reliability, but also on existence decisiveness and target-pair information geometry.

The local truth-referenced metrics clarify the cardinality story. The Cardinality-critical mode has the lowest local cardinality error, `0.221563`, so its cardinality-consensus advantage is not merely agreement around a wrong target count. It also has the best local E-OSPA, `2.009084`. Its tradeoff is local RMSE, where it reaches `1.704538`, worse than the Balanced mode (`1.598561`) but still slightly better than FID-FIA-weighted GA (`1.715746`).

Computational cost is the third evaluation axis for this method family. A 3-trial runtime supplement in the same 100-step tiered-drop scenario times only the distributed LMB filtering/fusion call. Scenario generation, communication-model sampling, and metric evaluation are excluded. The result is:

| Arm | Filter runtime (s) | Runtime/step (s) | Relative to fixed |
|:----|-------------------:|-----------------:|------------------:|
| Fixed Metropolis | 47.334 +/- 0.732 | 0.473 | 1.000x |
| FID-FIA-weighted GA | 137.830 +/- 3.479 | 1.378 | 2.913x |
| Balanced mode | 55.603 +/- 5.057 | 0.556 | 1.174x |
| Cardinality-critical mode | 143.107 +/- 2.950 | 1.431 | 3.023x |

This should be presented as a usage-oriented method family rather than a contradiction. The `Balanced mode` is the position-sensitive and low-overhead mode, recommended when RMSE/spatial stability, runtime, or energy budget is the limiting requirement. The `Cardinality-critical mode` is the cardinality-sensitive mode, recommended when target-number agreement, missed tracks, or false support dominate the application risk and the deployment can tolerate roughly `3x` fixed-weight filtering/fusion time.

### 2. Factor Ablation Of The Main Adaptive Design

The most important ablation is the factor-by-factor path from fixed fusion to the Balanced mode, followed by the FID-FIA-informed existence refinement. The first five rows diagnose the communication-aware backbone; the last row shows the branch-specific existence refinement that converts the Balanced mode into the cardinality-sensitive mode.

| Arm | OSPA Consensus Error | Matched Localization Disagreement | Cardinality Dispersion |
|:----|---------------:|---------------:|----------------------:|
| Fixed Metropolis | 2.624065 | 2.702602 | 0.878750 |
| Covariance-only adaptive | 2.211513 | 2.410976 | 0.589500 |
| Covariance-link adaptive | 1.877771 | 1.800945 | 0.245250 |
| Three-factor adaptive backbone | 1.874840 | 1.779820 | 0.244500 |
| Balanced mode | 1.862244 | 1.749608 | 0.244250 |
| Cardinality-critical mode | 1.668961 | 1.528182 | 0.061062 |

Several conclusions follow from this table.

First, covariance weighting is necessary but not sufficient. Adding covariance alone already improves the fixed-weight baseline, which confirms that posterior concentration is a useful first-order proxy for local posterior quality. However, the covariance-only arm still leaves a large gap relative to the two branch-decoupled operating modes, especially in cardinality dispersion.

Second, realized link quality is the dominant communication-aware factor under heterogeneous packet loss. Once link quality is added on top of covariance, OSPA consensus error drops from `2.211513` to `1.877771`, matched localization disagreement drops from `2.410976` to `1.800945`, and cardinality dispersion drops from `0.589500` to `0.245250`. This is the largest single incremental gain after the initial covariance correction. The interpretation is straightforward: under the tiered-drop model, two nodes with similar sensing models do not necessarily provide equally useful information, because one node's posterior may be delivered much more reliably than another's.

Third, existence confidence is the most effective third factor. Adding existence confidence on top of covariance and link quality yields a smaller but consistent improvement in all three network disagreement metrics. This matters because covariance and link quality alone do not explicitly express whether a node is decisive about target existence. The existence-confidence term adds exactly that missing dimension, and its most visible effect is on cardinality-related agreement.

Finally, before the FID-FIA existence refinement is added, the best result in this factor ablation comes from the Balanced mode rather than from a strong topology-driven redesign. The gain from Three-factor adaptive backbone to the Balanced mode is modest but consistent: OSPA improves from `1.874840` to `1.862244`, matched localization disagreement improves from `1.779820` to `1.749608`, and cardinality dispersion improves from `0.244500` to `0.244250`. The final Cardinality-critical mode row then reduces the three network disagreement metrics to `1.668961`, `1.528182`, and `0.061062`. The correct interpretation is therefore not that topology or FID-FIA alone solves the problem. Rather, once the three-factor adaptive backbone is in place, a weak branch-specific structural modulation provides the spatial-side refinement, while FID-FIA is injected only into the existence/cardinality branch. This yields two operating modes: the Balanced mode for position-sensitive deployment and the Cardinality-critical mode for cardinality-sensitive deployment.

### 3. Ideal-Communication Supporting Evidence

An obvious question is whether the weak structure-aware decoupled refinement only helps because it compensates for packet loss. The ideal-communication experiment addresses this point by setting the communication level to `0` and comparing Fixed Metropolis, PD-weighted GA, FID-FIA-weighted GA, the Balanced mode, and the Cardinality-critical mode under otherwise matched conditions.

The result remains positive, and the FID-FIA arms now make the supporting story sharper. The table reports mean values, with relative changes versus Fixed Metropolis in parentheses.

| Arm | OSPA Consensus Error | Matched Localization Disagreement | Cardinality Dispersion | Local E-OSPA | Local RMSE |
|:----|---------------:|---------------:|---------------:|-------------:|-----------:|
| Fixed Metropolis | 1.634 | 1.381 | 0.090 | 1.908 | 1.442 |
| PD-weighted GA | 1.434 (-12.2%) | 1.218 (-11.8%) | 0.056 (-37.6%) | 1.825 (-4.3%) | 1.377 (-4.5%) |
| FID-FIA-weighted GA | 1.534 (-6.1%) | 1.333 (-3.5%) | 0.057 (-36.3%) | 1.881 (-1.4%) | 1.500 (+4.0%) |
| Balanced mode | **1.427 (-12.7%)** | **1.202 (-13.0%)** | 0.071 (-20.9%) | 1.839 (-3.6%) | **1.375 (-4.7%)** |
| Cardinality-critical mode | 1.433 (-12.3%) | 1.303 (-5.7%) | **0.049 (-46.0%)** | **1.766 (-7.4%)** | 1.470 (+2.0%) |

This mirrors the main tiered-drop result: FID-FIA is especially useful for cardinality, while the Balanced mode remains strongest on ideal-communication local RMSE. Aggregated over sensors and trials, local E-OSPA is best for the Cardinality-critical mode (`1.755551`), while local RMSE remains best for the Balanced mode (`1.371501`).

This supporting study is useful for two reasons. First, it shows that the refinement is not merely repairing a damaged communication channel. Second, it shows that the consensus gains are not being purchased by catastrophic degradation in local tracking quality. The ideal-communication study therefore strengthens the interpretation of the method family as a better distributed fusion rule rather than only a packet-loss compensator.

### 4. Communication-Robustness Trend

A compact communication-level probe now varies the communication level from `0` to `3` under the same eight-sensor dual-formation GA-LMB scenario and compares Fixed Metropolis with the Balanced mode over three deterministic paired trials per level. Level `0` is ideal communication, level `1` adds only the global bandwidth cap, level `2` adds the tiered fixed link-loss model used in the main experiment, and level `3` further adds random node outages.

| Level | Additional constraint | OSPA consensus error | Matched localization disagreement | Cardinality dispersion |
|-----:|:----------------------|------------------:|----------------------:|-------------------------:|
| 0 | none | `1.622 -> 1.405` | `1.380 -> 1.190` | `0.098 -> 0.070` |
| 1 | bandwidth cap | `1.734 -> 1.494` | `1.503 -> 1.243` | `0.152 -> 0.093` |
| 2 | tiered link loss | `2.480 -> 1.810` | `2.400 -> 1.419` | `0.686 -> 0.210` |
| 3 | node outage | `2.440 -> 1.753` | `2.486 -> 1.450` | `0.754 -> 0.193` |

This is supporting evidence rather than a replacement for the 20-trial main comparison, but it removes the previous purely qualitative gap. The Balanced mode improves all three network disagreement metrics at every communication level. The reductions are modest under ideal or bandwidth-only communication, but they become larger once link loss and outage make realized delivery quality heterogeneous. This is consistent with the factor ablation in the main scenario, where link-quality modeling provides the largest communication-specific gain.

### 5. Secondary Consistency Modules

Among the secondary modules, NIS-based consistency weighting is the most relevant one to report because it was explored seriously before the current four-factor main line was stabilized. The key comparison is `w/o NIS -> robust NIS -> plain NIS`. Across five trials, the corresponding network disagreement metrics are:

- OSPA: `1.909 -> 1.909 -> 2.008`
- RMSE: `2.934 -> 2.980 -> 3.173`
- cardinality dispersion: `0.267 -> 0.262 -> 0.300`

Two conclusions follow. Plain NIS is clearly harmful in the present setting. Robust NIS is more stable than plain NIS and slightly improves cardinality relative to `w/o NIS`, but it does not improve the full metric set and is effectively tied with the no-NIS baseline in OSPA. This is why NIS is no longer part of the main method story. It remains useful as appendix evidence because it documents that innovation consistency was explored, but the current evidence is weaker and more coupled than the evidence for covariance, link quality, existence confidence, and weak structure-aware decoupling.

### 6. Negative And Appendix-Only Ablations

Several additional modules were explored and are worth recording briefly because they justify the paper's selective emphasis.

Freshness weighting does not help in the current setting. Relative to the robust-NIS baseline, adding freshness changes OSPA from `1.909` to `1.910`, leaves RMSE essentially unchanged at `2.980`, and slightly worsens cardinality dispersion from `0.262` to `0.263`. The effect is negligible.

History weighting also fails to provide a convincing gain. In a twenty-trial comparison, OSPA changes from `1.811` to `1.814`, RMSE changes from `3.173` to `3.158`, and cardinality dispersion changes from `0.214` to `0.215`. This is too small and too mixed to justify a central role in the paper body.

A preliminary cardinality-consensus add-on was also tested in a one-trial pilot and degraded all three network disagreement metrics relative to the covariance-and-link baseline, moving OSPA from `1.909508` to `1.954855`, RMSE from `1.621662` to `1.791287`, and cardinality dispersion from `0.242500` to `0.285000`. Because this evidence is only a pilot and is uniformly negative, it is best treated as appendix-only context rather than as part of the main narrative.

Taken together, these negative ablations strengthen the paper rather than weaken it. They show that the design space was explored beyond the final chosen method, and they justify why the present paper is intentionally centered on the four effective ingredients: covariance, realized link quality, existence confidence, and weak structure-aware decoupling.

## Results Notes

- Keep the paper narrative centered on network disagreement metrics; they are the primary outcome for distributed fusion quality.
- Use local metrics mainly as supporting evidence that consensus gains do not come from local collapse.
- Treat runtime as a first-class efficiency metric for operating-mode selection, not as an optional implementation note.
- Present the AA route, NIS, freshness, and history studies as appendix material, not as co-equal method pillars.
- Avoid overclaiming statistical certainty from the current small trial counts; the wording should remain empirical and evidence-based.
