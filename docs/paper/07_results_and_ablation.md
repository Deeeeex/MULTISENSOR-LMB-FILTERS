# Results And Ablation

## Recommended Section Order

1. Main GA consensus result
2. Factor ablation
3. Robust NIS ablation
4. Ideal communication comparison
5. Communication robustness
6. AA generalization
7. Negative ablations

## Section 1: Main GA Result

Primary claim:

- Adaptive weights significantly improve distributed consensus quality in the tiered-drop GA formation scenario.

Current headline result:

- fixed -> full adaptive (`cov + link + existence + weak structure-aware decoupling`)
- consensus OSPA `2.624 -> 1.862`
- consensus RMSE `2.703 -> 1.750`
- consensus cardinality `0.879 -> 0.244`

Main file:

- `RUN/GA/GA_TIERED_LINK_ABLATION_20260326_182435.md`

## Section 2: Factor Ablation

Primary claim:

- The best-supported method contribution is a three-factor adaptive weighting design refined by a weak structure-aware decoupled KLA update.

Current headline result:

- `+link quality`: OSPA `1.877771`, RMSE `1.800945`, cardinality `0.245250`
- `+structure-aware decoupled KLA`: OSPA `1.862244`, RMSE `1.749608`, cardinality `0.244250`

Interpretation:

- `covariance` delivers the first major gain over fixed weights
- `link quality` delivers the largest additional gain under heterogeneous packet loss
- `existence confidence` remains the key third factor that stabilizes cardinality
- a weak structure-aware decoupled refinement then provides the current best OSPA and RMSE while also slightly improving cardinality

Main file:

- `RUN/GA/GA_TIERED_LINK_ABLATION_20260326_182435.md`

## Section 3: Robust NIS Ablation

Primary claim:

- `robust NIS` is better than plain `NIS`, but it is no longer the strongest main-line driver once tiered communication, existence confidence, and weak structure-aware decoupling are adopted.

Recommended focus:

- show that `robust NIS` is close to `w/o NIS`
- show that plain `NIS` still degrades consensus
- position NIS as a secondary consistency module rather than the headline result

Main file:

- `RUN/GA/GA_TIERED_LINK_NIS_COMPARE_20260321_193628.md`

## Section 4: Ideal Communication Comparison

Primary claim:

- The structure-aware refinement is not only a communication-loss compensation trick; it also improves distributed GA under ideal communication.

Supporting evidence:

- ordinary GA -> structure-aware decoupled KLA
- consensus OSPA `1.706 -> 1.494`
- consensus RMSE `1.526 -> 1.290`
- consensus cardinality `0.161 -> 0.139`
- local E-OSPA `1.950 -> 1.877`
- local RMSE `1.442 -> 1.369`
- local H-OSPA remains essentially unchanged at `0.500`

Main file:

- `RUN/GA/GA_IDEAL_COMM_COMPARE_20260326_184508.md`

## Section 5: Communication Robustness

Primary claim:

- Adaptive weighting becomes more valuable as communication quality worsens.

Supporting evidence:

- communication-level analysis
- emphasize OSPA and cardinality trends

Main file:

- `analyzeCommunicationLevelImpact.m`

## Section 6: AA Generalization

Use this as a short extension section.

Primary claim:

- The weighting idea is not strictly limited to GA, though current strongest evidence remains on GA.

Current file:

- `docs/FORMATION_4PLUS4_THREEWAVES_AA_RUN.md`

## Section 7: Negative Ablations

Use short subsections:

- `freshness` gives negligible gain
- `history` gives little or inconsistent gain
- `association ambiguity` currently adds negligible gain
- `cardinality consensus` currently degrades all three metrics

Main files:

- `RUN/GA/Del_GA_TIERED_LINK_FRESHNESS_COMPARE_20260321_193131.md`
- `RUN/GA/Del_GA_TIERED_LINK_ABLATION_20260321_234540.md`
- `RUN/GA/GA_HISTORY_COMPARE_20260309_113545.md`
- `RUN/GA/GA_ASSOCIATION_AMBIGUITY_COMPARE_20260321_131050.md`

## Writing Advice

- Keep the narrative centered on consensus metrics.
- Use local metrics mainly to show that consensus gains do not come from catastrophic local degradation.
- Negative results should strengthen the paper's credibility, not feel like distractions.
