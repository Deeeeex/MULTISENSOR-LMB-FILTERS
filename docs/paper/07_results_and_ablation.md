# Results And Ablation

## Recommended Section Order

1. Main GA consensus result
2. Factor ablation
3. Robust NIS ablation
4. Communication robustness
5. AA generalization
6. Negative ablations

## Section 1: Main GA Result

Primary claim:

- Adaptive weights significantly improve distributed consensus quality in the tiered-drop GA formation scenario.

Current headline result:

- fixed -> full adaptive (`cov + link + existence`)
- consensus OSPA `2.624 -> 1.875`
- consensus RMSE `2.703 -> 1.780`
- consensus cardinality `0.879 -> 0.245`

Main file:

- `RUN/GA/GA_TIERED_LINK_ABLATION_20260322_001613.md`

## Section 2: Factor Ablation

Primary claim:

- The best-supported method contribution is the three-factor adaptive weighting design: covariance, realized link quality, and existence confidence.

Current headline result:

- `+link quality`: OSPA `1.877771`, RMSE `1.800945`, cardinality `0.245250`
- `+existence confidence`: OSPA `1.874840`, RMSE `1.779820`, cardinality `0.244500`

Interpretation:

- `covariance` delivers the first major gain over fixed weights
- `link quality` delivers the largest additional gain under heterogeneous packet loss
- `existence confidence` provides a smaller but consistent improvement across all three consensus metrics

Main file:

- `RUN/GA/GA_TIERED_LINK_ABLATION_20260322_001613.md`

## Section 3: Robust NIS Ablation

Primary claim:

- `robust NIS` is better than plain `NIS`, but it is no longer the strongest main-line driver once tiered communication and existence confidence are adopted.

Recommended focus:

- show that `robust NIS` is close to `w/o NIS`
- show that plain `NIS` still degrades consensus
- position NIS as a secondary consistency module rather than the headline result

Main file:

- `RUN/GA/GA_TIERED_LINK_NIS_COMPARE_20260321_193628.md`

## Section 4: Communication Robustness

Primary claim:

- Adaptive weighting becomes more valuable as communication quality worsens.

Supporting evidence:

- communication-level analysis
- emphasize OSPA and cardinality trends

Main file:

- `analyzeCommunicationLevelImpact.m`

## Section 5: AA Generalization

Use this as a short extension section.

Primary claim:

- The weighting idea is not strictly limited to GA, though current strongest evidence remains on GA.

Current file:

- `docs/FORMATION_4PLUS4_THREEWAVES_AA_RUN.md`

## Section 6: Negative Ablations

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
