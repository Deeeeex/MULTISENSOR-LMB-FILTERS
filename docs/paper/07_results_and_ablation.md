# Results And Ablation

## Recommended Section Order

1. Main GA consensus result
2. Robust NIS ablation
3. NIS parameter study
4. Communication robustness
5. AA generalization
6. Negative ablations

## Section 1: Main GA Result

Primary claim:

- Adaptive weights significantly improve distributed consensus quality in the main GA formation scenario.

Current headline result:

- consensus OSPA `2.203 -> 1.686`
- consensus RMSE `1.942 -> 1.440`
- consensus cardinality `0.604 -> 0.158`

Main file:

- `docs/FORMATION_4PLUS4_RUN.md`

## Section 2: Robust NIS Ablation

Primary claim:

- The best-supported method contribution is the decoupled `robust NIS` design.

Current headline result:

- consensus OSPA `1.811 -> 1.810 -> 1.901`
- consensus RMSE `3.173 -> 3.153 -> 3.329`
- consensus cardinality `0.214 -> 0.209 -> 0.234`

Interpretation:

- `robust NIS` preserves or slightly improves consensus relative to no NIS
- plain `NIS` is unstable and degrades all three consensus metrics

Main file:

- `RUN/GA/GA_NIS_COMPARE_20260309_164119.md`

## Section 3: Parameter Study

Primary claim:

- The chosen `robust NIS` configuration is not arbitrary.

Recommended focus:

- explain why `nisConsistencyConfidence = 0.7`
- explain why `nisPenaltyUpperScale = 6.0`

Main file:

- `RUN/GA/GA_NIS_GRID_20260309_163105.md`

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

- `history` gives little or inconsistent gain
- `association ambiguity` currently adds negligible gain

Main files:

- `RUN/GA/GA_HISTORY_COMPARE_20260309_113545.md`
- `RUN/GA/GA_ASSOCIATION_AMBIGUITY_COMPARE_20260321_131050.md`

## Writing Advice

- Keep the narrative centered on consensus metrics.
- Use local metrics mainly to show that consensus gains do not come from catastrophic local degradation.
- Negative results should strengthen the paper's credibility, not feel like distractions.
