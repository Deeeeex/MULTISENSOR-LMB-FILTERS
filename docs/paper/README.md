# Paper Workspace

This directory organizes the current paper plan for the adaptive KLA/GA-LMB fusion story.

## Current Positioning

Recommended main line:

- Focus the paper on `GA-LMB / KLA` distributed fusion under communication constraints.
- Treat adaptive fusion-weight allocation as the main method.
- Treat decoupled `robust NIS` as the key technical point.
- Treat `history` and `association ambiguity` as ablations or extensions, not headline contributions.
- Treat `AA` as a secondary generalization experiment, not a co-equal main line.

## Strongest Supported Claims

- In the 4+4 GA formation scenario, adaptive weights substantially improve consensus quality.
- In the 20-trial GA ablation, `robust NIS` is more stable than plain `NIS`.
- The present evidence is strongest on consensus metrics, not on universally improving local tracking.

## Current Headline Numbers

- GA main scenario: consensus OSPA `2.203 -> 1.686`, consensus RMSE `1.942 -> 1.440`, consensus cardinality `0.604 -> 0.158`
- GA NIS ablation: consensus OSPA `1.811 -> 1.810 -> 1.901`, consensus RMSE `3.173 -> 3.153 -> 3.329`, consensus cardinality `0.214 -> 0.209 -> 0.234`
- AA three-wave scenario: consensus OSPA `4.349 -> 3.811`, consensus RMSE `19.098 -> 16.472`, consensus cardinality `0.421 -> 0.307`

## File Map

- `00_positioning.md`: paper scope, contribution boundaries, title direction
- `01_abstract_and_title.md`: abstract framing and title candidates
- `02_introduction.md`: introduction paragraph plan
- `03_related_work.md`: related-work buckets and differentiation
- `04_problem_formulation.md`: notation and system model checklist
- `05_method_adaptive_kla.md`: method section skeleton
- `06_experimental_setup.md`: scenario, metrics, baselines, implementation details
- `07_results_and_ablation.md`: result-section order and claim mapping
- `08_conclusion.md`: conclusion and discussion skeleton
- `09_figures_tables_and_gaps.md`: figure plan, table plan, and missing evidence

## Recommended Writing Order

1. `00_positioning.md`
2. `05_method_adaptive_kla.md`
3. `06_experimental_setup.md`
4. `07_results_and_ablation.md`
5. `01_abstract_and_title.md`
6. `02_introduction.md`
7. `03_related_work.md`
8. `04_problem_formulation.md`
9. `08_conclusion.md`

## Primary Source Files

- `multisensorLmb/computeAdaptiveFusionWeights.m`
- `multisensorLmb/generateLmbSensorAssociationMatrices.m`
- `multisensorLmb/runParallelUpdateLmbFilter.m`
- `multisensorLmb/gaLmbTrackMerging.m`
- `docs/ADAPTIVE_FUSION_WEIGHTS_CN.md`
- `docs/NIS_IMPLEMENTATION_AND_ANALYSIS_CN.md`
- `docs/FORMATION_4PLUS4_RUN.md`
- `RUN/GA/GA_NIS_COMPARE_20260309_164119.md`
- `RUN/GA/GA_NIS_GRID_20260309_163105.md`
- `RUN/GA/GA_HISTORY_COMPARE_20260309_113545.md`
- `RUN/GA/GA_ASSOCIATION_AMBIGUITY_COMPARE_20260321_131050.md`
- `docs/FORMATION_4PLUS4_THREEWAVES_AA_RUN.md`
- `analyzeCommunicationLevelImpact.m`
