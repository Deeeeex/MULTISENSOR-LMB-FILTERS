# Paper Workspace

This directory organizes the current paper-ready draft for the adaptive KLA/GA-LMB fusion story.

## Current Positioning

Recommended main line:

- Focus the paper on `GA-LMB / KLA` distributed fusion under communication constraints.
- Treat adaptive fusion-weight allocation as the main method.
- Treat `covariance + link quality + existence confidence` as the core factorized weighting design, with weak structure-aware decoupled KLA as the current best refinement.
- Treat decoupled `robust NIS` as a secondary consistency-analysis result, not a headline contribution.
- Treat `history`, `freshness`, `association ambiguity`, and `cardinality consensus` as appendix-style ablations or extensions, not headline contributions.
- Treat `AA` as a secondary generalization experiment, not a co-equal main line.

## Strongest Supported Claims

- In the tiered-drop 4+4 GA formation scenario, adaptive weights substantially improve consensus quality over fixed weights.
- Adding `existence confidence` on top of `covariance + link quality` further improves consensus OSPA, RMSE, and cardinality simultaneously.
- A weak structure-aware decoupled KLA refinement further improves consensus OSPA and RMSE while also slightly improving cardinality.
- Under ideal communication, the same structure-aware refinement still improves ordinary GA on both consensus metrics and local E-OSPA or RMSE.
- `robust NIS` is still more stable than plain `NIS`, but it is no longer the strongest main-line improvement in this branch.
- The present evidence is strongest on consensus metrics, not on universally improving local tracking.

## Current Headline Numbers

Core main-line numbers:

- Tiered GA main scenario: fixed -> full adaptive (`cov + link + existence + weak structure-aware decoupling`) gives consensus OSPA `2.624 -> 1.862`, consensus RMSE `2.703 -> 1.750`, consensus cardinality `0.879 -> 0.244`
- Tiered GA targeted improvement: `cov + link + existence` -> `cov + link + existence + weak structure-aware decoupled KLA` gives consensus OSPA `1.875 -> 1.862`, consensus RMSE `1.780 -> 1.750`, consensus cardinality `0.2445 -> 0.2443`
- Ideal-comm GA comparison: `ordinary GA` -> `structure-aware decoupled KLA` gives consensus OSPA `1.706 -> 1.494`, consensus RMSE `1.526 -> 1.290`, consensus cardinality `0.161 -> 0.139`, and local E-OSPA `1.950 -> 1.877`

Secondary or appendix-only numbers:

- GA NIS ablation: consensus OSPA `1.909 -> 1.909 -> 2.008`, consensus RMSE `2.934 -> 2.980 -> 3.173`, consensus cardinality `0.267 -> 0.262 -> 0.300`
- GA history ablation: consensus OSPA `1.811 -> 1.814`, consensus RMSE `3.173 -> 3.158`, consensus cardinality `0.214 -> 0.215`
- AA three-wave scenario: consensus OSPA `4.349 -> 3.811`, consensus RMSE `19.098 -> 16.472`, consensus cardinality `0.421 -> 0.307`

## Writing Rule For Main Text

Use the following narrative order in the paper body:

1. `fixed -> +covariance -> +link quality -> +existence confidence -> +weak structure-aware decoupled KLA`
2. ideal-communication supporting evidence
3. communication-robustness and secondary generalization

Keep `robust NIS`, `history`, `freshness`, and other weak or strongly coupled modules out of the main text unless they are needed as brief negative or appendix evidence.

## File Map

- `00_positioning.md`: paper scope, contribution boundaries, title direction
- `01_abstract_and_title.md`: abstract framing and title candidates
- `02_introduction.md`: paper-ready introduction draft
- `03_related_work.md`: paper-ready related-work draft with real references
- `04_problem_formulation.md`: paper-ready problem formulation
- `05_method_adaptive_kla.md`: paper-ready method section
- `06_experimental_setup.md`: scenario, metrics, baselines, implementation details
- `07_results_and_ablation.md`: paper-ready results and ablation section
- `08_conclusion.md`: paper-ready conclusion draft
- `09_figures_tables_and_gaps.md`: production checklist for figures, tables, and evidence gaps
- `Appendix_A_simulation_setup.md`: appendix-ready simulation and communication settings
- `Appendix_B_additional_attempts.md`: appendix-ready record of secondary module attempts and reserved result slots

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
10. `09_figures_tables_and_gaps.md`
11. `Appendix_A_simulation_setup.md`
12. `Appendix_B_additional_attempts.md`

## Primary Source Files

- `multisensorLmb/computeAdaptiveFusionWeights.m`
- `multisensorLmb/generateLmbSensorAssociationMatrices.m`
- `multisensorLmb/runParallelUpdateLmbFilter.m`
- `multisensorLmb/gaLmbTrackMerging.m`
- `docs/ADAPTIVE_FUSION_WEIGHTS_CN.md`
- `docs/NIS_IMPLEMENTATION_AND_ANALYSIS_CN.md`
- `docs/COMMUNICATION_TIERED_DROP_UPDATE_CN.md`
- `docs/FORMATION_4PLUS4_RUN.md`
- `docs/IDEAL_COMM_COMPARE_CN.md`
- `RUN/GA/GA_TIERED_LINK_ABLATION_20260326_182435.md`
- `RUN/GA/GA_TIERED_LINK_ABLATION_20260322_001613.md`
- `RUN/GA/GA_IDEAL_COMM_COMPARE_20260326_184508.md`
- `RUN/GA/GA_TIERED_LINK_NIS_COMPARE_20260321_193628.md`
- `RUN/GA/GA_HISTORY_COMPARE_20260309_113545.md`
- `RUN/GA/Del_GA_TIERED_LINK_FRESHNESS_COMPARE_20260321_193131.md`
- `RUN/GA/Del_GA_TIERED_LINK_ABLATION_20260321_234540.md`
- `RUN/GA/runMultisensorFilters_formation_4plus4_AssociationAmbiguityCompare.m`
- `RUN/GA/GA_ASSOCIATION_AMBIGUITY_COMPARE_20260408_003930.md`
- `RUN/GA/runMultisensorFilters_formation_4plus4_PosteriorStructureCompare.m`
- `RUN/GA/GA_POSTERIOR_STRUCTURE_COMPARE_20260408_002901.md`
- `docs/FORMATION_4PLUS4_THREEWAVES_AA_RUN.md`
- `analyzeCommunicationLevelImpact.m`
