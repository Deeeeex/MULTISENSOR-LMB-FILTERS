# Paper Workspace

This directory organizes the current paper-ready draft for the adaptive KLA/GA-LMB fusion story.

## Current Positioning

Current working title:

- `Communication-Aware Adaptive Weights for Consensus-Oriented Distributed KLA-Based LMB Fusion`

Recommended main line:

- Focus the paper on `GA-LMB / KLA` distributed fusion under communication constraints.
- Treat adaptive fusion-weight allocation as the main method.
- Treat covariance quality, link quality, and existence confidence as the core factorized weighting backbone. Present the method as a family with two operating modes: the Balanced mode for position-sensitive operation, and the Cardinality-critical mode for cardinality-sensitive operation.
- Treat decoupled `robust NIS` as a secondary consistency-analysis result, not a headline contribution.
- Treat `history`, `freshness`, `association ambiguity`, and `cardinality consensus` as appendix-style ablations or extensions, not headline contributions.
- Treat `AA` as an appendix-only secondary route, not a main-text result or co-equal line.

## Strongest Supported Claims

- In the tiered-drop 4+4 GA formation scenario, adaptive weights substantially improve consensus quality over Fixed Metropolis.
- The Balanced mode improves spatial consensus over Three-factor adaptive backbone, while the existence-branch FID-FIA cue supplies the final cardinality gain.
- Under ideal communication, the same branch-decoupled story remains useful: FID-FIA is strong for cardinality, the Balanced mode is strongest for RMSE, and the Cardinality-critical mode gives the best OSPA/cardinality without using FID-FIA as the whole-posterior rule.
- `robust NIS` is still more stable than plain `NIS`, but it is no longer the strongest main-line improvement in this branch.
- The present evidence is strongest on consensus metrics, not on universally improving local tracking.

## Current Headline Numbers

Core main-line numbers:

- Tiered GA main scenario: Fixed Metropolis -> Cardinality-critical mode gives consensus OSPA `2.454 -> 1.669`, consensus position disagreement `2.336 -> 1.528`, and consensus cardinality disagreement `0.715 -> 0.061`.
- Tiered GA local safeguards: Fixed Metropolis -> Cardinality-critical mode gives local E-OSPA `2.853 -> 2.009` and local cardinality error `1.455 -> 0.222`, while local RMSE remains slightly better than FID-FIA.
- Factor ablation: Three-factor adaptive backbone -> Balanced mode improves consensus OSPA `1.830 -> 1.821` and consensus position disagreement `1.754 -> 1.750`; the Cardinality-critical mode then reduces cardinality disagreement to `0.061`.
- Ideal-communication comparison: Ordinary GA -> Cardinality-critical mode gives consensus OSPA `1.704 -> 1.433`, consensus position disagreement `1.532 -> 1.309`, consensus cardinality disagreement `0.162 -> 0.050`, and local E-OSPA `1.963 -> 1.756`.

Secondary or appendix-only numbers:

- GA NIS ablation: consensus OSPA `1.909 -> 1.909 -> 2.008`, consensus position disagreement `2.934 -> 2.980 -> 3.173`, consensus cardinality disagreement `0.267 -> 0.262 -> 0.300`
- GA history ablation: consensus OSPA `1.811 -> 1.814`, consensus position disagreement `3.173 -> 3.158`, consensus cardinality disagreement `0.214 -> 0.215`
- Appendix-only AA three-wave scenario: consensus OSPA `4.349 -> 3.811`, consensus position disagreement `19.098 -> 16.472`, consensus cardinality disagreement `0.421 -> 0.307`

## Writing Rule For Main Text

Use the following narrative order in the paper body:

1. Fixed Metropolis -> Covariance-only adaptive -> Covariance-link adaptive -> Three-factor adaptive backbone -> Balanced mode -> Cardinality-critical mode
2. ideal-communication supporting evidence
3. communication-robustness as supporting evidence

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
- `figure_captions.md`: centralized concise captions for manuscript figures
- `figures/`: source and rendered figure assets, with scalar comparisons now represented as manuscript tables where clearer
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
- `RUN/GA/GA_TIERED_LINK_ABLATION_20260410_143517.md`
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
