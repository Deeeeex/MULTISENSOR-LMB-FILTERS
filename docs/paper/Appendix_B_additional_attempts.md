# Appendix B. Additional Attempted Modules And Results

## Paper-Ready Appendix Draft

This appendix records the secondary modules explored in the current paper branch but not retained as part of the main claimed method. The purpose of this appendix is twofold. First, it documents that the design space was explored beyond the final four-factor main line. Second, it provides reserved result slots that can be filled after the corresponding reruns and report consolidation are completed.

In the present draft, the result tables below are intentionally left blank. Their structure is fixed now so that final numbers can be inserted later without rewriting the appendix.

### B.1 NIS-Based Consistency Weighting

This experiment studies whether innovation-based consistency information should enter the adaptive weight model. The relevant comparison is `w/o NIS -> robust NIS -> plain NIS`. In the current implementation, the NIS term is treated as a consistency penalty rather than a monotonic quality reward, because innovation consistency is structurally coupled with covariance-based posterior quality.

Relevant entry points and tracked artifacts:

- [runMultisensorFilters_formation_4plus4_TieredLinkNISCompare.m](/Users/dex/Desktop/Code/MULTISENSOR-LMB-FILTERS/RUN/GA/runMultisensorFilters_formation_4plus4_TieredLinkNISCompare.m)
- [GA_TIERED_LINK_NIS_COMPARE_20260321_193628.md](/Users/dex/Desktop/Code/MULTISENSOR-LMB-FILTERS/RUN/GA/GA_TIERED_LINK_NIS_COMPARE_20260321_193628.md)
- [runMultisensorFilters_formation_4plus4_NISGridSearch.m](/Users/dex/Desktop/Code/MULTISENSOR-LMB-FILTERS/RUN/GA/runMultisensorFilters_formation_4plus4_NISGridSearch.m)
- [GA_NIS_GRID_20260309_163105.md](/Users/dex/Desktop/Code/MULTISENSOR-LMB-FILTERS/RUN/GA/GA_NIS_GRID_20260309_163105.md)
- [generateLmbSensorAssociationMatrices.m](/Users/dex/Desktop/Code/MULTISENSOR-LMB-FILTERS/multisensorLmb/generateLmbSensorAssociationMatrices.m)

Reserved result table:

| Arm | Consensus OSPA | Consensus RMSE | Consensus Cardinality |
|:----|---------------:|---------------:|----------------------:|
| `w/o NIS` | `1.909` | `2.934` | `0.267` |
| `robust NIS` | `1.909` | `2.980` | `0.262` |
| `plain NIS` | `2.008` | `3.173` | `0.300` |

Current reading:

- keep this module as a secondary consistency analysis rather than a headline method component
- treat any final result here as supporting or appendix evidence

### B.2 Freshness Weighting

This experiment tests whether a recency-oriented score improves fusion beyond the robust-NIS baseline. The intended comparison is `robust NIS baseline -> robust NIS + freshness`.

Relevant entry points and tracked artifacts:

- [runMultisensorFilters_formation_4plus4_TieredLinkFreshnessCompare.m](/Users/dex/Desktop/Code/MULTISENSOR-LMB-FILTERS/RUN/GA/runMultisensorFilters_formation_4plus4_TieredLinkFreshnessCompare.m)
- [Del_GA_TIERED_LINK_FRESHNESS_COMPARE_20260321_193131.md](/Users/dex/Desktop/Code/MULTISENSOR-LMB-FILTERS/RUN/GA/Del_GA_TIERED_LINK_FRESHNESS_COMPARE_20260321_193131.md)
- [computeAdaptiveFusionWeights.m](/Users/dex/Desktop/Code/MULTISENSOR-LMB-FILTERS/multisensorLmb/computeAdaptiveFusionWeights.m)

Reserved result table:

| Arm | Consensus OSPA | Consensus RMSE | Consensus Cardinality |
|:----|---------------:|---------------:|----------------------:|
| `robust NIS baseline` | `1.909` | `2.980` | `0.262` |
| `robust NIS + freshness` | `1.910` | `2.980` | `0.263` |

Current reading:

- keep this module in appendix unless a rerun reveals a clearer and more stable gain

### B.3 History Weighting

This experiment evaluates whether a temporal-stability score improves fusion quality. The comparison is `w/o history -> history`, with the history term computed from temporally smoothed posterior-quality signals.

Relevant entry points and tracked artifacts:

- [runMultisensorFilters_formation_4plus4_HistoryCompare.m](/Users/dex/Desktop/Code/MULTISENSOR-LMB-FILTERS/RUN/GA/runMultisensorFilters_formation_4plus4_HistoryCompare.m)
- [GA_HISTORY_COMPARE_20260309_113545.md](/Users/dex/Desktop/Code/MULTISENSOR-LMB-FILTERS/RUN/GA/GA_HISTORY_COMPARE_20260309_113545.md)
- [computeAdaptiveFusionWeights.m](/Users/dex/Desktop/Code/MULTISENSOR-LMB-FILTERS/multisensorLmb/computeAdaptiveFusionWeights.m)

Reserved result table:

| Arm | Consensus OSPA | Consensus RMSE | Consensus Cardinality |
|:----|---------------:|---------------:|----------------------:|
| `w/o history` | `1.811` | `3.173` | `0.214` |
| `history` | `1.814` | `3.158` | `0.215` |

Current reading:

- keep this module secondary because its effect is coupled and currently not part of the main-line claim

### B.4 Cardinality-Consensus Weighting

This experiment tests whether a direct cardinality-consensus score should be included in the weight model. In the current branch, the cardinality add-on is treated as an exploratory negative ablation rather than a mature module.

Relevant entry points and tracked artifacts:

- [runMultisensorFilters_formation_4plus4_TieredLinkAblation.m](/Users/dex/Desktop/Code/MULTISENSOR-LMB-FILTERS/RUN/GA/runMultisensorFilters_formation_4plus4_TieredLinkAblation.m)
- [Del_GA_TIERED_LINK_ABLATION_20260321_234540.md](/Users/dex/Desktop/Code/MULTISENSOR-LMB-FILTERS/RUN/GA/Del_GA_TIERED_LINK_ABLATION_20260321_234540.md)
- [computeAdaptiveFusionWeights.m](/Users/dex/Desktop/Code/MULTISENSOR-LMB-FILTERS/multisensorLmb/computeAdaptiveFusionWeights.m)

Reserved result table:

| Arm | Consensus OSPA | Consensus RMSE | Consensus Cardinality |
|:----|---------------:|---------------:|----------------------:|
| `fixed weights` | `2.590531` | `2.268101` | `0.868750` |
| `+covariance` | `2.243220` | `1.774557` | `0.608750` |
| `+link quality` | `1.909508` | `1.621662` | `0.242500` |
| `+cardinality consensus` | `1.954855` | `1.791287` | `0.285000` |

Current reading:

- keep this result in appendix unless a future redesign removes the current degradation trend

### B.5 Association-Ambiguity Weighting

This experiment slot is reserved for ambiguity-aware weighting derived from data-association uncertainty. The motivation is that posterior quality may depend not only on covariance and existence decisiveness, but also on how ambiguous the underlying association structure is. In the current repository state, this idea has been discussed and scoped, but no stable tracked report artifact is available yet.

Relevant code context:

- [generateLmbSensorAssociationMatrices.m](/Users/dex/Desktop/Code/MULTISENSOR-LMB-FILTERS/multisensorLmb/generateLmbSensorAssociationMatrices.m)
- [computeAdaptiveFusionWeights.m](/Users/dex/Desktop/Code/MULTISENSOR-LMB-FILTERS/multisensorLmb/computeAdaptiveFusionWeights.m)
- [runMultisensorFilters_formation_4plus4_AssociationAmbiguityCompare.m](/Users/dex/Desktop/Code/MULTISENSOR-LMB-FILTERS/RUN/GA/runMultisensorFilters_formation_4plus4_AssociationAmbiguityCompare.m)
- [GA_ASSOCIATION_AMBIGUITY_COMPARE_20260408_003930.md](/Users/dex/Desktop/Code/MULTISENSOR-LMB-FILTERS/RUN/GA/GA_ASSOCIATION_AMBIGUITY_COMPARE_20260408_003930.md)

Reserved result table:

| Arm | Consensus OSPA | Consensus RMSE | Consensus Cardinality |
|:----|---------------:|---------------:|----------------------:|
| `baseline` | `1.874840` | `1.779820` | `0.244500` |
| `+association ambiguity` | `1.876368` | `1.769102` | `0.245500` |

Current reading:

- implementation and tracked report remain to be completed
- keep this item out of the paper body until a stable rerun artifact exists

### B.6 Posterior-Structure-Consistency Extension

This experiment slot is reserved for the stronger dynamic structure route in which structure scores are derived from pairwise posterior disagreement among neighboring nodes. The mechanism is already present as an optional mode in the adaptive-fusion implementation, but it is not part of the current best-performing main-line configuration.

Relevant code context:

- [computeAdaptiveFusionWeights.m](/Users/dex/Desktop/Code/MULTISENSOR-LMB-FILTERS/multisensorLmb/computeAdaptiveFusionWeights.m)
- [05_method_adaptive_kla.md](/Users/dex/Desktop/Code/MULTISENSOR-LMB-FILTERS/docs/paper/05_method_adaptive_kla.md)
- [09_figures_tables_and_gaps.md](/Users/dex/Desktop/Code/MULTISENSOR-LMB-FILTERS/docs/paper/09_figures_tables_and_gaps.md)
- [runMultisensorFilters_formation_4plus4_PosteriorStructureCompare.m](/Users/dex/Desktop/Code/MULTISENSOR-LMB-FILTERS/RUN/GA/runMultisensorFilters_formation_4plus4_PosteriorStructureCompare.m)
- [GA_POSTERIOR_STRUCTURE_COMPARE_20260408_002901.md](/Users/dex/Desktop/Code/MULTISENSOR-LMB-FILTERS/RUN/GA/GA_POSTERIOR_STRUCTURE_COMPARE_20260408_002901.md)

Reserved result table:

| Arm | Consensus OSPA | Consensus RMSE | Consensus Cardinality |
|:----|---------------:|---------------:|----------------------:|
| `static weak structure prior` | `1.862244` | `1.749608` | `0.244250` |
| `posterior-structure-consistency` | `1.862244` | `1.749608` | `0.244250` |

Current reading:

- keep this item as a future extension candidate rather than a present claim
- rerun and tracked report are still needed before any paper-level conclusion is made

### B.7 Use Of This Appendix In The Final Paper

The recommended use of this appendix is conservative:

- keep the main paper body centered on the covariance, link-quality, and existence-confidence backbone, branch decoupling, and the existence-branch FID-FIA cue
- use this appendix to show that alternative modules were considered and empirically bounded
- only promote an item from this appendix into the main paper body if a rerun yields stable gains together with a clean tracked report artifact
