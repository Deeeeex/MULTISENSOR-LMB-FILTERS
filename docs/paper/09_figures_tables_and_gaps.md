# Figures, Tables, And Gaps

## Paper-Ready Planning Draft

This document records the current figure plan, table plan, and evidence gaps for the paper workspace. It should be read as a practical production checklist rather than as a brainstorming note. The guiding rule is consistent with the main paper narrative: prioritize the tiered-drop GA result, the factor ablation, computational-cost reporting, and the ideal-communication supporting evidence; keep AA and secondary consistency modules in appendix-style material; and avoid reserving prominent space for modules that currently have weak or unstable evidence.

### 1. Figure Plan

#### Figure 1. System Overview And Fusion Pipeline

Purpose:

- show the peer-to-peer distributed tracking setting
- show local measurement update, neighborhood communication, and adaptive GA/KLA fusion
- visually separate the local posterior update from the adaptive fusion-weight allocation block

Recommended content:

- two four-sensor formations with local neighborhoods
- communication-constrained message exchange
- local LMB posterior at each node
- adaptive fusion block with branch outputs for spatial and existence fusion

Status:

- production prompt prepared in [figure1_system_overview_prompt.md](/Users/dex/Desktop/Code/MULTISENSOR-LMB-FILTERS/docs/paper/figures/figure1_system_overview_prompt.md)
- should be finalized in a diagram or illustration tool rather than extracted from simulation output

Priority:

- high

#### Figure 2. Adaptive Weight Factorization Diagram

Purpose:

- explain the main method compactly
- show that the claimed core method is the availability-masked covariance, link-quality, and existence-confidence backbone
- show that decoupled spatial and existence branches are refinements on top of the shared backbone

Recommended content:

- shared backbone: `mask`, `covScore`, `linkQuality`, `existenceConfidence`
- branch refinement: spatial-weight path and existence-weight path
- weak structure-aware branch prior after branch decoupling
- FID-FIA cue only on the existence-weight path
- EMA smoothing and minimum-weight safeguard as stabilization wrappers

Status:

- production prompt prepared in [figure2_weight_factorization_prompt.md](/Users/dex/Desktop/Code/MULTISENSOR-LMB-FILTERS/docs/paper/figures/figure2_weight_factorization_prompt.md)
- all required content is already stable in [05_method_adaptive_kla.md](/Users/dex/Desktop/Code/MULTISENSOR-LMB-FILTERS/docs/paper/05_method_adaptive_kla.md)

Priority:

- high

#### Figure 3. Existence-Confidence Mapping Curve

Purpose:

- explain why existence confidence adds information not captured by covariance or link quality
- visually show that decisiveness is high near `r = 0` and `r = 1`, and low near `r = 0.5`

Recommended content:

- plot of `c = |2r - 1|`
- optional second curve after bounded mapping with `existenceConfidenceMinScore` and `existenceConfidencePower`

Status:

- generated as a Python-rendered PDF artifact
- formula is stable and already documented in [05_method_adaptive_kla.md](/Users/dex/Desktop/Code/MULTISENSOR-LMB-FILTERS/docs/paper/05_method_adaptive_kla.md)
- current artifact: [figure3_existence_confidence_curve.pdf](/Users/dex/Desktop/Code/MULTISENSOR-LMB-FILTERS/docs/paper/figures/figure3_existence_confidence_curve.pdf)

Priority:

- medium-high

#### Figure 4. Main GA Consensus Result

Purpose:

- visualize the headline improvement in the primary tiered-drop GA scenario
- show that adaptive weighting improves consensus over time rather than only in a final scalar average

Recommended content:

- time-series curves for consensus OSPA
- optional companion panels for consensus position disagreement and consensus cardinality disagreement
- compare Fixed Metropolis against the existence-refined operating mode

Status:

- generated as a Python-rendered PDF artifact
- time-series export path now uses [exportFigure4ConsensusSeries.m](/Users/dex/Desktop/Code/MULTISENSOR-LMB-FILTERS/RUN/GA/exportFigure4ConsensusSeries.m)
- current artifact: [figure4_main_ga_consensus.pdf](/Users/dex/Desktop/Code/MULTISENSOR-LMB-FILTERS/docs/paper/figures/figure4_main_ga_consensus.pdf)

Priority:

- high

#### Table 3. Factor Ablation Under Tiered Packet Loss

Purpose:

- summarize the main factor-by-factor story compactly
- reinforce that the paper's main contribution is the four-step progression from fixed to weak structure-aware adaptive fusion

Recommended content:

- table rows: Fixed Metropolis, Covariance-only adaptive, Covariance-link adaptive, Three-factor adaptive backbone, Balanced mode, Cardinality-critical mode
- columns for consensus OSPA, consensus position disagreement, and consensus cardinality disagreement

Primary sources:

- [GA_TIERED_LINK_ABLATION_20260322_001613.md](/Users/dex/Desktop/Code/MULTISENSOR-LMB-FILTERS/RUN/GA/GA_TIERED_LINK_ABLATION_20260322_001613.md)
- [GA_TIERED_LINK_ABLATION_20260326_182435.md](/Users/dex/Desktop/Code/MULTISENSOR-LMB-FILTERS/RUN/GA/GA_TIERED_LINK_ABLATION_20260326_182435.md)

Status:

- represented in the manuscript as Table 3 rather than a figure, because the scalar differences are easier to read precisely in table form

Priority:

- highest

#### Table 4. Ideal-Communication Supporting Comparison

Purpose:

- support the claim that the final weak structure-aware refinement is not only compensating for packet loss

Recommended content:

- compact table with consensus OSPA, RMSE, cardinality, and local safeguards
- comparison: Ordinary GA, Balanced mode, FID-FIA baseline, Cardinality-critical mode

Primary source:

- [GA_IDEAL_COMM_COMPARE_20260326_184508.md](/Users/dex/Desktop/Code/MULTISENSOR-LMB-FILTERS/RUN/GA/GA_IDEAL_COMM_COMPARE_20260326_184508.md)

Status:

- represented in the manuscript as Table 4 rather than a bar chart, because the four-arm result is more informative as a compact quantitative comparison

Priority:

- medium-high

#### Optional Appendix Figure. Communication-Level Robustness

Purpose:

- show how fixed and adaptive fusion behave as communication quality degrades

Primary source:

- [analyzeCommunicationLevelImpact.m](/Users/dex/Desktop/Code/MULTISENSOR-LMB-FILTERS/analyzeCommunicationLevelImpact.m)

Status:

- partial
- analysis code exists, but there is no stable saved report or paper-ready result table under the current branch narrative

Priority:

- optional until rerun and exported cleanly

### 2. Table Plan

#### Table 1. Scenario And Parameter Configuration

Content:

- sensor count, target count, simulation length
- detection probability, clutter, measurement noise
- communication level and tiered drop configuration
- baseline and current best adaptive settings

Primary source:

- [06_experimental_setup.md](/Users/dex/Desktop/Code/MULTISENSOR-LMB-FILTERS/docs/paper/06_experimental_setup.md)

Status:

- ready to write

#### Table 2. Main GA Headline Result

Content:

- Fixed Metropolis versus the existence-refined operating mode
- local metrics and consensus metrics

Primary sources:

- [GA_TIERED_LINK_ABLATION_20260322_001613.md](/Users/dex/Desktop/Code/MULTISENSOR-LMB-FILTERS/RUN/GA/GA_TIERED_LINK_ABLATION_20260322_001613.md)
- [GA_TIERED_LINK_ABLATION_20260326_182435.md](/Users/dex/Desktop/Code/MULTISENSOR-LMB-FILTERS/RUN/GA/GA_TIERED_LINK_ABLATION_20260326_182435.md)
- [GA_TIERED_LINK_ABLATION_20260410_143517.md](/Users/dex/Desktop/Code/MULTISENSOR-LMB-FILTERS/RUN/GA/GA_TIERED_LINK_ABLATION_20260410_143517.md)

Status:

- ready
- consensus metrics and headline local metrics are now consolidated into one clean paper table

#### Table 3. Factor Ablation Under Tiered Packet Loss

Content:

- Fixed Metropolis
- Covariance-only adaptive
- Covariance-link adaptive
- Three-factor adaptive backbone
- Balanced mode
- Cardinality-critical mode

Primary sources:

- [GA_TIERED_LINK_ABLATION_20260322_001613.md](/Users/dex/Desktop/Code/MULTISENSOR-LMB-FILTERS/RUN/GA/GA_TIERED_LINK_ABLATION_20260322_001613.md)
- [GA_TIERED_LINK_ABLATION_20260326_182435.md](/Users/dex/Desktop/Code/MULTISENSOR-LMB-FILTERS/RUN/GA/GA_TIERED_LINK_ABLATION_20260326_182435.md)

Status:

- ready

#### Table 4. Ideal-Communication Supporting Comparison

Content:

- Ordinary GA, Balanced mode, FID-FIA baseline, and Cardinality-critical mode
- consensus OSPA, RMSE, cardinality disagreement
- aggregated local E-OSPA and RMSE safeguards

Primary source:

- [GA_IDEAL_COMM_COMPARE_20260326_184508.md](/Users/dex/Desktop/Code/MULTISENSOR-LMB-FILTERS/RUN/GA/GA_IDEAL_COMM_COMPARE_20260326_184508.md)

Status:

- ready

#### Table 5. Computational Cost In The Main Scenario

Content:

- Fixed Metropolis, FID-FIA baseline, Balanced mode, and Cardinality-critical mode
- filter/fusion runtime, runtime per simulation step, and runtime relative to Fixed Metropolis
- short interpretation of Balanced mode as the low-overhead option and Cardinality-critical mode as the higher-cost cardinality option

Primary source:

- [GA_TIERED_LINK_ABLATION_N3_SEED1_20260515_105137.md](/Users/dex/Desktop/Code/MULTISENSOR-LMB-FILTERS/RUN/GA/GA_TIERED_LINK_ABLATION_N3_SEED1_20260515_105137.md)

Status:

- ready as a main-text efficiency table

#### Appendix Table. AA-Based Secondary Route

Content:

- base AA versus adaptive AA
- consensus OSPA, RMSE, and cardinality disagreement
- optional note on average local metric behavior

Primary source:

- [FORMATION_4PLUS4_THREEWAVES_AA_RUN.md](/Users/dex/Desktop/Code/MULTISENSOR-LMB-FILTERS/docs/FORMATION_4PLUS4_THREEWAVES_AA_RUN.md)

Status:

- ready for a compact supporting table

#### Table 6. Secondary NIS Ablation

Content:

- `w/o NIS`
- `robust NIS`
- `plain NIS`

Primary source:

- [GA_TIERED_LINK_NIS_COMPARE_20260321_193628.md](/Users/dex/Desktop/Code/MULTISENSOR-LMB-FILTERS/RUN/GA/GA_TIERED_LINK_NIS_COMPARE_20260321_193628.md)

Status:

- ready

#### Table 7. Negative Or Appendix-Only Ablations

Content:

- freshness
- history
- preliminary cardinality-consensus add-on

Primary sources:

- [Del_GA_TIERED_LINK_FRESHNESS_COMPARE_20260321_193131.md](/Users/dex/Desktop/Code/MULTISENSOR-LMB-FILTERS/RUN/GA/Del_GA_TIERED_LINK_FRESHNESS_COMPARE_20260321_193131.md)
- [GA_HISTORY_COMPARE_20260309_113545.md](/Users/dex/Desktop/Code/MULTISENSOR-LMB-FILTERS/RUN/GA/GA_HISTORY_COMPARE_20260309_113545.md)
- [Del_GA_TIERED_LINK_ABLATION_20260321_234540.md](/Users/dex/Desktop/Code/MULTISENSOR-LMB-FILTERS/RUN/GA/Del_GA_TIERED_LINK_ABLATION_20260321_234540.md)

Status:

- ready as appendix material

Note:

- association-ambiguity weighting is intentionally not reserved a main table slot at present, because there is no stable tracked report artifact in the current repository state

#### Optional Appendix Table. Communication-Level Robustness

Content:

- communication levels `0` to `3`
- fixed versus adaptive fusion modes
- means and variance for OSPA and cardinality disagreement

Primary source:

- [analyzeCommunicationLevelImpact.m](/Users/dex/Desktop/Code/MULTISENSOR-LMB-FILTERS/analyzeCommunicationLevelImpact.m)

Status:

- not ready
- requires rerun, export, and consistency checks under the current paper branch

### 3. Current Evidence Gaps

#### Highest Priority

- increase the Monte Carlo strength of the final weak structure-aware result beyond the current five-trial headline evidence
- add variance, standard deviation, or confidence-interval reporting for the main GA tables
- consolidate local metrics for the final tiered-drop headline comparison into one clean saved artifact
- produce paper-ready plots for the main GA result and the factor ablation

#### Medium Priority

- rerun the communication-level study and export a stable table if it is to appear in the paper body or appendix
- keep the AA route entirely in a short appendix subsection
- add at least one stronger external baseline beyond fixed-weight versus adaptive internal ablations

#### Low Priority

- revisit posterior-structure-consistency only if a cleaner and less coupled result emerges
- revisit ambiguity-aware weighting only if a stable tracked report is reproduced
- expand the ideal-scenario visualization only if the main tiered-drop figures are already complete

### 4. Recommended Production Order

The current paper branch should produce figures and tables in the following order:

1. Table 3, the factor ablation under tiered packet loss
2. Table 5, the computational-cost table for operating-mode selection
3. Table 4, the ideal-communication supporting comparison
4. Figure 5, the tiered-drop factor ablation figure
5. Figure 4, the main GA consensus-over-time figure
6. Table 1 and Table 2
7. Table 6 and Table 7 as appendix support
8. the optional communication-level appendix material if time permits

This order matches the current evidence hierarchy and minimizes the risk of spending time on low-priority material before the core story is fully stabilized.

### 5. Practical Next Steps

1. Export a clean paper table for the five-arm tiered GA ablation.
2. Consolidate the ideal-communication comparison into a paper table and one compact figure.
3. Add a saved artifact for local metrics in the main tiered-drop headline comparison.
4. Generate the method schematic and adaptive-weight factorization figure.
5. Decide whether communication-level robustness will be rerun for the main submission or deferred to appendix-only status.
