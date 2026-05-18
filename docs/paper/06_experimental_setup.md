# Experimental Setup

## Paper-Ready Experimental-Setup Draft

This section defines the evaluation protocol used to support the current paper narrative. The experiments are organized around one main scenario and a small set of supporting studies. The main scenario is the dual-formation eight-sensor distributed GA-LMB tracking problem under tiered heterogeneous packet loss. Supporting studies are used to check whether the observed gains persist under ideal communication, how the method behaves as communication becomes more constrained, and whether several optional modules should remain outside the core method.

### 1. Evaluation Hierarchy

The current evaluation is intentionally hierarchical.

Main experiment:

- dual-formation eight-sensor distributed GA-LMB tracking under tiered heterogeneous packet loss
- factor ablation from Fixed Metropolis to the Balanced mode and the Cardinality-critical mode

Supporting experiments:

- ideal-communication comparison between Ordinary GA, the Balanced mode, the FID-FIA baseline, and the Cardinality-critical mode
- communication-robustness analysis across communication levels
- computational-cost measurement for the main baseline and proposed operating modes
- appendix-only AA-based secondary route

Appendix or negative ablations:

- `w/o NIS -> robust NIS -> plain NIS`
- `w/o history -> history`
- `robust NIS baseline -> robust NIS + freshness`
- other weak or currently negative modules

This hierarchy matches the current evidence strength: the strongest claims are supported by the tiered-drop GA scenario, while the remaining experiments are used to contextualize or bound the method rather than define the main contribution.

### 2. Main Scenario: Dual-Formation Eight-Sensor GA Tracking

The primary evaluation setting uses eight mobile sensors arranged as two four-sensor formations. The filtering mode is distributed local fusion, meaning that each node fuses only its own posterior and the posteriors available inside its communication neighborhood. The local multi-object filter is GA-LMB with loopy belief propagation (LBP) for data association.

The common sensor-side configuration is:

- number of sensors: `8`
- filter mode: `GA-LMB`
- data association: `LBP`
- clutter rate: `3` for every sensor
- detection probability: `0.9` for every sensor
- measurement-noise scale: `q = 3` for every sensor
- sensor field of view enabled with half-angle `60 deg` and range `60000`

The sensor motion model is constant velocity with zero process noise for the platform trajectories. The two four-sensor formations move from left to right with formation centers initialized at approximately `[-80, 35]^T` and `[-80, -35]^T`, respectively, and with nominal velocity `[0.8, 0]^T`. The communication neighborhood is fixed by the dual-formation graph used in the current scripts: each formation is fully connected internally, and the two formations are linked by one-to-one cross-formation connections. In the corresponding implementation, the default fusion-weighting rule for the fixed-weight baseline is Metropolis weighting over each local neighborhood.

The target side consists of ten objects arranged in three groups with staggered births. The three groups contain `3 + 3 + 4` targets, respectively, and the targets move toward the central region with nominal speed `0.45`. Births are staggered every eight time steps, and the target-formation lifetime is `100` steps. The simulator also uses a total simulation length of `100` steps, so the main reported metrics are averages over the full time horizon.

### 3. Communication Model

The main scenario uses the tiered heterogeneous packet-loss setting introduced for the present paper branch. Communication is applied before distributed fusion through a measurement-delivery model with bandwidth limitation and link-level packet loss.

The main communication configuration is:

- communication level: `2`
- global maximum delivered measurements per step: `80`
- packet-selection policy: weighted priority with random measurement selection
- link model: fixed packet drop
- target mean drop rate: `pDrop = 0.2`
- tiered drop levels: `[0, 0.1, 0.2, 0.5]`
- tiered level counts: `[1, 4, 1, 2]`
- maximum outage nodes parameter retained as `1`, although the main level-2 experiment does not invoke node outages

This tiered design preserves the historical mean communication degradation while introducing persistent cross-node heterogeneity. At the start of each Monte Carlo trial, a sensor-wise drop-rate vector is sampled by shuffling the tiered levels; the resulting per-sensor drop assignments remain fixed throughout that trial. This design is important because it creates the long-term link-quality variation needed to test whether adaptive communication-aware weighting is useful.

### 4. Mainline Ablation Protocol

The core ablation study compares the following five arms:

1. Fixed Metropolis
2. Covariance-only adaptive
3. Covariance-link adaptive
4. Three-factor adaptive backbone with covariance, link quality, and existence confidence
5. the Balanced mode, which adds weak structure-aware modulation after covariance, link-quality, and existence-confidence weighting

This ablation path is implemented in [runMultisensorFilters_formation_4plus4_TieredLinkAblation.m](/Users/dex/Desktop/Code/MULTISENSOR-LMB-FILTERS/RUN/GA/runMultisensorFilters_formation_4plus4_TieredLinkAblation.m) and is the main source for the factor-by-factor method claims.

The main confirmation additionally compares the FID-FIA baseline and the Cardinality-critical mode. The five-arm path should be read as the backbone diagnostic; manuscript Table 3 also appends the Cardinality-critical mode to show the final branch-specific FID-FIA extension on the same scale. The Balanced mode is the position-sensitive mode for position-sensitive operation, while the Cardinality-critical mode is the cardinality-sensitive mode.

The Balanced mode uses:

- `emaAlpha = 0.7`
- `minWeight = 0.05`
- `useCovariance = true`
- `useLinkQuality = true`
- `useExistenceConfidence = true`
- `existenceConfidenceMinScore = 0.85`
- `existenceConfidencePower = 2.0`
- `useDecoupledKla = true`
- `useStructureAwareKla = true`
- `usePosteriorStructureConsistency = false`
- `spatialDecouplingStrength = 0.5`
- `existenceDecouplingStrength = 0.15`
- `spatialStructureStrength = 0.45`
- `existenceStructureStrength = 0.08`
- `structureReliabilityPower = 0.30`
- `useNIS = false`
- `useHistory = false`

The Cardinality-critical mode uses the same spatial and backbone settings, and additionally applies the following existence-branch settings:

- `useFidFiaExistence = true`
- `fidFiaExistenceStrength = 4.0`
- `fidFiaExistenceMinScore = 0.0`
- `existenceEmaAlpha = 0.0`
- `existenceMinWeight = 0.0`

The FID-FIA settings apply only to the existence branch; the spatial branch keeps the Balanced mode configuration above.

### 5. Monte Carlo Protocol And Reporting Convention

For the current headline experiments, the reported comparisons are based on Monte Carlo trials with deterministic seed control. The main scripts use `baseSeed = 1` and seed each trial as `baseSeed + trial` when fixed seeds are enabled. The current headline main GA comparison uses `20` trials with seeds `2`--`21`; future algorithm arms should reuse these fixed baseline results where possible and rerun only the new arm.

Unless otherwise stated, reported scalar metrics are Monte Carlo means. Local metrics are first computed per sensor over time and then summarized across sensors or trials as needed. Consensus metrics are computed from the distributed state estimates returned by all nodes and then averaged over time and Monte Carlo trials.

The current paper draft should avoid overclaiming statistical certainty from a small trial count. The safer wording is that the present results are consistent across repeated trials and define the current best-supported direction, while larger Monte Carlo studies remain desirable for the final paper version.

### 6. Evaluation Metrics

The experiments report three categories of metrics.

Primary consensus metrics:

- consensus OSPA
- consensus position disagreement
- consensus cardinality disagreement

Secondary local tracking metrics:

- local E-OSPA
- local RMSE
- local cardinality error

Efficiency metrics:

- filter/fusion wall-clock runtime per trial
- runtime per simulation step
- runtime ratio relative to Fixed Metropolis

The consensus metrics are network-level disagreement metrics, not standard truth-referenced tracking benchmarks. Their ingredients are standard: OSPA/GOSPA-style finite-set distances, Hungarian-matched position errors, and cardinality dispersion. The paper-specific step is to aggregate them across post-fusion sensor outputs, because the goal is distributed agreement under heterogeneous communication. For that reason, they should always be reported together with local E-OSPA, local RMSE, and local cardinality error.

The current paper should emphasize consensus metrics as the primary outcome. This is a deliberate design choice: the method is meant to improve the consistency and quality of distributed fused belief across sensors, and the strongest current evidence is indeed on consensus OSPA, consensus position disagreement, and consensus cardinality disagreement. Local metrics are still reported to show that the consensus gains are not obtained by catastrophic local degradation.

Computational cost should be treated as a first-class evaluation axis, not as an optional footnote. The runtime measurement in the main experiment intentionally times only the distributed LMB filtering/fusion call for each arm. Scenario generation, communication-model sampling, plotting, and metric evaluation are excluded. This isolates the algorithmic overhead of the adaptive weighting and branch-specific FID-FIA logic. In interpretation, the Balanced mode should be judged as the low-overhead operating point, while the Cardinality-critical mode should be judged as a higher-cost option that spends additional computation for stronger cardinality consensus.

### 7. Ideal-Communication Supporting Experiment

To test whether the method is merely compensating for packet loss, a supporting ideal-communication experiment compares Ordinary GA, the Balanced mode, the FID-FIA baseline, and the Cardinality-critical mode under the same dual-formation eight-sensor tracking scenario.

The ideal-communication configuration is:

- communication level: `0`
- `pDrop = 0`
- `pDropBySensor = 0` for all sensors

The ordinary-GA and structure-aware comparison is implemented in [runMultisensorFilters_formation_4plus4_IdealCommCompare.m](/Users/dex/Desktop/Code/MULTISENSOR-LMB-FILTERS/RUN/GA/runMultisensorFilters_formation_4plus4_IdealCommCompare.m). The FID-FIA arms reuse the same deterministic seeds and the same ideal-communication setting through the main ablation runner with communication level `0`. This supporting experiment reports both consensus metrics and local metrics. Its role is not to replace the main tiered-drop scenario, but to show how the spatial and existence-branch refinements behave when communication degradation is removed.

### 8. Communication-Robustness And Appendix-Only AA Route

Beyond the main scenario, the paper can include a short communication-robustness supporting study. The AA route should be kept in the appendix rather than in the main text.

Communication-robustness analysis:

- vary the communication level from `0` to `3`
- inspect how consensus OSPA and consensus cardinality disagreement change as communication quality degrades
- use this study to argue that adaptive weighting becomes more useful as communication becomes more heterogeneous or constrained

Appendix-only AA-based secondary route:

- use the secondary AA scenario documented in [FORMATION_4PLUS4_THREEWAVES_AA_RUN.md](/Users/dex/Desktop/Code/MULTISENSOR-LMB-FILTERS/docs/FORMATION_4PLUS4_THREEWAVES_AA_RUN.md)
- retain this only as appendix evidence, because the paper deliberately chooses the GA/KLA route
- do not use this as a main-text result, because the strongest evidence and theory remain on GA-LMB

The AA scenario uses the same eight-sensor communication-constrained setting, but with an arithmetic-average fusion mode and a three-wave target arrangement. The current role of this experiment is appendix evidence, not a main-text evaluation line.

### 9. Secondary And Appendix-Only Ablations

Several experiments are useful for completeness but should not occupy a central place in the main paper body:

- `w/o NIS -> robust NIS -> plain NIS`
- `w/o history -> history`
- `robust NIS baseline -> robust NIS + freshness`
- ambiguity-aware or cardinality-consensus add-ons

These ablations serve two purposes. First, they document that the design space was explored beyond the final chosen method. Second, they justify why the main paper is intentionally centered on the covariance, link-quality, and existence-confidence backbone, followed by branch decoupling and the existence-branch FID-FIA cue. At present, the gains from the secondary modules are weaker, more coupled, or less stable than those from the main-line factors.

### 10. Writing Rule For The Experimental Section

The paper body should present experiments in the following order:

1. main tiered-drop GA result
2. factor ablation
3. ideal-communication supporting evidence
4. communication-robustness analysis
5. short secondary or appendix ablations, including the AA route

This order keeps the narrative tightly aligned with the current evidence hierarchy and prevents weaker modules from diluting the main contribution.

## Experimental-Setup Notes

- Use `an eight-sensor distributed formation scenario composed of two four-sensor formations` in the paper body instead of bare `4+4`.
- Be explicit that the main communication model is tiered heterogeneous packet loss with preserved mean drop rate.
- Be explicit that the current strongest claims are based on consensus metrics.
- Avoid turning the experimental section into a changelog of every attempted factor; keep weak modules in short appendix-style subsections.
