# Experimental Setup

## Paper-Ready Experimental-Setup Draft

This section defines the evaluation protocol used to support the current paper narrative. The experiments are organized around one main scenario and a small set of supporting studies. The main scenario is the dual-formation eight-sensor distributed GA-LMB tracking problem under tiered heterogeneous packet loss. Supporting studies are used to check whether the observed gains persist under ideal communication, whether the method generalizes beyond the main GA setting, and whether several optional modules are worth keeping in the main story.

### 1. Evaluation Hierarchy

The current evaluation is intentionally hierarchical.

Main experiment:

- dual-formation eight-sensor distributed GA-LMB tracking under tiered heterogeneous packet loss
- factor ablation from fixed weights to the full adaptive method

Supporting experiments:

- ideal-communication comparison between ordinary GA, the branch-decoupled backbone, the scalar FID-FIA baseline, and the proposed branch-decoupled fusion
- communication-robustness analysis across communication levels
- AA-based secondary generalization experiment

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

1. fixed fusion weights
2. adaptive weighting with covariance only
3. joint covariance-and-link weighting
4. the three-factor backbone with covariance, link quality, and existence confidence
5. the branch-decoupled backbone, which adds weak structure-aware modulation after covariance, link-quality, and existence-confidence weighting

This ablation path is implemented in [runMultisensorFilters_formation_4plus4_TieredLinkAblation.m](/Users/dex/Desktop/Code/MULTISENSOR-LMB-FILTERS/RUN/GA/runMultisensorFilters_formation_4plus4_TieredLinkAblation.m) and is the main source for the factor-by-factor method claims.

The main confirmation additionally compares the Cao-Zhao scalar FID-FIA baseline and the proposed branch-decoupled fusion. The five-arm path should be read as the backbone diagnostic; manuscript Table 3 also appends the proposed method row to show the final branch-specific FID-FIA extension on the same scale.

The current best adaptive configuration uses:

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
- `useFidFiaExistence = true`
- `fidFiaExistenceStrength = 4.0`
- `fidFiaExistenceMinScore = 0.0`
- `existenceEmaAlpha = 0.0`
- `existenceMinWeight = 0.0`
- `useNIS = false`
- `useHistory = false`

This configuration corresponds to the current main-line method definition used throughout the paper workspace. The FID-FIA settings apply only to the existence branch; the spatial branch keeps the structure-aware KLA configuration above.

### 5. Monte Carlo Protocol And Reporting Convention

For the current headline experiments, the reported comparisons are based on Monte Carlo trials with deterministic seed control. The main scripts use `baseSeed = 1` and seed each trial as `baseSeed + trial` when fixed seeds are enabled. The current headline main GA comparison uses `20` trials with seeds `2`--`21`; future algorithm arms should reuse these fixed baseline results where possible and rerun only the new arm.

Unless otherwise stated, reported scalar metrics are Monte Carlo means. Local metrics are first computed per sensor over time and then summarized across sensors or trials as needed. Consensus metrics are computed from the distributed state estimates returned by all nodes and then averaged over time and Monte Carlo trials.

The current paper draft should avoid overclaiming statistical certainty from a small trial count. The safer wording is that the present results are consistent across repeated trials and define the current best-supported direction, while larger Monte Carlo studies remain desirable for the final paper version.

### 6. Evaluation Metrics

The experiments report two categories of metrics.

Local tracking metrics:

- local E-OSPA
- local RMSE

Consensus metrics:

- consensus OSPA
- consensus RMSE
- consensus cardinality disagreement

The current paper should emphasize consensus metrics as the primary outcome. This is a deliberate design choice: the method is meant to improve the consistency and quality of distributed fused belief across sensors, and the strongest current evidence is indeed on consensus OSPA, consensus RMSE, and consensus cardinality disagreement. Local metrics are still reported to show that the consensus gains are not obtained by catastrophic local degradation.

Runtime can be reported as an optional supplementary metric, but it is not part of the current headline claim set.

### 7. Ideal-Communication Supporting Experiment

To test whether the method is merely compensating for packet loss, a supporting ideal-communication experiment compares ordinary GA, the branch-decoupled backbone, the Cao-Zhao scalar FID-FIA baseline, and the proposed branch-decoupled fusion under the same dual-formation eight-sensor tracking scenario.

The ideal-communication configuration is:

- communication level: `0`
- `pDrop = 0`
- `pDropBySensor = 0` for all sensors

The ordinary-GA and structure-aware comparison is implemented in [runMultisensorFilters_formation_4plus4_IdealCommCompare.m](/Users/dex/Desktop/Code/MULTISENSOR-LMB-FILTERS/RUN/GA/runMultisensorFilters_formation_4plus4_IdealCommCompare.m). The FID-FIA arms reuse the same deterministic seeds and the same ideal-communication setting through the main ablation runner with communication level `0`. This supporting experiment reports both consensus metrics and local metrics. Its role is not to replace the main tiered-drop scenario, but to show how the spatial and existence-branch refinements behave when communication degradation is removed.

### 8. Communication-Robustness And Secondary Generalization

Beyond the main scenario, the paper can include two short supporting studies.

Communication-robustness analysis:

- vary the communication level from `0` to `3`
- inspect how consensus OSPA and consensus cardinality change as communication quality degrades
- use this study to argue that adaptive weighting becomes more useful as communication becomes more heterogeneous or constrained

AA-based generalization:

- use the secondary AA scenario documented in [FORMATION_4PLUS4_THREEWAVES_AA_RUN.md](/Users/dex/Desktop/Code/MULTISENSOR-LMB-FILTERS/docs/FORMATION_4PLUS4_THREEWAVES_AA_RUN.md)
- treat this as evidence that the general weighting idea is not strictly tied to GA
- keep the section short, because the strongest evidence remains on GA-LMB

The AA scenario uses the same eight-sensor communication-constrained setting, but with an arithmetic-average fusion mode and a three-wave target arrangement. The current role of this experiment is extension evidence, not a co-equal main evaluation line.

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
5. AA generalization
6. short secondary or appendix ablations

This order keeps the narrative tightly aligned with the current evidence hierarchy and prevents weaker modules from diluting the main contribution.

## Experimental-Setup Notes

- Use `an eight-sensor distributed formation scenario composed of two four-sensor formations` in the paper body instead of bare `4+4`.
- Be explicit that the main communication model is tiered heterogeneous packet loss with preserved mean drop rate.
- Be explicit that the current strongest claims are based on consensus metrics.
- Avoid turning the experimental section into a changelog of every attempted factor; keep weak modules in short appendix-style subsections.
