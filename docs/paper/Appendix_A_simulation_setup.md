# Appendix A. Simulation Setup

## Paper-Ready Appendix Draft

This appendix summarizes the simulation settings used by the current paper branch. The purpose is reproducibility rather than narrative emphasis. The main body already explains why the experiments were chosen; the present appendix records the concrete scenario geometry, target initialization, communication model, and supporting ideal-communication variant.

### A.1 Main Scenario Overview

The primary experiment uses an eight-sensor distributed formation-tracking scenario composed of two four-sensor formations. Each node runs a local GA-LMB filter with loopy belief propagation (LBP) for data association, and distributed fusion is performed only over the local communication neighborhood. The main script is [runMultisensorFilters_formation_4plus4_TieredLinkAblation.m](/Users/dex/Desktop/Code/MULTISENSOR-LMB-FILTERS/RUN/GA/runMultisensorFilters_formation_4plus4_TieredLinkAblation.m).

The common filter-side settings are:

- number of sensors: `8`
- local filter mode: `GA-LMB`
- data association: `LBP`
- simulation length: `100` time steps
- sensor communication range: `150`
- fixed-weight baseline fusion rule: `Metropolis`
- clutter rate at every sensor: `3`
- detection probability at every sensor: `0.9`
- measurement-noise scale at every sensor: `q = 3`
- field of view enabled with half-angle `60 deg` and range `60000`

The current headline Monte Carlo studies use deterministic seed control with `baseSeed = 1` and trial seed `baseSeed + trial`. The recent headline tiered-drop and ideal-communication comparisons both use `5` trials.

### A.2 Sensor Geometry And Neighborhood Graph

The eight sensors are divided into two groups of four. Both groups follow constant-velocity platform trajectories with zero process noise, and both move from left to right with nominal velocity `[0.8, 0]^T`.

The two formation centers are:

- Group A center: `[-80, 35]^T`
- Group B center: `[-80, -35]^T`

Each group uses the `Leader3` local formation pattern with spacing `20`. In the implementation, the unscaled `Leader3` offsets are

$$
\begin{bmatrix}
0 & -1 & -1 & -2 \\
0 & -0.7 & 0.7 & 0
\end{bmatrix},
$$

so the four sensor positions in each group are obtained by multiplying these offsets by `20` and adding the corresponding group center.

For Group A, the initial sensor positions are:

- Sensor 1: `[-80, 35]^T`
- Sensor 2: `[-100, 21]^T`
- Sensor 3: `[-100, 49]^T`
- Sensor 4: `[-120, 35]^T`

For Group B, the initial sensor positions are:

- Sensor 5: `[-80, -35]^T`
- Sensor 6: `[-100, -49]^T`
- Sensor 7: `[-100, -21]^T`
- Sensor 8: `[-120, -35]^T`

The local communication graph is fixed. Each formation is fully connected internally, and the two formations are connected by one-to-one cross links:

- `1 <-> 5`
- `2 <-> 6`
- `3 <-> 7`
- `4 <-> 8`

The corresponding neighborhood sets used for distributed fusion are therefore:

- Sensor 1: `{1, 2, 3, 4, 5}`
- Sensor 2: `{1, 2, 3, 4, 6}`
- Sensor 3: `{1, 2, 3, 4, 7}`
- Sensor 4: `{1, 2, 3, 4, 8}`
- Sensor 5: `{1, 5, 6, 7, 8}`
- Sensor 6: `{2, 5, 6, 7, 8}`
- Sensor 7: `{3, 5, 6, 7, 8}`
- Sensor 8: `{4, 5, 6, 7, 8}`

### A.3 Target Initialization

The target side contains ten objects divided into three groups with staggered births. The birth interval is `8` time steps, the birth start time is `1`, and the target lifetime is `100` time steps. The target center used to define motion directions is the origin `[0, 0]^T`.

The three groups are:

- Group 1: `3` targets, `Triangle` formation, center `[70, 80]^T`, spacing `30`, speed `0.45`
- Group 2: `3` targets, `Triangle` formation, center `[80, 0]^T`, spacing `25`, speed `0.45`
- Group 3: `4` targets, `Leader3` formation, center `[70, -80]^T`, spacing `20`, speed `0.45`

The unscaled `Triangle` offsets are

$$
\begin{bmatrix}
0 & -0.5 & 0.5 \\
0 & -0.866 & -0.866
\end{bmatrix},
$$

and the `Leader3` offsets are the same as those used by the sensor formations. For each group, the velocity vector is computed by normalizing the direction from the group center toward `[0, 0]^T` and scaling it by the group speed. This yields three inward-moving target groups approaching the central region from the upper-right, right, and lower-right parts of the plane.

### A.4 Communication Model

Communication degradation is applied through [applyCommunicationModel.m](/Users/dex/Desktop/Code/MULTISENSOR-LMB-FILTERS/common/applyCommunicationModel.m) before distributed fusion. The model combines a global per-step measurement budget with optional link-level packet loss and optional node outages.

The communication levels used by the code are:

- level `0`: no communication restriction
- level `1`: global measurement budget only
- level `2`: level `1` plus link-level packet loss
- level `3`: level `2` plus node outages

The main paper scenario uses level `2` with the following settings:

- communication level: `2`
- global maximum delivered measurements per step: `80`
- sensor packet weights: uniform over sensors
- packet-priority policy: `weightedPriority`
- measurement-selection policy: `random`
- link model: `fixed`
- nominal mean packet-drop rate: `pDrop = 0.2`
- maximum outage nodes parameter retained as `1`

The distinctive part of the main communication model is the tiered heterogeneous packet-loss configuration:

- `pDropLevels = [0, 0.1, 0.2, 0.5]`
- `pDropLevelCounts = [1, 4, 1, 2]`

This tiered configuration preserves the historical mean packet-drop rate because

$$
\frac{1 \times 0 + 4 \times 0.1 + 1 \times 0.2 + 2 \times 0.5}{8} = 0.2.
$$

At the start of each Monte Carlo trial, the code constructs a sensor-wise `pDropBySensor` vector from the above tier counts and then randomly shuffles the assignment across the eight sensors. The resulting per-sensor drop rates remain fixed within that trial. This design creates persistent node-to-node communication heterogeneity while avoiding a permanent bias toward a specific sensor index across trials.

### A.5 Ideal-Communication Supporting Variant

The supporting ideal-communication comparison uses the same sensor geometry, target initialization, and local filtering configuration as the main scenario, but removes communication degradation. The relevant script is [runMultisensorFilters_formation_4plus4_IdealCommCompare.m](/Users/dex/Desktop/Code/MULTISENSOR-LMB-FILTERS/RUN/GA/runMultisensorFilters_formation_4plus4_IdealCommCompare.m).

The ideal-communication settings are:

- communication level: `0`
- global maximum delivered measurements per step: `inf`
- link model: `fixed`
- `pDrop = 0`
- `pDropBySensor = 0` for all eight sensors

This experiment is used only as supporting evidence. Its role is to verify that the final weak structure-aware decoupled refinement is not merely compensating for packet loss.

### A.6 Main Adaptive Configuration Used In The Headline Comparison

The current best adaptive configuration used in the main tiered-drop headline comparison is:

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

This appendix lists these parameters for completeness because they define the final arm in the primary factor ablation.

### A.7 Entry Points And Report Files

The main entry points used by the present paper branch are:

- [runMultisensorFilters_formation_4plus4_TieredLinkAblation.m](/Users/dex/Desktop/Code/MULTISENSOR-LMB-FILTERS/RUN/GA/runMultisensorFilters_formation_4plus4_TieredLinkAblation.m)
- [runMultisensorFilters_formation_4plus4_IdealCommCompare.m](/Users/dex/Desktop/Code/MULTISENSOR-LMB-FILTERS/RUN/GA/runMultisensorFilters_formation_4plus4_IdealCommCompare.m)
- [applyCommunicationModel.m](/Users/dex/Desktop/Code/MULTISENSOR-LMB-FILTERS/common/applyCommunicationModel.m)

The corresponding saved headline reports currently used in the paper workspace are:

- [GA_TIERED_LINK_ABLATION_20260322_001613.md](/Users/dex/Desktop/Code/MULTISENSOR-LMB-FILTERS/RUN/GA/GA_TIERED_LINK_ABLATION_20260322_001613.md)
- [GA_TIERED_LINK_ABLATION_20260326_182435.md](/Users/dex/Desktop/Code/MULTISENSOR-LMB-FILTERS/RUN/GA/GA_TIERED_LINK_ABLATION_20260326_182435.md)
- [GA_IDEAL_COMM_COMPARE_20260326_184508.md](/Users/dex/Desktop/Code/MULTISENSOR-LMB-FILTERS/RUN/GA/GA_IDEAL_COMM_COMPARE_20260326_184508.md)
