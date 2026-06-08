# Experimental Setup

## Paper-Ready Experimental-Setup Draft

The primary evidence is the dual-formation eight-sensor distributed GA-LMB tracking problem under tiered heterogeneous packet loss. A controlled paired experiment isolates each retained component and the rejected stabilization option under one consistent configuration.

### 1. Main Scenario

The evaluation uses eight mobile sensors arranged as two four-sensor formations. Each node performs local GA-LMB filtering with LBP data association and fuses only the posteriors available in its communication neighborhood.

Common settings:

- number of sensors: `8`
- simulation length: `100` steps
- filter mode: `GA-LMB`
- data association: `LBP`
- clutter rate: `3`
- detection probability: `0.9`
- Cartesian measurement-noise scale: `q = 3`
- field of view: half-angle `60 deg`, range `60000`
- targets: ten objects in staggered `3 + 3 + 4` birth waves

### 2. Communication Model

The main scenario applies communication degradation before distributed fusion:

- communication level: `2`
- maximum delivered measurements per step: `80`
- packet selection: weighted priority with random measurement selection
- link model: fixed packet drop
- tiered drop levels: `[0, 0.1, 0.2, 0.5]`
- tier counts: `[1, 4, 1, 2]`

At the start of each trial, the tiered drop-rate vector is shuffled across sensors and then kept fixed throughout that trial. This creates persistent cross-node delivery heterogeneity while avoiding permanent sensor-index bias.

### 3. Mainline Ablation Protocol

The 50-trial paired study compares:

1. Fixed Metropolis
2. covariance only
3. link quality only
4. existence confidence only
5. covariance + link
6. covariance + link + existence confidence with shared weights
7. branch-aware spatial/existence refinement with weak structure-aware modulation, retained as Balanced mode
8. Cardinality-critical mode, which adds FID-FIA only to the existence branch

The retained Balanced configuration uses:

- `useCovariance = true`
- `useLinkQuality = true`
- `useExistenceConfidence = true`
- `existenceConfidenceMinScore = 0.85`
- `existenceConfidencePower = 2.0`
- `useDecoupledKla = true`
- `useStructureAwareKla = true`
- `spatialDecouplingStrength = 0.5`
- `existenceDecouplingStrength = 0.15`
- `spatialStructureStrength = 0.45`
- `existenceStructureStrength = 0.08`
- `emaAlpha = 0.0`
- `minWeight = 0.0`
- `spatialEmaAlpha = 0.0`
- `existenceEmaAlpha = 0.0`
- `spatialMinWeight = 0.0`
- `existenceMinWeight = 0.0`

The FID-FIA extension preserves all six zero stabilization fields and changes
only the existence branch.

### 4. Monte Carlo Protocol

All reported main results use 50 deterministic paired trials with seeds `2`--`51`. Within each trial, every arm shares the same truth, measurements, and communication realization. Local metrics are averaged over sensors before the across-trial mean and standard deviation are computed.

### 5. Metrics

Primary network-disagreement metrics:

- OSPA consensus error
- matched localization disagreement
- cardinality dispersion

Truth-referenced local safeguards:

- local E-OSPA
- local RMSE
- local cardinality error

Efficiency metrics:

- filtering/fusion wall-clock runtime per trial
- runtime per simulation step
- runtime relative to Fixed Metropolis

Scenario generation, communication sampling, plotting, and metric evaluation are excluded from the runtime interval.

### 6. Evidence Boundary

The current no-stabilization Balanced and Cardinality-critical configurations are confirmed only in the main tiered heterogeneous packet-loss experiment. Earlier ideal-communication, communication-level, and arithmetic-average results used the retired stabilization settings. They remain historical diagnostics and must be rerun with the current configuration before returning to the manuscript evidence chain.
