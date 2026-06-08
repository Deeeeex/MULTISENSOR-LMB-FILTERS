# Paper Workspace

## Current Positioning

Working title:

`Communication-Aware Adaptive Weights for Consensus-Oriented Distributed KLA-Based LMB Fusion`

The retained method is the no-stabilization Balanced configuration:

- covariance, realized link quality, and existence confidence;
- branch-decoupled spatial/existence weights;
- weak structure-aware correction;
- no EMA smoothing;
- no final-weight floor;
- Cardinality-critical mode adds FID-FIA only to the existence branch.

## Strongest Supported Claims

- Realized link quality is the dominant factor under tiered heterogeneous packet loss.
- Covariance provides an independent signal and an additional cardinality benefit.
- Branch decoupling and weak structure awareness are evaluated jointly as a branch-aware refinement with a small but repeatable spatial-consensus improvement.
- Existence confidence has a small isolated incremental effect in the current scenario.
- EMA/floor is not retained: it improves only local RMSE while degrading the other disagreement and local finite-set metrics.

## Headline Numbers

Balanced mode over 50 deterministic paired trials:

- network disagreement: `1.696 / 1.461 / 0.095`;
- local safeguards: `2.072 / 1.636 / 0.283`;
- runtime: `4.793 s`, approximately `1.099x` Fixed Metropolis.

Cardinality-critical mode over 50 deterministic trials:

- network disagreement: `1.713 / 1.590 / 0.062`;
- local safeguards: `2.030 / 1.744 / 0.209`;
- absolute Octave runtime: `144.057 s` per 100-step trial.

## Evidence Boundary

Old ideal-communication, communication-level, and AA results used the retired stabilized configuration. They must be rerun before returning to the main manuscript evidence chain.

## Primary Files

- `els-cas-templates/manuscript.tex`
- `els-cas-templates/sections/04_method.tex`
- `els-cas-templates/sections/05_experimental_setup.tex`
- `els-cas-templates/sections/06_results.tex`
- `els-cas-templates/sections/07_conclusion.tex`
- `05_method_adaptive_kla.md`
- `07_results_and_ablation.md`
- `../PAPER_MAIN_RESULTS_CN.md`
