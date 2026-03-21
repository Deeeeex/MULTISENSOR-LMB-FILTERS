# Experimental Setup

## Primary Experimental Story

Use the GA 4+4 distributed formation scenario as the main evaluation setting.

Recommended order:

1. Main scenario
2. NIS ablation
3. Parameter study
4. Communication robustness
5. AA generalization

## Main Scenario

Use:

- `8` sensors in `4+4` formation
- distributed local fusion
- `GA-LMB`
- `LBP` data association
- communication level `2`
- fixed link drop `pDrop = 0.2`
- Metropolis weighting

Main reference:

- `docs/FORMATION_4PLUS4_RUN.md`

## Core Ablation

Use:

- `w/o NIS`
- `robust NIS`
- `plain NIS`

Main reference:

- `RUN/GA/GA_NIS_COMPARE_20260309_164119.md`

## Parameter Study

Use:

- confidence grid
- upper penalty scale grid

Main reference:

- `RUN/GA/GA_NIS_GRID_20260309_163105.md`

## Secondary Experiments

Communication robustness:

- use `analyzeCommunicationLevelImpact.m`
- use communication levels `0-3`

AA generalization:

- use `docs/FORMATION_4PLUS4_THREEWAVES_AA_RUN.md`

## Metrics To Report

Local metrics:

- local E-OSPA
- local RMSE

Consensus metrics:

- consensus OSPA
- consensus RMSE
- consensus cardinality disagreement

Optional:

- runtime

## Baselines

Minimum baseline set:

1. fixed fusion weights
2. adaptive weights without NIS
3. adaptive weights with robust NIS
4. adaptive weights with plain NIS

If more time is available, add:

- uniform weighting
- communication-only weighting
- covariance-only weighting

## Reporting Rule

Whenever possible, report Monte Carlo mean and variance or confidence interval.
