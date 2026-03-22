# Experimental Setup

## Primary Experimental Story

Use the GA 4+4 distributed formation scenario as the main evaluation setting.

Recommended order:

1. Main scenario
2. Factor ablation
3. NIS ablation
4. Communication robustness
5. AA generalization

## Main Scenario

Use:

- `8` sensors in `4+4` formation
- distributed local fusion
- `GA-LMB`
- `LBP` data association
- communication level `2`
- tiered link drop with mean `pDrop = 0.2`
- `pDropLevels = [0, 0.1, 0.2, 0.5]`
- `pDropLevelCounts = [1, 4, 1, 2]`
- Metropolis weighting

Main reference:

- `docs/COMMUNICATION_TIERED_DROP_UPDATE_CN.md`

## Core Ablation

Use:

- `fixed weights`
- `+covariance`
- `+link quality`
- `+existence confidence`
- `+structure-aware decoupled KLA`

Main reference:

- `RUN/GA/GA_TIERED_LINK_ABLATION_20260322_023216.md`

## Secondary NIS Ablation

Use:

- `w/o NIS`
- `robust NIS`
- `plain NIS`

Main reference:

- `RUN/GA/GA_TIERED_LINK_NIS_COMPARE_20260321_193628.md`

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
2. adaptive weights with covariance only
3. adaptive weights with covariance plus link quality
4. adaptive weights with covariance plus link quality plus existence confidence
5. adaptive weights with covariance plus link quality plus existence confidence plus weak structure-aware decoupled KLA

If more time is available, add:

- robust NIS as optional consistency term
- uniform weighting
- communication-only weighting

## Reporting Rule

Whenever possible, report Monte Carlo mean and variance or confidence interval.
