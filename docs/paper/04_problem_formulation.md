# Problem Formulation

## Minimum Notation Checklist

Define:

- sensor set and communication graph
- time index
- local measurements
- local LMB posterior after per-sensor update
- distributed fusion objective
- consensus quality target

## System Description

Suggested subsections:

1. Dynamic and measurement model
2. Communication model
3. Local multi-sensor LMB update
4. Distributed KLA fusion objective
5. Decoupled spatial and existence weighting objective

## Communication Model To Describe

Use the current implementation assumptions:

- bandwidth-limited delivery
- link drop model
- optional outage model
- availability mask at each time
- local neighborhood graph and per-node communication heterogeneity

## Metric Separation

State early that the paper evaluates two distinct aspects:

- local tracking quality
- inter-sensor consensus quality

Current consensus metrics in the codebase:

- consensus OSPA
- consensus RMSE
- consensus cardinality disagreement

Current local metrics in the codebase:

- local E-OSPA
- local RMSE

## Decoupled Fusion Framing

State that the adaptive fusion objective can use:

- a shared quality backbone from covariance, link quality, and existence confidence
- a spatial branch for Gaussian-state fusion
- an existence branch for Bernoulli existence fusion

Then explain that the current best implementation only applies a very weak structure-aware correction to these decoupled branches, especially on the existence side.

## Important Framing Choice

The problem formulation should make it natural that improving consensus can be valuable even when local metrics improve only mildly.
