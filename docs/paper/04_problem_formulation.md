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

## Communication Model To Describe

Use the current implementation assumptions:

- bandwidth-limited delivery
- link drop model
- optional outage model
- availability mask at each time

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

## Important Framing Choice

The problem formulation should make it natural that improving consensus can be valuable even when local metrics improve only mildly.
