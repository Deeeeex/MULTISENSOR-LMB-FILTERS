# Dynamic-topology teacher-signal screen

- Preset: `d12-hard`
- Seeds: `7`
- Status: `stop-closed-loop-redundant`
- Mean predictive gain vs static: `1.601%`
- Mean predictive gain vs current teacher: `0.000%`
- Positive gain vs current rate: `0.0%`
- Changed-topology rate: `0.0%`
- Mean predictive action spread: `3.676%`
- Mean valid candidate count: `15.00`

- Behavior E-OSPA at snapshots: `79.4301`
- Behavior cardinality error at snapshots: `7.5833`

- Closed-loop gain vs static: `1.501%`
- Closed-loop gain vs current teacher: `0.000%`
- Positive closed-loop gain vs current rate: `0.0%`
- Closed-loop changed-topology rate: `0.0%`
- Closed-loop action spread: `2.070%`
- Mean closed-loop teacher time: `93.32 s`

- Best fixed candidate: `8`
- Best-fixed evaluation split: `in-sample diagnostic`
- Predictive gain vs best fixed: `-0.320%`

## Snapshot records

| Seed | Time | Current idx | Predictive idx | Gain vs static | Gain vs current | Risk spread | Valid |
|---:|---:|---:|---:|---:|---:|---:|---:|
| 7 | 30 | 4 | 4 | 1.601% | 0.000% | 3.676% | 15 |

## Closed-loop rollout records

| Seed | Time | Current idx | Closed-loop idx | Gain vs static | Gain vs current | Risk spread | Seconds |
|---:|---:|---:|---:|---:|---:|---:|---:|
| 7 | 30 | 4 | 4 | 1.501% | 0.000% | 2.070% | 93.32 |

## Decision

The future-measurement rollout rarely improves on the current-risk action; do not train on these labels.

## Claim boundary

This paired snapshot test validates counterfactual label separation and action sensitivity only. Closed-loop tracking improvement requires a separate paired policy experiment.
