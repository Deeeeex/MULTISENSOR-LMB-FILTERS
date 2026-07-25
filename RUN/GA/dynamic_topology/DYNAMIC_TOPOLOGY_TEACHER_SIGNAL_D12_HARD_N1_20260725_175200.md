# Dynamic-topology teacher-signal screen

- Preset: `d12-hard`
- Seeds: `7`
- Status: `stop-closed-loop-redundant`
- Mean predictive gain vs static: `0.891%`
- Mean predictive gain vs current teacher: `0.000%`
- Positive gain vs current rate: `0.0%`
- Changed-topology rate: `0.0%`
- Mean predictive action spread: `2.545%`
- Mean valid candidate count: `11.00`

- Behavior E-OSPA at snapshots: `69.5386`
- Behavior cardinality error at snapshots: `5.8333`

- Closed-loop gain vs static: `2.709%`
- Closed-loop gain vs current teacher: `0.000%`
- Positive closed-loop gain vs current rate: `0.0%`
- Closed-loop changed-topology rate: `0.0%`
- Closed-loop action spread: `4.270%`
- Mean closed-loop teacher time: `65.37 s`

- Best fixed candidate: `16`
- Best-fixed evaluation split: `in-sample diagnostic`
- Predictive gain vs best fixed: `-0.715%`

## Snapshot records

| Seed | Time | Current idx | Predictive idx | Gain vs static | Gain vs current | Risk spread | Valid |
|---:|---:|---:|---:|---:|---:|---:|---:|
| 7 | 60 | 4 | 4 | 0.891% | 0.000% | 2.545% | 11 |

## Closed-loop rollout records

| Seed | Time | Current idx | Closed-loop idx | Gain vs static | Gain vs current | Risk spread | Seconds |
|---:|---:|---:|---:|---:|---:|---:|---:|
| 7 | 60 | 4 | 4 | 2.709% | 0.000% | 4.270% | 65.37 |

## Decision

The future-measurement rollout rarely improves on the current-risk action; do not train on these labels.

## Claim boundary

This paired snapshot test validates counterfactual label separation and action sensitivity only. Closed-loop tracking improvement requires a separate paired policy experiment.
