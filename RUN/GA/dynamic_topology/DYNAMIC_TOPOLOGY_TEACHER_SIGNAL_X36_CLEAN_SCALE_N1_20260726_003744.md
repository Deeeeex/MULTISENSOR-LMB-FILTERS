# Dynamic-topology teacher-signal screen

- Preset: `x36-clean-scale`
- Seeds: `7`
- Status: `current-task-signal`
- Mean current gain vs static: `1.595%`
- Mean current action spread: `11.237%`
- Mean valid candidate count: `33.00`

- Behavior E-OSPA at snapshots: `46.8048`
- Behavior cardinality error at snapshots: `2.4444`

- Max normalized behavior E-OSPA: `0.3120`
- Max normalized behavior cardinality error: `0.1019`
- Behavior filter-health gate: `1`

## Snapshot records

| Seed | Time | Current idx | Gain vs static | Risk spread | Valid |
|---:|---:|---:|---:|---:|---:|
| 7 | 75 | 22 | 1.595% | 11.237% | 33 |

## Decision

Current task risk separates feasible actions at this scale; validate realized closed-loop performance next.

## Claim boundary

This paired snapshot test validates counterfactual label separation and action sensitivity only. Closed-loop tracking improvement requires a separate paired policy experiment.
