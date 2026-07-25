# Dynamic-topology teacher-signal screen

- Preset: `m24-hard`
- Seeds: `7`
- Status: `current-task-signal`
- Mean current gain vs static: `2.346%`
- Mean current action spread: `15.373%`
- Mean valid candidate count: `28.00`

- Behavior E-OSPA at snapshots: `25.5087`
- Behavior cardinality error at snapshots: `0.9167`

## Snapshot records

| Seed | Time | Current idx | Gain vs static | Risk spread | Valid |
|---:|---:|---:|---:|---:|---:|
| 7 | 75 | 43 | 2.346% | 15.373% | 28 |

## Decision

Current task risk separates feasible actions at this scale; validate realized closed-loop performance next.

## Claim boundary

This paired snapshot test validates counterfactual label separation and action sensitivity only. Closed-loop tracking improvement requires a separate paired policy experiment.
