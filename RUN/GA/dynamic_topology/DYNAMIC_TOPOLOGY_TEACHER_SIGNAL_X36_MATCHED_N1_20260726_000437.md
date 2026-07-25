# Dynamic-topology teacher-signal screen

- Preset: `x36-matched`
- Seeds: `7`
- Status: `stop-unhealthy-behavior-state`
- Mean current gain vs static: `1.263%`
- Mean current action spread: `2.574%`
- Mean valid candidate count: `33.00`

- Behavior E-OSPA at snapshots: `90.0227`
- Behavior cardinality error at snapshots: `8.6944`

- Max normalized behavior E-OSPA: `0.6002`
- Max normalized behavior cardinality error: `0.3623`
- Behavior filter-health gate: `0`

## Snapshot records

| Seed | Time | Current idx | Gain vs static | Risk spread | Valid |
|---:|---:|---:|---:|---:|---:|
| 7 | 75 | 28 | 1.263% | 2.574% | 33 |

## Decision

The shared behavior snapshot fails the registered filter-health gate (normalized E-OSPA 0.600, normalized cardinality error 0.362); redesign the scale scene or filter before comparing topology policies.

## Claim boundary

This paired snapshot test validates counterfactual label separation and action sensitivity only. Closed-loop tracking improvement requires a separate paired policy experiment.
