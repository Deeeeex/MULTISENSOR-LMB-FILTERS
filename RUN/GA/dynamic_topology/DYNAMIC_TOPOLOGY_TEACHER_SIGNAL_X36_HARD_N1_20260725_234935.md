# Dynamic-topology teacher-signal screen

- Preset: `x36-hard`
- Seeds: `7`
- Status: `stop-unhealthy-behavior-state`
- Mean current gain vs static: `0.736%`
- Mean current action spread: `1.551%`
- Mean valid candidate count: `33.00`

- Behavior E-OSPA at snapshots: `107.3636`
- Behavior cardinality error at snapshots: `12.3056`

- Max normalized behavior E-OSPA: `0.7158`
- Max normalized behavior cardinality error: `0.5127`
- Behavior filter-health gate: `0`

## Snapshot records

| Seed | Time | Current idx | Gain vs static | Risk spread | Valid |
|---:|---:|---:|---:|---:|---:|
| 7 | 75 | 9 | 0.736% | 1.551% | 33 |

## Decision

The shared behavior snapshot fails the registered filter-health gate (normalized E-OSPA 0.716, normalized cardinality error 0.513); redesign the scale scene or filter before comparing topology policies.

## Claim boundary

This paired snapshot test validates counterfactual label separation and action sensitivity only. Closed-loop tracking improvement requires a separate paired policy experiment.
