# Dynamic-topology teacher-signal screen

- Preset: `d12-hard`
- Seeds: `7`
- Status: `stop-lookahead-redundant`
- Mean predictive gain vs static: `1.117%`
- Mean predictive gain vs current teacher: `0.000%`
- Positive gain vs current rate: `0.0%`
- Changed-topology rate: `0.0%`
- Mean predictive action spread: `2.965%`
- Mean valid candidate count: `13.67`

- Behavior E-OSPA at snapshots: `69.2546`
- Behavior cardinality error at snapshots: `5.8056`

## Snapshot records

| Seed | Time | Current idx | Predictive idx | Gain vs static | Gain vs current | Risk spread | Valid |
|---:|---:|---:|---:|---:|---:|---:|---:|
| 7 | 40 | 4 | 4 | 1.103% | 0.000% | 2.999% | 15 |
| 7 | 60 | 4 | 4 | 0.891% | 0.000% | 2.545% | 11 |
| 7 | 80 | 4 | 4 | 1.358% | 0.000% | 3.349% | 15 |

## Decision

Look-ahead rarely changes the action value relative to the current-risk teacher; do not train on the predictive labels.

## Claim boundary

This paired snapshot test validates counterfactual label separation and action sensitivity only. Closed-loop tracking improvement requires a separate paired policy experiment.
