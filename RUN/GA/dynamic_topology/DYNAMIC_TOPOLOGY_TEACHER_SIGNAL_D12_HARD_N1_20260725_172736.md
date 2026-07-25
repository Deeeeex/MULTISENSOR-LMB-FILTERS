# Dynamic-topology teacher-signal screen

- Preset: `d12-hard`
- Seeds: `7`
- Status: `stop-lookahead-redundant`
- Mean predictive gain vs static: `1.601%`
- Mean predictive gain vs current teacher: `0.000%`
- Positive gain vs current rate: `0.0%`
- Changed-topology rate: `0.0%`
- Mean predictive action spread: `3.676%`
- Mean valid candidate count: `15.00`

## Snapshot records

| Seed | Time | Current idx | Predictive idx | Gain vs static | Gain vs current | Risk spread | Valid |
|---:|---:|---:|---:|---:|---:|---:|---:|
| 7 | 30 | 4 | 4 | 1.601% | 0.000% | 3.676% | 15 |

## Decision

Look-ahead rarely changes the action value relative to the current-risk teacher; do not train on the predictive labels.

## Claim boundary

This paired snapshot test validates counterfactual label separation and action sensitivity only. Closed-loop tracking improvement requires a separate paired policy experiment.
