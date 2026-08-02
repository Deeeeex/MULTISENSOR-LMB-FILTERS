# V28 conflict-aware selective-mixing preflight

- Contract: `formation-conflict-aware-v28-preflight-v1`
- Generation commit: `624cabe0c6e701c220723b0f4577136ad796b903`
- Tracked worktree dirty: `0`
- Preset / seed / time: `m24-formation-fov / 211 / 72`
- Source cache SHA-256: `60dfbf2615181cde046af15f42bba37c415ea0034cb7ce53685b79042bfaf762`
- Structural actions: `23`
- Safe nonreference actions: `4`
- Tracking continuation executed: `0`
- New state / X36 / X48 opened: `0 / 0 / 0`
- Bank construction: `166.92 s`

## Frozen current-posterior safety gate

- Maximum reference-weighted retention risk: `0.010`
- Minimum formation-mean expected-cardinality change: `-0.050`
- Minimum supported-label retention ratio: `0.80`
- Maximum 0.5-threshold label drops: `0`
- Maximum one-step disagreement increase: `1.0%`

| Action | Formations | Cross edges | Cross trust | Mix gap | Disagreement change | Retention risk | Min form. d-card | Min label retention | Threshold drops | Safe |
|:--|:--|--:|--:|--:|--:|--:|--:|--:|--:|:--:|
| `reference` | `[]` | 4 | 0.2000 | 0.008298 | +0.000% | 0.000000 | +0.000000 | 1.000000 | 0 | 1 |
| `damp-reference-f1` | `1` | 4 | 0.1750 | 0.005217 | +0.111% | 0.000002 | +0.000000 | 0.999921 | 0 | 1 |
| `damp-reference-f2` | `2` | 4 | 0.1750 | 0.005217 | +0.990% | 0.000000 | +0.000000 | 1.000000 | 0 | 1 |
| `damp-reference-f3` | `3` | 4 | 0.1750 | 0.005217 | +0.399% | 0.000006 | +0.000000 | 0.999729 | 0 | 1 |
| `damp-reference-f4` | `4` | 4 | 0.1750 | 0.005217 | +0.395% | 0.000111 | +0.000000 | 0.996589 | 0 | 1 |
| `pair-redistribute-f1-f2` | `[1 2]` | 6 | 0.2000 | 0.006125 | +0.578% | 0.012188 | -0.017736 | 0.325680 | 0 | 0 |
| `pair-add-low-f1-f2` | `[1 2]` | 6 | 0.2500 | 0.008298 | -0.525% | 0.012188 | -0.116878 | 0.325680 | 0 | 0 |
| `pair-add-reference-f1-f2` | `[1 2]` | 6 | 0.3000 | 0.008298 | -0.889% | 0.014694 | -0.141122 | 0.326414 | 0 | 0 |
| `pair-redistribute-f1-f3` | `[1 3]` | 6 | 0.2000 | 0.007451 | -0.733% | 0.025145 | -0.215937 | 0.077344 | 2 | 0 |
| `pair-add-low-f1-f3` | `[1 3]` | 6 | 0.2500 | 0.008298 | -1.245% | 0.025144 | -0.243293 | 0.077344 | 2 | 0 |
| `pair-add-reference-f1-f3` | `[1 3]` | 6 | 0.3000 | 0.008298 | -4.499% | 0.082463 | -0.797572 | 0.073108 | 8 | 0 |
| `pair-redistribute-f1-f4` | `[1 4]` | 6 | 0.2000 | 0.006125 | -1.946% | 0.054958 | -0.338749 | 0.060378 | 5 | 0 |
| `pair-add-low-f1-f4` | `[1 4]` | 6 | 0.2500 | 0.008298 | -2.444% | 0.054847 | -0.466183 | 0.060378 | 5 | 0 |
| `pair-add-reference-f1-f4` | `[1 4]` | 6 | 0.3000 | 0.008298 | -3.004% | 0.066438 | -0.564703 | 0.060354 | 6 | 0 |
| `pair-redistribute-f2-f3` | `[2 3]` | 6 | 0.2000 | 0.006125 | -1.273% | 0.050323 | -0.174628 | 0.028368 | 4 | 0 |
| `pair-add-low-f2-f3` | `[2 3]` | 6 | 0.2500 | 0.008298 | -2.655% | 0.050323 | -0.483322 | 0.028368 | 4 | 0 |
| `pair-add-reference-f2-f3` | `[2 3]` | 6 | 0.3000 | 0.008298 | -5.378% | 0.087333 | -0.844358 | 0.028370 | 11 | 0 |
| `pair-redistribute-f2-f4` | `[2 4]` | 6 | 0.2000 | 0.007451 | -1.024% | 0.044204 | -0.239391 | 0.031542 | 4 | 0 |
| `pair-add-low-f2-f4` | `[2 4]` | 6 | 0.2500 | 0.008298 | -2.384% | 0.044093 | -0.405590 | 0.031542 | 4 | 0 |
| `pair-add-reference-f2-f4` | `[2 4]` | 6 | 0.3000 | 0.008298 | -3.358% | 0.059479 | -0.569876 | 0.031544 | 6 | 0 |
| `pair-redistribute-f3-f4` | `[3 4]` | 6 | 0.2000 | 0.006125 | +0.236% | 0.011933 | +0.000000 | 0.077476 | 1 | 0 |
| `pair-add-low-f3-f4` | `[3 4]` | 6 | 0.2500 | 0.008298 | -0.549% | 0.011822 | -0.101361 | 0.077476 | 1 | 0 |
| `pair-add-reference-f3-f4` | `[3 4]` | 6 | 0.3000 | 0.008298 | -1.333% | 0.029792 | -0.295824 | 0.061062 | 3 | 0 |

## Decision

At least one nonreference action passes every frozen current-posterior guard. The primary H=3 tracking screen may execute only the reference and these pre-approved action indices: `[2 3 4 5]`. This authorization does not open another M24 state or X36/X48.

## Evidence boundary

v28 uses current-posterior KLA compatibility only to construct pairwise candidates, then requires reference-relative existence retention, expected-cardinality, disagreement, physical, message, and connectivity guards before a tracking continuation may run. Thresholds are frozen before the v28 outcome screen. The first screen reuses only the already-opened seed-211 t=72 M24 state. Other M24 states, X36, X48, and reserved seeds remain sealed until the preregistered primary gate passes.
