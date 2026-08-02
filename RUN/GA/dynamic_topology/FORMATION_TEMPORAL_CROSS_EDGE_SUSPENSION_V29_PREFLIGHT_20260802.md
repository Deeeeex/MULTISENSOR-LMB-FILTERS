# V29 temporal cross-edge suspension preflight

- Contract: `formation-temporal-suspension-v29-preflight-v1`
- Generation commit: `5af2031c175fabfc354031d52886e90447951da0`
- Tracked worktree dirty: `0`
- Preset / seed / time: `m24-formation-fov / 211 / 72`
- Source cache SHA-256: `60dfbf2615181cde046af15f42bba37c415ea0034cb7ce53685b79042bfaf762`
- Structural actions: `16`
- Safe nonreference actions: `15`
- Tracking continuation executed: `0`
- New state / X36 / X48 opened: `0 / 0 / 0`
- Bank construction: `86.37 s`

## Frozen safety boundary

The action is applied for one step and followed by two reference steps. Label-retention thresholds are unchanged from v28. All selected three-step sensor and formation windows must be strongly connected, and every candidate sends fewer messages than reference. One-step disagreement is recorded but is not a hard gate.

| Action | Suspended formations | Dropped messages | H3 message saving | One-step disagreement change | Retention risk | Min form. d-card | Min label retention | Threshold drops | Safe |
|:--|:--|--:|--:|--:|--:|--:|--:|--:|:--:|
| `reference` | `[]` | 0 | 0.000% | +0.000% | 0.000000 | +0.000000 | 1.000000 | 0 | 1 |
| `suspend-f1` | `1` | 1 | 0.833% | +0.360% | 0.000001 | +0.000000 | 0.999962 | 0 | 1 |
| `suspend-f2` | `2` | 1 | 0.833% | +2.362% | 0.000000 | +0.000000 | 1.000000 | 0 | 1 |
| `suspend-f1-f2` | `[1 2]` | 2 | 1.667% | +2.733% | 0.000001 | +0.000000 | 0.999962 | 0 | 1 |
| `suspend-f3` | `3` | 1 | 0.833% | +1.023% | 0.000015 | +0.000000 | 0.999370 | 0 | 1 |
| `suspend-f1-f3` | `[1 3]` | 2 | 1.667% | +1.383% | 0.000015 | +0.000000 | 0.999370 | 0 | 1 |
| `suspend-f2-f3` | `[2 3]` | 2 | 1.667% | +3.434% | 0.000015 | +0.000000 | 0.999370 | 0 | 1 |
| `suspend-f1-f2-f3` | `[1 2 3]` | 3 | 2.500% | +3.805% | 0.000015 | +0.000000 | 0.999370 | 0 | 1 |
| `suspend-f4` | `4` | 1 | 0.833% | +1.571% | 0.000214 | +0.000000 | 0.993334 | 0 | 1 |
| `suspend-f1-f4` | `[1 4]` | 2 | 1.667% | +1.944% | 0.000214 | +0.000000 | 0.993334 | 0 | 1 |
| `suspend-f2-f4` | `[2 4]` | 2 | 1.667% | +4.013% | 0.000214 | +0.000000 | 0.993334 | 0 | 1 |
| `suspend-f1-f2-f4` | `[1 2 4]` | 3 | 2.500% | +4.397% | 0.000214 | +0.000000 | 0.993334 | 0 | 1 |
| `suspend-f3-f4` | `[3 4]` | 2 | 1.667% | +2.601% | 0.000214 | +0.000000 | 0.993334 | 0 | 1 |
| `suspend-f1-f3-f4` | `[1 3 4]` | 3 | 2.500% | +2.975% | 0.000214 | +0.000000 | 0.993334 | 0 | 1 |
| `suspend-f2-f3-f4` | `[2 3 4]` | 3 | 2.500% | +5.092% | 0.000214 | +0.000000 | 0.993334 | 0 | 1 |
| `suspend-f1-f2-f3-f4` | `[1 2 3 4]` | 4 | 3.333% | +5.476% | 0.000214 | +0.098505 | 0.993334 | 0 | 1 |

## Decision

At least one nonreference subset passes all frozen causal and rolling-connectivity gates. The primary H=3 continuation may execute only the reference and safe indices `[2 3 4 5 6 7 8 9 10 11 12 13 14 15 16]`. No other M24 state or X36/X48 is authorized.

## Evidence boundary

v29 removes selected registered cross-formation payloads for one step and then executes the fixed reference for two steps. Candidate subsets use no truth or future outcomes and must pass reference-relative existence retention, expected-cardinality, physical, row-stochastic, lower-message-count, and selected rolling-B3 constraints. One-step posterior disagreement is diagnostic only because v27/v28 falsified it as a recursive safety certificate. The primary screen reuses only seed-211 t=72; other M24 states, X36, X48, and reserved seeds remain sealed until its gate passes.
