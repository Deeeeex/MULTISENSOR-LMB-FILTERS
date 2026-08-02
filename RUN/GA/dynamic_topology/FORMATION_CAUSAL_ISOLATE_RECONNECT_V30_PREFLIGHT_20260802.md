# V30 causal retention-debt preflight

- Contract: `formation-causal-isolate-reconnect-v30-preflight-v1`
- Generation commit: `f438759d7136fdb2e59ab2a2ab41c080cd151a64`
- Tracked worktree dirty: `0`
- Preset / seed / time: `m24-formation-fov / 211 / 72`
- Source cache SHA-256: `60dfbf2615181cde046af15f42bba37c415ea0034cb7ce53685b79042bfaf762`
- Safe initial nonreference actions: `15`
- Controller evaluations at anchor: `6`
- Anchor requested / selected formations: `[2 3 4] / [2 3 4]`
- Selected initial action: `15 / suspend-f2-f3-f4`
- Tracking continuation executed: `0`
- New state / X36 / X48 opened: `0 / 0 / 0`
- Initial bank / controller construction: `86.38 / 33.74 s`

## Anchor-state source-only diagnostic

| Formation | Previously suspended | Reference E-card | Suspend d-card | Retention debt | Active threshold | Single safe | Requested |
|--:|:--:|--:|--:|--:|--:|:--:|:--:|
| 1 | 0 | 14.337635 | +0.098505 | +0.687% | 2.000% | 1 | 0 |
| 2 | 0 | 13.447422 | +0.766576 | +5.701% | 2.000% | 1 | 1 |
| 3 | 0 | 14.372194 | +0.336880 | +2.344% | 2.000% | 1 | 1 |
| 4 | 0 | 12.271022 | +0.570676 | +4.651% | 2.000% | 1 | 1 |

## Decision

The v29 initial bank is reproduced exactly and at least one nonreference suspension action remains safe. The frozen 2%/1% hysteretic retention-debt rule selects formations `[2 3 4]` without truth or future outcomes and maps uniquely to `suspend-f2-f3-f4`. The controller will be recomputed from the live posterior at all three selected steps, with exact joint label-retention and rolling-B3 projection. The primary M24/t72 tracking screen may run.

## Evidence boundary

v30 reuses only the already-opened seed-211 t=72 M24 state. At every selected step, the controller estimates each formation's counterfactual expected-cardinality recovery after withholding its registered cross input. A two-threshold hysteresis rule requests suspensions, and the joint action is projected through the frozen reference-relative existence and rolling-B3 constraints. The controller reads only the current posterior, physical graph, link reliability, and selected-topology history; it reads no truth or future outcome. Offline truth scores the H=3 development outcome only. Other M24 states, GNN training, X36, X48, and reserved validation remain sealed until the primary strong gate passes.
