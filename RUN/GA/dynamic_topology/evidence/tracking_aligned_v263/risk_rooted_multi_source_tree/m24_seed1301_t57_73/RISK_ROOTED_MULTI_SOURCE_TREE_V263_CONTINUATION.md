# V263 risk-rooted multi-source tree

- Scene / seed: `m24-formation-fov-temporal-coupled-formation-braid / 1301`
- Generation commit: `60841d88d9f3d8a0050f7c06da2cc38b17433196`
- Continuation: `t=57--73` from the V259 V242 state
- Rooted tree initiated / active pages: `57 / [57 58 59 60 61 62 63 64 65 66]`
- Maximum eligible / initially shortened sources: `2 / 1`
- Active pages componentwise shortest: `1`
- Mechanism / full-M24 authorization: `0 / 0`

| Arm | E-OSPA | RMSE | Consistency | F4 event E / RMSE | Window bytes | Spliced static saving |
|:--|--:|--:|--:|:--|--:|--:|
| V242 minimum backbone | 118.651 | 16.844 | 130.254 | 123.683 / 26.636 | 2991480 | 10.041% |
| V263 risk-rooted multi-source tree | 115.919 | 18.145 | 124.802 | 124.467 / 19.647 | 2887680 | 10.295% |

## Gains over V242

| Scope | E-OSPA | RMSE |
|:--|--:|--:|
| Network | `+2.302%` | `-7.723%` |
| F4 event | `-0.634%` | `+26.236%` |
| Weakest formation | `-0.596%` | `-43.135%` |

- Consistency gain: `+4.185%`
- Window byte change vs V242: `+3.470%`
- Short-horizon gate: `0`
- Next method decision: `test-one-edge-risk-cycle-before-label-selective-edge-trust`

## Decision

The componentwise shortest risk-rooted tree does not repair the diagnosed F4 event under the registered guards. Close tree-only routing and test one minimally redundant risk-cycle edge before changing label-wise trust or training a learned ranker.

## Evidence boundary

V263 is an outcome-opened M24 continuation mechanism screen. The current controller uses only current LMB posterior summaries, the current physical graph, current link reliability and past selected routes. When one formation has a high localization tail and at least two other formations hold supported lower-risk copies of its highest-risk label, V263 roots a breadth-first formation tree at that formation. This minimizes every formation-to-target hop distance componentwise over current physical spanning trees, so both localization and set-support sources receive shortest-hop access without selecting a tuned mixture coefficient. The route keeps exactly N+2(F-1) posterior messages and the V242 local-cycle and weight contract. The controller is centralized and its synopsis cost is not included, so this result can establish only action-mechanism headroom, not deployment cost, validation or generalization.
