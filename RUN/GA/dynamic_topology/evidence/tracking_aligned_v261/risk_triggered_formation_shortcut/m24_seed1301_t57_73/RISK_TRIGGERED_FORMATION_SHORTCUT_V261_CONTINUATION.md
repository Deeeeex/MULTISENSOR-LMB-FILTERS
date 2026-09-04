# V261 risk-triggered formation shortcut

- Scene / seed: `m24-formation-fov-temporal-coupled-formation-braid / 1301`
- Generation commit: `ed1c44a56d5c6221bcef427809b86bc2cd1da32f`
- Continuation: `t=57--73` from the V259 V242 state
- Shortcut initiated / active pages: `[57 58 59 60 61 62 63 64 65 66 67] / [57 58 59 60 61 62 63 64 65 66 67]`
- Mechanism / full-M24 authorization: `0 / 0`

| Arm | E-OSPA | RMSE | Consistency | F4 event E / RMSE | Window bytes | Spliced static saving |
|:--|--:|--:|--:|:--|--:|--:|
| V242 minimum backbone | 118.651 | 16.844 | 130.254 | 123.683 / 26.636 | 2991480 | 10.041% |
| V261 risk-triggered formation shortcut | 118.229 | 15.920 | 129.599 | 124.047 / 22.342 | 2962056 | 10.113% |

## Gains over V242

| Scope | E-OSPA | RMSE |
|:--|--:|--:|
| Network | `+0.356%` | `+5.488%` |
| F4 event | `-0.294%` | `+16.119%` |
| Weakest formation | `-0.276%` | `-3.619%` |

- Consistency gain: `+0.503%`
- Window byte change vs V242: `+0.984%`
- Short-horizon gate: `0`
- Next method decision: `close-formation-hop-shortcut-and-revisit-relay-gateway-alignment`

## Decision

The same-budget formation shortcut does not repair the diagnosed F4 event. Close pure hop shortening and test whether temporal gateway alignment, rather than the formation tree alone, is the missing relay mechanism.

## Evidence boundary

V261 is an outcome-opened M24 continuation mechanism screen. The V259 trace revealed a formation-level common localization bias rather than an intra-formation disagreement failure. The online policy uses only current LMB posterior summaries, the current physical graph and past selected routes. When one formation has a high localization tail and another has a well-supported lower-risk copy of the same label, it replaces the current formation tree only if a current physical path can reduce their KLA hop distance. The projected route keeps exactly N+2(F-1) posterior messages and the V242 local-cycle and weight contract. The controller is centralized and its synopsis cost is not included, so the result can establish action-mechanism headroom only, not deployment cost, validation or generalization.
