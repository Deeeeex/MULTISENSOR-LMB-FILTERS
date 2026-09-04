# V265 label-selective risk shortcut

- Scene / seed: `m24-formation-fov-temporal-coupled-formation-braid / 1301`
- Generation commit: `596acd89fcd6c019c533c51ad21028769d0e0081`
- Continuation: `t=57--73` from the V259 V242 state
- Label-route active / delivered / applied pages: `[57 58 59 60 61 62 63 64 65 66 67 68 69] / [57 59 60 62 63 64 65 66 67] / [57 59]`
- Label payload attempted / delivered bytes: `10976 / 8064`
- Mechanism / full-M24 authorization: `0 / 0`

| Arm | E-OSPA | RMSE | Consistency | F4 event E / RMSE | Window bytes | Spliced static saving |
|:--|--:|--:|--:|:--|--:|--:|
| V242 minimum backbone | 118.651 | 16.844 | 130.254 | 123.683 / 26.636 | 2991480 | 10.041% |
| V265 label-selective risk shortcut | 118.682 | 16.059 | 130.197 | 123.764 / 23.345 | 3028280 | 9.951% |

## Gains over V242

| Scope | E-OSPA | RMSE |
|:--|--:|--:|
| Network | `-0.026%` | `+4.660%` |
| F4 event | `-0.065%` | `+12.353%` |
| Weakest formation | `-0.061%` | `-0.108%` |

- Consistency gain: `+0.044%`
- Window byte change vs V242: `-1.230%`
- Short-horizon gate: `0`
- Next method decision: `inspect-label-route-action-value-before-any-learned-ranker`

## Decision

The causal label-only shortcut does not repair the diagnosed event under the registered guards. Inspect whether the failure is source selection, one-hop latency or insufficient label weight before introducing a learned ranker.

## Evidence boundary

V265 is an outcome-opened M24 continuation mechanism screen. It keeps the complete V242 full-posterior backbone and its fusion weights unchanged. The causal V261 current-posterior risk rule selects one formation, one label and a lower-risk donor formation. Only that complete Bernoulli Gaussian-mixture label is sent across the first physical hop of the shorter path and added with zero global topology mass to one ordinary label-wise KLA call. A fixed V242 residual share is assigned only to that label, and the existing receiver-side eta projection falls back to ordinary fusion if the fused existence log odds leave the registered envelope. The complete label payload is charged even when the projection rejects it; centralized risk-synopsis cost is not yet included. The controller uses no target truth, future measurement or future tracking outcome, so this result can establish mechanism headroom only, not deployment cost, validation or generalization.
