# V262 risk-episode latched formation shortcut

- Scene / seed: `m24-formation-fov-temporal-coupled-formation-braid / 1301`
- Generation commit: `ce92a479c2c45acb36f8ef75f5d3af4de8f17fb0`
- Continuation: `t=57--73` from the V259 V242 state
- Shortcut initiated / active pages: `57 / [57 58 59 60 61 62 63 64 65 66]`
- Latch retained / released pages: `[58 59 60 61 62 63 64 65 66] / 67`
- Donor switches suppressed at: `[58 59 60 61 62 63 64 65 66]`
- Mechanism / full-M24 authorization: `0 / 0`

- Runtime contract flags: `[true true true true true true true true true true true]` (passed `1`)

| Arm | E-OSPA | RMSE | Consistency | F4 event E / RMSE | Window bytes | Spliced static saving |
|:--|--:|--:|--:|:--|--:|--:|
| V242 minimum backbone | 118.651 | 16.844 | 130.254 | 123.683 / 26.636 | 2991480 | 10.041% |
| V262 risk-episode latched formation shortcut | 118.022 | 15.517 | 129.813 | 124.384 / 20.160 | 2932848 | 10.185% |

## Gains over V242

| Scope | E-OSPA | RMSE |
|:--|--:|--:|
| Network | `+0.530%` | `+7.878%` |
| F4 event | `-0.567%` | `+24.311%` |
| Weakest formation | `-0.533%` | `-6.464%` |

- Consistency gain: `+0.338%`
- Window byte change vs V242: `+1.960%`
- Short-horizon gate: `0`
- Next method decision: `close-single-donor-latch-and-formulate-multi-source-risk-tree`

## Decision

Suppressing single-donor chatter does not close all registered guards. Close this latch mechanism and formulate a multi-source risk tree before considering a learned ranker.

## Evidence boundary

V262 is an outcome-opened M24 continuation mechanism screen. The V259 trace revealed a formation-level common localization bias rather than an intra-formation disagreement failure. The online policy uses only current LMB posterior summaries, the current physical graph and its immediately preceding policy schedule. When one formation has a high localization tail and another has a well-supported lower-risk copy of the same label, it replaces the current formation tree only if a current physical path can reduce their KLA hop distance. The accepted donor, target, label and tree are then retained while that same risk episode remains supported, suppressing the donor chattering observed in V261. The projected route keeps exactly N+2(F-1) posterior messages and the V242 local-cycle and weight contract. The controller is centralized and its synopsis cost is not included, so the result can establish action-mechanism headroom only, not deployment cost, validation or generalization.
