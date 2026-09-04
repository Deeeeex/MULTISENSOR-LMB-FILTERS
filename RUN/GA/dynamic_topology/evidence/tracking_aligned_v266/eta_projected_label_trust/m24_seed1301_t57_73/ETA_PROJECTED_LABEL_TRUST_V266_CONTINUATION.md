# V266 eta-projected label trust

- Scene / seed: `m24-formation-fov-temporal-coupled-formation-braid / 1301`
- Generation commit: `e18de81a684596699e0961fdc53df2939713c463`
- Continuation: `t=57--73` from the V259 V242 state
- Label-route active / delivered / applied pages: `[57 58 59 60 61 62 63 64 65] / [57 59 60 62 63 64 65] / [57 59 60 62 63 64 65]`
- Label payload attempted / delivered bytes: `8064 / 6776`
- Mechanism / full-M24 authorization: `0 / 0`

| Arm | E-OSPA | RMSE | Consistency | F4 event E / RMSE | Window bytes | Spliced static saving |
|:--|--:|--:|--:|:--|--:|--:|
| V242 minimum backbone | 118.651 | 16.844 | 130.254 | 123.683 / 26.636 | 2991480 | 10.041% |
| V266 eta-projected label trust | 118.880 | 14.642 | 130.682 | 124.612 / 17.374 | 3029400 | 9.948% |

## Gains over V242

| Scope | E-OSPA | RMSE |
|:--|--:|--:|
| Network | `-0.193%` | `+13.076%` |
| F4 event | `-0.751%` | `+34.773%` |
| Weakest formation | `-0.706%` | `-0.108%` |

- Consistency gain: `-0.328%`
- Window byte change vs V242: `-1.268%`
- Short-horizon gate: `0`
- Next method decision: `inspect-source-value-or-routing-latency-after-eta-trust-projection`

## Receiver projection diagnostics

| t | Source to receiver | Selected share | Source r / risk | Target / donor risk | Candidate r | Change in log odds | log eta | Outcome |
|--:|:--|--:|:--|:--|--:|--:|--:|:--|
| 57 | S2 to S13 | 0.05000 | 0.915 / 0.009 | 0.273 / 0.008 | 0.258 | +0.127 | -0.051 | accepted |
| 58 | S1 to S13 | 0.00000 | 0.838 / 0.009 | 0.311 / 0.009 | NaN | +NaN | NaN | transport-drop |
| 59 | S1 to S13 | 0.05000 | 0.977 / 0.008 | 0.355 / 0.008 | 0.272 | +0.003 | -0.243 | accepted |
| 60 | S2 to S13 | 0.01250 | 0.995 / 0.007 | 0.384 / 0.007 | 0.249 | -0.175 | -0.390 | accepted |
| 61 | S3 to S13 | 0.00000 | 0.997 / 0.007 | 0.432 / 0.007 | NaN | +NaN | NaN | transport-drop |
| 62 | S3 to S13 | 0.02500 | 0.989 / 0.008 | 0.456 / 0.006 | 0.312 | -0.097 | -0.274 | accepted |
| 63 | S6 to S13 | 0.05000 | 0.999 / 0.004 | 0.522 / 0.005 | 0.354 | +0.162 | -0.343 | accepted |
| 64 | S5 to S13 | 0.00625 | 0.991 / 0.005 | 0.575 / 0.006 | 0.144 | -0.245 | -0.982 | accepted |
| 65 | S5 to S13 | 0.05000 | 0.997 / 0.004 | 0.633 / 0.006 | 0.154 | -0.234 | -0.770 | accepted |

## Decision

The causal label-only action does not pass the joint mechanism gate. Follow the registered next-method decision before introducing a learned ranker.

## Evidence boundary

V266 is an outcome-opened M24 continuation mechanism screen. It keeps the complete V242 full-posterior backbone and uses the same causal formation, label and donor decision as V265. The complete selected label is charged before receiver projection. Instead of testing only the fixed 0.05 share, the receiver evaluates the frozen descending source-share grid and executes the largest positive label-wise KLA share that satisfies the unchanged eta/existence envelope. The grid is an online trust projection, not an outcome sweep. Centralized risk-synopsis cost is not yet included. No target truth, future measurement or future tracking outcome is available to the controller, so the result can establish mechanism headroom only, not deployment cost, validation or generalization.
