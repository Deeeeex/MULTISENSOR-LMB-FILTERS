# V265 label-selective risk shortcut

- Scene / seed: `m24-formation-fov-temporal-coupled-formation-braid / 1301`
- Generation commit: `5d62da5960cefb30f9a5af69aa74c6b715fba174`
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

## Receiver projection diagnostics

| t | Source to receiver | Source r / risk | Target / donor risk | Candidate r | Change in log odds | log eta | Outcome |
|--:|:--|:--|:--|--:|--:|--:|:--|
| 57 | S2 to S13 | 0.915 / 0.009 | 0.273 / 0.008 | 0.258 | +0.127 | -0.051 | accepted |
| 58 | S1 to S13 | 0.838 / 0.009 | 0.311 / 0.009 | NaN | +NaN | NaN | transport-drop |
| 59 | S1 to S13 | 0.977 / 0.008 | 0.355 / 0.008 | 0.272 | +0.003 | -0.243 | accepted |
| 60 | S2 to S13 | 0.995 / 0.007 | 0.384 / 0.007 | 0.210 | -0.399 | -0.845 | eta-existence-lower-bound-failed |
| 61 | S3 to S13 | 0.997 / 0.007 | 0.432 / 0.007 | NaN | +NaN | NaN | transport-drop |
| 62 | S3 to S13 | 0.989 / 0.008 | 0.456 / 0.006 | 0.240 | -0.510 | -0.795 | eta-existence-lower-bound-failed |
| 63 | S6 to S13 | 0.999 / 0.004 | 0.522 / 0.005 | 0.176 | -1.036 | -1.408 | eta-existence-lower-bound-failed |
| 64 | S5 to S13 | 0.991 / 0.005 | 0.588 / 0.006 | 0.146 | -0.942 | -1.232 | eta-existence-lower-bound-failed |
| 65 | S5 to S13 | 0.997 / 0.004 | 0.647 / 0.006 | 0.143 | -0.736 | -1.110 | eta-existence-lower-bound-failed |
| 66 | S5 to S13 | 0.990 / 0.005 | 0.718 / 0.007 | 0.163 | -0.483 | -0.781 | eta-existence-lower-bound-failed |
| 67 | S5 to S13 | 0.985 / 0.005 | 0.794 / 0.009 | 0.155 | -0.628 | -0.912 | eta-existence-lower-bound-failed |
| 68 | S4 to S13 | 1.000 / 0.006 | 0.884 / 0.009 | NaN | +NaN | NaN | transport-drop |
| 69 | S4 to S13 | 0.989 / 0.009 | 0.967 / 0.009 | NaN | +NaN | NaN | transport-drop |

## Decision

The causal label-only shortcut does not repair the diagnosed event under the registered guards. Inspect whether the failure is source selection, one-hop latency or insufficient label weight before introducing a learned ranker.

## Evidence boundary

V265 is an outcome-opened M24 continuation mechanism screen. It keeps the complete V242 full-posterior backbone and its fusion weights unchanged. The causal V261 current-posterior risk rule selects one formation, one label and a lower-risk donor formation. Only that complete Bernoulli Gaussian-mixture label is sent across the first physical hop of the shorter path and added with zero global topology mass to one ordinary label-wise KLA call. A fixed V242 residual share is assigned only to that label, and the existing receiver-side eta projection falls back to ordinary fusion if the fused existence log odds leave the registered envelope. The complete label payload is charged even when the projection rejects it; centralized risk-synopsis cost is not yet included. The controller uses no target truth, future measurement or future tracking outcome, so this result can establish mechanism headroom only, not deployment cost, validation or generalization.
