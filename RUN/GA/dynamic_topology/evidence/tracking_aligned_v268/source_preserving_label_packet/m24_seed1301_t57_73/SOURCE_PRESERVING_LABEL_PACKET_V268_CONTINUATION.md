# V268 source-preserving label packet

- Scene / seed: `m24-formation-fov-temporal-coupled-formation-braid / 1301`
- Generation commit: `2b28f361d47a1e2758a80ba11b9e981e6ac45037`
- Continuation: `t=57--73` from the V259 V242 state
- First-hop policy-active pages: `[57 58 59 60]`
- Final-fusion applied pages: `[58 60]`
- First-hop scheduled / delivered pages: `[57 58 59 60] / [57 59 60]`
- Second-hop scheduled / delivered pages: `[58 60 61] / [58 60 61]`
- Label payload attempted / delivered bytes: `9072 / 8496`
- Mechanism / full-M24 authorization: `0 / 0`

| Arm | E-OSPA | RMSE | Consistency | F4 event E / RMSE | Window bytes | Spliced static saving |
|:--|--:|--:|--:|:--|--:|--:|
| V242 minimum backbone | 118.651 | 16.844 | 130.254 | 123.683 / 26.636 | 2991480 | 10.041% |
| V268 source-preserving label packet | 119.027 | 13.757 | 130.863 | 125.283 / 13.516 | 3026376 | 9.955% |

## Gains over V242

| Scope | E-OSPA | RMSE |
|:--|--:|--:|
| Network | `-0.317%` | `+18.328%` |
| F4 event | `-1.293%` | `+49.255%` |
| Weakest formation | `-1.216%` | `-0.000%` |

- Consistency gain: `-0.468%`
- Window byte change vs V242: `-1.167%`
- Short-horizon gate: `0`
- Next method decision: `close-source-preserving-label-packets-or-redesign-final-beneficiary`

## Packet transport diagnostics

| t | First hop | First outcome | Second hop (origin, age) | Second outcome | Attempted / delivered bytes |
|--:|:--|:--|:--|:--|--:|
| 57 | S2 to S13 | stored-without-relay-fusion | - | - | 2760 / 2760 |
| 58 | S1 to S13 | first-hop-transport-drop | S13 to S22 (S2, age 1) | delivered-and-fused | 3336 / 2760 |
| 59 | S1 to S13 | stored-without-relay-fusion | - | - | 744 / 744 |
| 60 | S2 to S13 | stored-without-relay-fusion | S13 to S22 (S1, age 1) | delivered-and-fused | 1488 / 1488 |
| 61 | - | - | S13 to S22 (S2, age 1) | delivered-projection-rejected | 744 / 744 |

## Receiver projection diagnostics

| t | Source to receiver | Selected share | Source r / risk | Target / donor risk | Reference / candidate r | Change in log odds | log eta | Outcome |
|--:|:--|--:|:--|:--|--:|--:|--:|:--|
| 58 | S2 to S22 | 0.05000 | 0.838 / 0.009 | 0.311 / 0.009 | 0.568 / 0.568 | -0.001 | -0.101 | accepted |
| 60 | S1 to S22 | 0.05000 | 0.995 / 0.007 | 0.279 / 0.007 | 0.573 / 0.522 | -0.205 | -0.385 | accepted |
| 61 | S2 to S22 | 0.00000 | NaN / NaN | NaN / NaN | 0.505 / 0.439 | -0.266 | -0.594 | no-source-weight-passed |

## Decision

The source-preserving packet does not pass the joint mechanism gate. Use the registered failure decision before changing the final beneficiary or source rule.

## Evidence boundary

V268 is an outcome-opened M24 continuation mechanism screen. It keeps the V242 full-posterior backbone, V265 causal source/relay selection, V266 online source-share grid and V267 asymmetric existence-retention rule. A complete selected Bernoulli-GM label is sent over a first real physical hop and cached at the relay without changing the relay LMB. On the next page the original source object is predicted once, sent over a second real physical hop, and fused only at the final target-formation receiver. Both transmissions and the source/time provenance metadata are charged; dropped second hops are not retried. Centralized risk-synopsis cost remains excluded. No truth or future outcome is visible to the controller, so the result is mechanism evidence only, not validation or generalization.
