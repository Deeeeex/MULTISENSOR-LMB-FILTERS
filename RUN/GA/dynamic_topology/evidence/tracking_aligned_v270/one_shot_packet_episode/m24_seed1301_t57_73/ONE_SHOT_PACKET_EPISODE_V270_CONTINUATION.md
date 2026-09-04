# V270 one-shot packet episode

- Scene / seed: `m24-formation-fov-temporal-coupled-formation-braid / 1301`
- Generation commit: `da5f7f8935d66db3057672661c670cedc16d27ee`
- Continuation: `t=57--73` from the V259 V242 state
- First-hop policy-active pages: `[57 58 59 68]`
- Final-fusion applied pages: `58`
- First-hop episode-suppressed pages: `[58 59]`
- First-hop scheduled / delivered pages: `[57 68] / 57`
- Second-hop scheduled / delivered pages: `58 / 58`
- Label payload attempted / delivered bytes: `6432 / 5520`
- Mechanism / full-M24 authorization: `0 / 0`

| Arm | E-OSPA | RMSE | Consistency | F4 event E / RMSE | Window bytes | Spliced static saving |
|:--|--:|--:|--:|:--|--:|--:|
| V242 minimum backbone | 118.651 | 16.844 | 130.254 | 123.683 / 26.636 | 2991480 | 10.041% |
| V270 one-shot packet episode | 118.704 | 15.358 | 130.419 | 123.911 / 20.322 | 3022728 | 9.964% |

## Gains over V242

| Scope | E-OSPA | RMSE |
|:--|--:|--:|
| Network | `-0.045%` | `+8.820%` |
| F4 event | `-0.184%` | `+23.705%` |
| Weakest formation | `-0.173%` | `-0.000%` |

- Consistency gain: `-0.127%`
- Window byte change vs V242: `-1.045%`
- Short-horizon gate: `0`
- Next method decision: `close-source-preserving-spatial-packet-action-family`

## Packet transport diagnostics

| t | First hop | First outcome | Second hop (origin, age) | Second outcome | Attempted / delivered bytes |
|--:|:--|:--|:--|:--|--:|
| 57 | S2 to S13 | stored-without-relay-fusion | - | - | 2760 / 2760 |
| 58 | - | - | S13 to S24 (S2, age 1, r 0.655) | delivered-and-fused | 2760 / 2760 |
| 68 | S4 to S13 | first-hop-transport-drop | - | - | 912 / 0 |

## Receiver projection diagnostics

| t | Source to receiver | Selected share | Source r / risk | Target / donor risk | Reference / candidate r | Change in log odds | log eta | Outcome |
|--:|:--|--:|:--|:--|--:|--:|--:|:--|
| 58 | S2 to S24 | 0.05000 | 0.838 / 0.009 | 0.311 / 0.009 | 0.634 / 0.630 | -0.015 | -0.101 | accepted |

## Decision

The source-preserving packet does not pass the joint mechanism gate. Use the registered failure decision before changing the final beneficiary or source rule.

## Evidence boundary

V270 is an outcome-opened M24 continuation causal ablation. It keeps the V269 existence-margin beneficiary and every V268 packet/fusion semantic, but admits at most one successfully stored packet during a continuous target-formation/label risk episode. A failed first hop does not consume the episode; the latch resets only when the policy becomes inactive or the target-formation/label key changes. The controller uses no truth, realized future delivery or future outcome. Both physical hops remain charged and centralized synopsis cost remains excluded, so the result is mechanism evidence only.
