# V269 existence-margin beneficiary

- Scene / seed: `m24-formation-fov-temporal-coupled-formation-braid / 1301`
- Generation commit: `73da844a8ac4b7d01ce41c5b064e3a9b3f9512f4`
- Continuation: `t=57--73` from the V259 V242 state
- First-hop policy-active pages: `[57 58 59]`
- Final-fusion applied pages: `[58 60]`
- First-hop scheduled / delivered pages: `[57 58 59] / [57 59]`
- Second-hop scheduled / delivered pages: `[58 60] / [58 60]`
- Label payload attempted / delivered bytes: `7584 / 7008`
- Mechanism / full-M24 authorization: `0 / 0`

| Arm | E-OSPA | RMSE | Consistency | F4 event E / RMSE | Window bytes | Spliced static saving |
|:--|--:|--:|--:|:--|--:|--:|
| V242 minimum backbone | 118.651 | 16.844 | 130.254 | 123.683 / 26.636 | 2991480 | 10.041% |
| V269 existence-margin beneficiary | 118.910 | 14.122 | 130.627 | 124.786 / 15.066 | 3024216 | 9.960% |

## Gains over V242

| Scope | E-OSPA | RMSE |
|:--|--:|--:|
| Network | `-0.219%` | `+16.162%` |
| F4 event | `-0.892%` | `+43.436%` |
| Weakest formation | `-0.838%` | `+0.000%` |

- Consistency gain: `-0.287%`
- Window byte change vs V242: `-1.094%`
- Short-horizon gate: `0`
- Next method decision: `close-single-final-beneficiary-packet-family`

## Packet transport diagnostics

| t | First hop | First outcome | Second hop (origin, age) | Second outcome | Attempted / delivered bytes |
|--:|:--|:--|:--|:--|--:|
| 57 | S2 to S13 | stored-without-relay-fusion | - | - | 2760 / 2760 |
| 58 | S1 to S13 | first-hop-transport-drop | S13 to S24 (S2, age 1, r 0.655) | delivered-and-fused | 3336 / 2760 |
| 59 | S1 to S13 | stored-without-relay-fusion | - | - | 744 / 744 |
| 60 | - | - | S13 to S20 (S1, age 1, r 0.632) | delivered-and-fused | 744 / 744 |

## Receiver projection diagnostics

| t | Source to receiver | Selected share | Source r / risk | Target / donor risk | Reference / candidate r | Change in log odds | log eta | Outcome |
|--:|:--|--:|:--|:--|--:|--:|--:|:--|
| 58 | S2 to S24 | 0.05000 | 0.838 / 0.009 | 0.311 / 0.009 | 0.634 / 0.630 | -0.015 | -0.101 | accepted |
| 60 | S1 to S20 | 0.05000 | NaN / NaN | NaN / NaN | 0.579 / 0.631 | +0.219 | -0.026 | accepted |

## Decision

The source-preserving packet does not pass the joint mechanism gate. Use the registered failure decision before changing the final beneficiary or source rule.

## Evidence boundary

V269 is an outcome-opened M24 continuation mechanism screen. It changes only the final beneficiary of V268: among currently physical receivers in the selected target formation, it chooses the sensor with the greatest current existence probability for the selected label, breaking ties by current V242 backbone alignment and physical sensor identity. The source, relay, label, one-page prediction, source-share grid, asymmetric eta guard, two-hop transport and byte charge are unchanged. The decision uses no realized delivery, truth or future outcome. Centralized synopsis cost remains excluded, so the result is mechanism evidence only.
