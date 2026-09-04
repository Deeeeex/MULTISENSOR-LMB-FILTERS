# V260 risk-triggered formation-local pulse

- Scene / seed: `m24-formation-fov-temporal-coupled-formation-braid / 1301`
- Generation commit: `5a876edf058cf0e8e8b64e0e67e656c1dbd85070`
- Continuation: `t=57--73` from the V259 V242 state
- Selected pulse weight: `0.10`
- Mechanism / full-M24 authorization: `0 / 0`

| Arm | E-OSPA | RMSE | Consistency | F4 event E / RMSE | Window bytes | Spliced static saving | Pulse pages | Gate |
|:--|--:|--:|--:|:--|--:|--:|:--|:--:|
| V242 minimum backbone | 118.651 | 16.844 | 130.254 | 123.683 / 26.636 | 2991480 | 10.041% | `--` | -- |
| V260 pulse weight 0.10 | 118.240 | 16.941 | 129.383 | 121.935 / 27.074 | 3222792 | 9.473% | `[57 59 61 63 65 67 69 71 73]` | 0 |
| V260 pulse weight 0.20 | 118.026 | 17.067 | 129.307 | 121.028 / 27.563 | 3223104 | 9.473% | `[57 59 61 63 65 67 69 71 73]` | 0 |

## Candidate gains over V242

| Weight | Network E / R / C | F4 event E / R | Weakest formation E / R | Window byte change |
|--:|:--|:--|:--|--:|
| 0.10 | `+0.347% / -0.575% / +0.669%` | `+1.413% / -1.646%` | `-0.000% / -1.555%` | `-7.732%` |
| 0.20 | `+0.526% / -1.321% / +0.727%` | `+2.147% / -3.482%` | `-0.000% / -3.302%` | `-7.743%` |

## Decision

Neither registered pulse repairs F4 under the bounded short-horizon guard. Reject residual-bundle pulsing before a full episode and reconsider a zero-extra-message local-cycle action.

## Evidence boundary

V260 is an outcome-opened M24 continuation screen. Its thresholds, t=57--73 window and two pulse strengths follow the completed V259 development trace and the earlier structural duty-cycle result. The executed policy reads only current LMB posteriors, current physical topology and past selected routes. It never reads truth or future measurements, but it assumes a centralized current-network risk synopsis whose deployment bytes are not yet included. Tracking outcomes may select the next action family but cannot establish validation, cross-seed gain or generalization.
