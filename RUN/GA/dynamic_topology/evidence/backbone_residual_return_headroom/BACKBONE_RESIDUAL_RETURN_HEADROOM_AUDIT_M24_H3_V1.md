# Backbone residual H=3 return-headroom audit

- Protocol: `backbone-preserving-residual-return-headroom-m24-h3-v1`
- Proposal SHA-256: `64ca50d68ffddf823c1204ad44026c954e87b64b24743f12fb93887dd2e22a97`
- Candidates: `40`
- Gate passed: `0`
- Return-data generation authorized: `0`
- Evidence boundary: The best first action was selected after observing paired three-step outcomes, so this is a nondeployable development upper bound. A pass authorizes broader return-data generation only; it does not validate a learned policy or X36 transfer.

## Controls

| Control | E-OSPA | Worst | Bytes |
|---|---:|---:|---:|
| `local` | 23.4600 | 42.0469 | 0 |
| `fixed-index-w70` | 19.2470 | 34.6420 | 3512112 |
| `static-residual` | 18.8201 | 34.6311 | 5868888 |
| `scheduled-residual` | 19.4752 | 34.6311 | 5866488 |
| `analytic-residual` | 18.7966 | 38.6978 | 5860992 |

## Best candidate and gates

- Candidate: `38`, `truth-current-ban-227`
- Family: `offline-truth-current-risk-diverse`
- Graph SHA-256: `3965f79751d05c902758b5d066784e7357a0157f73600aa40aabaf6244b7a875`
- E-OSPA / worst: `18.9375 / 34.6311`
- Minimum baseline mean gain: `-0.75%`
- Minimum baseline tail gain: `0.00%`
- Maximum byte deviation: `0.05%`
- Mean / tail / byte / safety / runtime gates: `0 / 1 / 1 / 1 / 1`

## Top candidates

| Rank | Index | Proposal | Family | E-OSPA | Worst | Bytes | Offline truth |
|---:|---:|---|---|---:|---:|---:|---:|
| 1 | 38 | `truth-current-ban-227` | `offline-truth-current-risk-diverse` | 18.9375 | 34.6311 | 5869176 | 1 |
| 2 | 40 | `truth-current-ban-167-227` | `offline-truth-current-risk-diverse` | 18.9375 | 34.6311 | 5869848 | 1 |
| 3 | 35 | `truth-current-base` | `offline-truth-current-risk` | 18.9377 | 34.6311 | 5868528 | 1 |
| 4 | 37 | `truth-current-ban-167` | `offline-truth-current-risk-diverse` | 18.9377 | 34.6311 | 5869200 | 1 |
| 5 | 39 | `truth-current-ban-143-227` | `offline-truth-current-risk-diverse` | 18.9383 | 34.6311 | 5869680 | 1 |
| 6 | 36 | `truth-current-ban-143` | `offline-truth-current-risk-diverse` | 18.9384 | 34.6311 | 5869032 | 1 |
| 7 | 34 | `posterior-analytic` | `truth-free-posterior-analytic` | 18.9391 | 34.6311 | 5861256 | 0 |
| 8 | 29 | `observable-71-receiver-rescue` | `truth-free-observable-basis` | 18.9398 | 34.6311 | 5861736 | 0 |
| 9 | 28 | `observable-70-receiver-rescue` | `truth-free-observable-basis` | 18.9432 | 34.6311 | 5861064 | 0 |
| 10 | 24 | `observable-66-compatibility-conservative` | `truth-free-observable-basis` | 18.9434 | 34.6311 | 5865144 | 0 |
