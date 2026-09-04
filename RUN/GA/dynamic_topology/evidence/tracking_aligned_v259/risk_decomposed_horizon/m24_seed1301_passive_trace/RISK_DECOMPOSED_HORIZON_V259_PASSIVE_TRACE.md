# V259 passive risk-decomposition trace

- Scene / seed: `m24-formation-fov-temporal-coupled-formation-braid / 1301`
- Generation commit: `c34983dcd0a08f7576ea7d48f6b15754c3f14518`
- Passive V242 reproduction passed: `1`
- Development evidence only: `1`

| Passive arm | E-OSPA | RMSE | Focus consistency | Bytes | Messages / step |
|:--|--:|--:|--:|--:|--:|
| V242 with passive capture | 122.462 | 12.183 | 131.664 | 36675624 | 30--30 |

Maximum reproduced E-OSPA/RMSE array difference: `0 / 0`; byte difference: `+0`.

## Opened diagnostic anchors

| Time | Highest localization-tail formation / score | Highest support-prominence formation / score | Its coverage deficit | Its active-count deficit |
|--:|:--|:--|--:|--:|
| 70 | F4 / 0.514 | F3 / 0.037 | 0.091 | 0.130 |
| 84 | F2 / 0.168 | F3 / 0.092 | 0.385 | 0.385 |
| 151 | F2 / 0.954 | F1 / 0.292 | 0.857 | 0.724 |

## Evidence boundary

V259 passively captures every pre-topology V242 local posterior on the already opened corrected M24 seed. Formation risk summaries use only current LMB posteriors, formation membership and the registered OSPA cutoff. Recorded truth-level errors are joined only offline. The trace may guide a causal controller but cannot establish tracking gain, held-out generalization or a paper claim.
