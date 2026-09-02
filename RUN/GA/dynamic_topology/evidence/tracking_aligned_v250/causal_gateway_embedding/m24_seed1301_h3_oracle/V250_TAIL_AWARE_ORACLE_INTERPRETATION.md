# V250 tail-aware H=3 oracle interpretation

- Scene / seed: `m24-formation-fov-temporal-coupled-formation-braid / 1301`
- Oracle source commit: `3248df0edeba8e8e24d897821a4922522cdec5a8`
- Analysis source commit: `e76b13524f2dda5b2bff219d6bd6a3463d2cc8c4`
- Old / tail-aware aligned anchors: `1 / 3` of `3`
- Tail-aware teacher / ridge / GNN authorized: `1 / 1 / 0`

## Decision

The original network-minimum selector is not aligned with the diagnosed V248 failure: it improves the reference-worst formation at only 1 of 3 anchors. The revised teacher first requires positive network E-OSPA, RMSE and consistency gains, nonincreasing attempted bytes and the existing 2.0% formation regression cap. Among those arms it requires nondegrading E-OSPA and positive RMSE gain for the reference-worst formation, then maximizes that formation RMSE gain. This selects a tail-relevant action at every anchor and authorizes ridge labels, not an end-to-end or generalization claim.

| Anchor | Reference-worst formation / RMSE | Old selected | Tail-aware selected | Type | Network E / R / C | Byte saving | Target formation E / R | Weakest any formation E / R |
|--:|:--|--:|--:|:--|:--|--:|:--|:--|
| 70 | F4 / 25.832 | 4 | 11 | receiver-local-assignment | +0.006% / +0.644% / +0.371% | +0.224% | +0.023% / +1.537% | -0.000% / -0.143% |
| 84 | F2 / 19.153 | 17 | 20 | single-directed-arc | +0.139% / +1.932% / +0.003% | +0.037% | +0.001% / +0.180% | +0.000% / +0.000% |
| 151 | F1 / 23.918 | 13 | 13 | receiver-local-assignment | +0.219% / +0.262% / +1.440% | +0.068% | +0.000% / +0.011% | +0.000% / +0.000% |

## Aggregate comparison

| Metric | Original selector | Tail-aware selector |
|:--|--:|--:|
| E-OSPA gain | `+0.096%` | `+0.120%` |
| RMSE gain | `+1.316%` | `+0.855%` |
| Consistency gain | `+0.914%` | `+0.572%` |
| Attempted-byte saving | `+0.322%` | `+0.094%` |
| Weakest formation E-OSPA | `+0.000%` | `-0.000%` |
| Weakest formation RMSE | `+0.000%` | `-0.035%` |

| Formation | Tail-aware E-OSPA gain | Tail-aware RMSE gain |
|--:|--:|--:|
| F1 | +0.182682% | +1.877255% |
| F2 | +0.282258% | +0.405920% |
| F3 | -0.000056% | -0.034869% |
| F4 | +0.007813% | +0.819220% |

## Evidence boundary

This is a post-hoc selection over already completed paired H=3 arms from one M24 development seed. Truth and future outcomes are used only to create offline teacher labels. The result shows that the fixed-budget gateway action space contains localization-tail-relevant actions; it is not a deployable policy, a full-episode best method, an X36 result or a paper-level conclusion. The persistent current-best table therefore remains unchanged.
