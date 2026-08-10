# V77 centered perturbation-recovery replay

- Common mode constrained: `0`
- Centered total energy must be monotone non-increasing: `1`
- Prediction / measurement / future link / truth: `0 / 0 / 0 / 0`

## m24-formation-fov-merge-split / t=80

- Direct-safe formations replayed: `3`

### Historical exact V71/V72 route

- Applied slot triples: `[14 10 21;17 19 3]`

| Round | Common energy | Centered energy | Common fraction | Existence centered | Spatial centered |
|--:|--:|--:|--:|--:|--:|
| 1 | 0.000124346492 | 0.00503634255 | 2.409% | 0.00473285236 | 0.000303490187 |
| 2 | 0.000184253457 | 0.0055767847 | 3.198% | 0.00548489322 | 9.18914789e-05 |
| 3 | 0.000824322922 | 0.00405308025 | 16.901% | 0.00375676706 | 0.00029631319 |

- Total / existence / spatial centered monotone: `0 / 0 / 0`
- Terminal centered contraction: `+19.523%`
- Centered-recovery certificate: `0`

### Prospective exact V73 route

- Applied slot triples: `[14 10 21;17 19 20]`

| Round | Common energy | Centered energy | Common fraction | Existence centered | Spatial centered |
|--:|--:|--:|--:|--:|--:|
| 1 | 0.000171259918 | 0.00400088151 | 4.105% | 0.00385863271 | 0.000142248798 |
| 2 | 0.000342062665 | 0.00421285577 | 7.510% | 0.00413685034 | 7.60054361e-05 |
| 3 | 0.000868660408 | 0.00416493049 | 17.257% | 0.00385903388 | 0.000305896613 |

- Total / existence / spatial centered monotone: `0 / 0 / 0`
- Terminal centered contraction: `-4.100%`
- Centered-recovery certificate: `0`

## x36-formation-fov-merge-split / t=52

- Direct-safe formations replayed: `4`

### Historical exact V71/V72 route

- Applied slot triples: `[20 16 34;23 25 35]`

| Round | Common energy | Centered energy | Common fraction | Existence centered | Spatial centered |
|--:|--:|--:|--:|--:|--:|
| 1 | 5.16299831e-05 | 0.00179825365 | 2.791% | 0.00170655996 | 9.16936908e-05 |
| 2 | 5.73558676e-05 | 0.00144706006 | 3.813% | 0.0013923478 | 5.4712262e-05 |
| 3 | 0.000378331247 | 0.00289740391 | 11.550% | 0.0027795802 | 0.000117823702 |

- Total / existence / spatial centered monotone: `0 / 0 / 0`
- Terminal centered contraction: `-61.123%`
- Centered-recovery certificate: `0`

### Prospective exact V73 route

- Applied slot triples: `[20 16 34;23 25 36]`

| Round | Common energy | Centered energy | Common fraction | Existence centered | Spatial centered |
|--:|--:|--:|--:|--:|--:|
| 1 | 5.74972256e-05 | 0.00198748278 | 2.812% | 0.0018956884 | 9.17943738e-05 |
| 2 | 5.96173922e-05 | 0.00147559692 | 3.883% | 0.00142087291 | 5.4724008e-05 |
| 3 | 0.000378453458 | 0.00289953618 | 11.545% | 0.00278171342 | 0.000117822761 |

- Total / existence / spatial centered monotone: `0 / 0 / 0`
- Terminal centered contraction: `-45.890%`
- Centered-recovery certificate: `0`

## Summary

- All historical centered-recovery certificates pass: `0`
- All aligned centered-recovery certificates pass: `0`
- Route executed / tracking outcome read: `0 / 0`
- Validation claim allowed: `0`

## Evidence boundary

V77 repeats the frozen V76 source-only virtual KLA replay but replaces distance-to-reference recovery with a label-wise modal decomposition. Candidate-minus-reference existence and position perturbations are split across network nodes into a common mode and a centered disagreement mode. The common mode is unconstrained; only total centered energy must be non-increasing. Fusion uses the registered directed weights, formal mixture-aware heavy receiver, and deterministic current-link reliability weighting. No packet draw, prediction, target motion, new measurement, truth, future link page, route execution, tracking outcome, or model training is used.
