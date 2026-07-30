# M24 causal H=3 safe graph-option preflight

- Seed / anchor: `17 / 80`
- Audit mode: `behavior-distribution-shift`
- Return times: `[80 81 82]`
- Source SHA-256: `bcd1bad02344e9b54d0e55e6e9b65c1c2204863eafe6069b8497a7e5bd84bb0c`
- Source behavior: `fixed-counter-clockwise`
- Low-level model SHA-256: `9d6732320e9f5f846e6dbd46f1e7711da372ffbc95c39a31bbe2174788cc6af4`
- Exact message budget per step: `40`
- Restart equivalence passed: `1`
- Best option: `LLL`
- Best gain versus CCC: `7.106%`

| Option | Mean E-OSPA | Worst | Card err | Consensus | Attempted B | Byte dev. | B3 |
|:--|--:|--:|--:|--:|--:|--:|--:|
| `CCC` | 20.909655 | 38.885594 | 0.611111 | 21.021046 | 6013752 | +0.000% | 1 |
| `LLL` | 19.423743 | 34.830035 | 0.527778 | 20.217681 | 6021672 | +0.132% | 1 |

## Boundary

This opened-training audit compares causal three-step safe graph options on one registered behavior-state distribution. It uses future truth only to form offline return labels and does not establish a learned policy, development, held-out M24 or X36 claim.
