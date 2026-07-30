# M24 causal H=3 safe graph-option preflight

- Seed / anchor: `17 / 81`
- Audit mode: `behavior-distribution-shift`
- Return times: `[81 82 83]`
- Source SHA-256: `bcd1bad02344e9b54d0e55e6e9b65c1c2204863eafe6069b8497a7e5bd84bb0c`
- Source behavior: `fixed-counter-clockwise`
- Low-level model SHA-256: `9d6732320e9f5f846e6dbd46f1e7711da372ffbc95c39a31bbe2174788cc6af4`
- Exact message budget per step: `40`
- Restart equivalence passed: `1`
- Best option: `LLL`
- Best gain versus CCC: `4.728%`

| Option | Mean E-OSPA | Worst | Card err | Consensus | Attempted B | Byte dev. | B3 |
|:--|--:|--:|--:|--:|--:|--:|--:|
| `CCC` | 21.619175 | 46.077427 | 0.638889 | 21.471494 | 5976888 | +0.000% | 1 |
| `LLL` | 20.597103 | 34.799476 | 0.569444 | 20.775866 | 5975904 | -0.016% | 1 |

## Boundary

This opened-training audit compares causal three-step safe graph options on one registered behavior-state distribution. It uses future truth only to form offline return labels and does not establish a learned policy, development, held-out M24 or X36 claim.
