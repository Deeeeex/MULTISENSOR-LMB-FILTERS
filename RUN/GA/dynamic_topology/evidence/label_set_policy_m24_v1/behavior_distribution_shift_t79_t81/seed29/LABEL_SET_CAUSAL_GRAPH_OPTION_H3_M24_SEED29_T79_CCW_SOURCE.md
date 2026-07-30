# M24 causal H=3 safe graph-option preflight

- Seed / anchor: `29 / 79`
- Audit mode: `behavior-distribution-shift`
- Return times: `[79 80 81]`
- Source SHA-256: `d9f3a25e8c082ccf3e5de2b4ff840aad84e6997a8eb6e219388188639139ca35`
- Source behavior: `fixed-counter-clockwise`
- Low-level model SHA-256: `9d6732320e9f5f846e6dbd46f1e7711da372ffbc95c39a31bbe2174788cc6af4`
- Exact message budget per step: `40`
- Restart equivalence passed: `1`
- Best option: `CCC`
- Best gain versus CCC: `0.000%`

| Option | Mean E-OSPA | Worst | Card err | Consensus | Attempted B | Byte dev. | B3 |
|:--|--:|--:|--:|--:|--:|--:|--:|
| `CCC` | 14.209230 | 34.926187 | 0.361111 | 18.297602 | 6116448 | +0.000% | 1 |
| `LLL` | 14.359384 | 35.014654 | 0.375000 | 18.612514 | 6116448 | +0.000% | 1 |

## Boundary

This opened-training audit compares causal three-step safe graph options on one registered behavior-state distribution. It uses future truth only to form offline return labels and does not establish a learned policy, development, held-out M24 or X36 claim.
