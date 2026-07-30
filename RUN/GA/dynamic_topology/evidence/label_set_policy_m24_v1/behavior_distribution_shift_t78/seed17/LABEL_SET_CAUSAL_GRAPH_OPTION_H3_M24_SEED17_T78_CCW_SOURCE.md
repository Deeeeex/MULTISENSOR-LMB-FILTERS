# M24 causal H=3 safe graph-option preflight

- Seed / anchor: `17 / 78`
- Audit mode: `behavior-distribution-shift`
- Return times: `[78 79 80]`
- Source SHA-256: `bcd1bad02344e9b54d0e55e6e9b65c1c2204863eafe6069b8497a7e5bd84bb0c`
- Source behavior: `fixed-counter-clockwise`
- Low-level model SHA-256: `9d6732320e9f5f846e6dbd46f1e7711da372ffbc95c39a31bbe2174788cc6af4`
- Exact message budget per step: `40`
- Restart equivalence passed: `1`
- Best option: `LLL`
- Best gain versus CCC: `7.932%`

| Option | Mean E-OSPA | Worst | Card err | Consensus | Attempted B | Byte dev. | B3 |
|:--|--:|--:|--:|--:|--:|--:|--:|
| `CCC` | 16.191609 | 34.698717 | 0.416667 | 19.294103 | 5971776 | +0.000% | 1 |
| `LLL` | 14.907267 | 34.717352 | 0.361111 | 19.004219 | 6004032 | +0.540% | 1 |

## Boundary

This opened-training audit compares causal three-step safe graph options on one registered behavior-state distribution. It uses future truth only to form offline return labels and does not establish a learned policy, development, held-out M24 or X36 claim.
