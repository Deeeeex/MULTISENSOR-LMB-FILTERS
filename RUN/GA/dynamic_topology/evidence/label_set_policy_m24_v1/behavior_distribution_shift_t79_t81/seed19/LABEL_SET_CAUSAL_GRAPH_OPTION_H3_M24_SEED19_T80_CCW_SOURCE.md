# M24 causal H=3 safe graph-option preflight

- Seed / anchor: `19 / 80`
- Audit mode: `behavior-distribution-shift`
- Return times: `[80 81 82]`
- Source SHA-256: `d1d4a88021ceea4d547668dbfa1c5f31db2d82e245d553c1c5e3d64e2ffa02b9`
- Source behavior: `fixed-counter-clockwise`
- Low-level model SHA-256: `9d6732320e9f5f846e6dbd46f1e7711da372ffbc95c39a31bbe2174788cc6af4`
- Exact message budget per step: `40`
- Restart equivalence passed: `1`
- Best option: `LLL`
- Best gain versus CCC: `3.849%`

| Option | Mean E-OSPA | Worst | Card err | Consensus | Attempted B | Byte dev. | B3 |
|:--|--:|--:|--:|--:|--:|--:|--:|
| `CCC` | 19.918904 | 42.665165 | 0.583333 | 21.894579 | 5815152 | +0.000% | 1 |
| `LLL` | 19.152293 | 30.711960 | 0.541667 | 21.446883 | 5815464 | +0.005% | 1 |

## Boundary

This opened-training audit compares causal three-step safe graph options on one registered behavior-state distribution. It uses future truth only to form offline return labels and does not establish a learned policy, development, held-out M24 or X36 claim.
