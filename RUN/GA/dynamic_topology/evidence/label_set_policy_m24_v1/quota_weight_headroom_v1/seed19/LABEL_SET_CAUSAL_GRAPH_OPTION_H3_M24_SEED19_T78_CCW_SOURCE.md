# M24 causal H=3 safe graph-option preflight

- Seed / anchor: `19 / 78`
- Audit mode: `quota-weight-headroom`
- Return times: `[78 79 80]`
- Source SHA-256: `d1d4a88021ceea4d547668dbfa1c5f31db2d82e245d553c1c5e3d64e2ffa02b9`
- Source behavior: `fixed-counter-clockwise`
- Low-level model SHA-256: `9d6732320e9f5f846e6dbd46f1e7711da372ffbc95c39a31bbe2174788cc6af4`
- Exact message budget per step: `40`
- Restart equivalence passed: `1`
- Best option: `CCC`
- Best gain versus CCC: `0.000%`

- Registered projection failures: `0`

| Option | Status | Mean E-OSPA | Worst | Card err | Consensus | Attempted B | Byte dev. | B3 |
|:--|:--|--:|--:|--:|--:|--:|--:|--:|
| `CCC` | completed | 19.856173 | 38.877235 | 0.638889 | 23.436646 | 5966592 | +0.000% | 1 |
| `LLL` | completed | 20.285314 | 40.545148 | 0.680556 | 24.067267 | 5961048 | -0.093% | 1 |
| `Q3E10-Q3E10-Q3E10` | completed | 20.858002 | 44.589391 | 0.736111 | 25.552552 | 5978256 | +0.195% | 1 |
| `Q3E20-Q3E20-Q3E20` | completed | 22.808331 | 52.704678 | 0.902778 | 28.839704 | 5952240 | -0.241% | 1 |
| `Q3E25-Q3E25-Q3E25` | completed | 22.729937 | 50.402957 | 0.888889 | 28.648514 | 5951544 | -0.252% | 1 |
| `Q6E10-Q6E10-Q6E10` | completed | 22.821084 | 44.630271 | 0.861111 | 27.376673 | 5936040 | -0.512% | 1 |
| `Q6E20-Q6E20-Q6E20` | completed | 25.834474 | 50.417612 | 1.138889 | 32.493163 | 5942544 | -0.403% | 1 |
| `Q6E25-Q6E25-Q6E25` | completed | 25.165390 | 54.952312 | 1.083333 | 31.904191 | 5909136 | -0.963% | 1 |

## Boundary

This opened-training audit compares causal three-step safe graph options on one registered behavior-state distribution. It uses future truth only to form offline return labels and does not establish a learned policy, development, held-out M24 or X36 claim.
