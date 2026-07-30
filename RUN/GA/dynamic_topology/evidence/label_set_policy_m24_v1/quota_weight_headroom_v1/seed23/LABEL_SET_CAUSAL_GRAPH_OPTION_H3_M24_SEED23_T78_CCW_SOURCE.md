# M24 causal H=3 safe graph-option preflight

- Seed / anchor: `23 / 78`
- Audit mode: `quota-weight-headroom`
- Return times: `[78 79 80]`
- Source SHA-256: `e7677e4962c65842a50ea196348d8380b1160b7fe925dacfa478f54c45fc5e96`
- Source behavior: `fixed-counter-clockwise`
- Low-level model SHA-256: `9d6732320e9f5f846e6dbd46f1e7711da372ffbc95c39a31bbe2174788cc6af4`
- Exact message budget per step: `40`
- Restart equivalence passed: `1`
- Best option: `LLL`
- Best gain versus CCC: `19.620%`

- Registered projection failures: `0`

| Option | Status | Mean E-OSPA | Worst | Card err | Consensus | Attempted B | Byte dev. | B3 |
|:--|:--|--:|--:|--:|--:|--:|--:|--:|
| `CCC` | completed | 7.341908 | 30.494389 | 0.083333 | 10.138767 | 6055968 | +0.000% | 1 |
| `LLL` | completed | 5.901460 | 13.736025 | 0.027778 | 7.549215 | 6055968 | +0.000% | 1 |
| `Q3E10-Q3E10-Q3E10` | completed | 6.793761 | 17.667423 | 0.069444 | 9.155220 | 6055296 | -0.011% | 1 |
| `Q3E20-Q3E20-Q3E20` | completed | 9.460494 | 30.134303 | 0.180556 | 13.492525 | 6047856 | -0.134% | 1 |
| `Q3E25-Q3E25-Q3E25` | completed | 10.076129 | 30.431502 | 0.208333 | 14.709562 | 6051192 | -0.079% | 1 |
| `Q6E10-Q6E10-Q6E10` | completed | 7.513791 | 26.215486 | 0.097222 | 10.452606 | 6061344 | +0.089% | 1 |
| `Q6E20-Q6E20-Q6E20` | completed | 11.508863 | 38.590877 | 0.291667 | 16.677598 | 6038352 | -0.291% | 1 |
| `Q6E25-Q6E25-Q6E25` | completed | 12.893759 | 30.572322 | 0.347222 | 18.631841 | 6042384 | -0.224% | 1 |

## Boundary

This opened-training audit compares causal three-step safe graph options on one registered behavior-state distribution. It uses future truth only to form offline return labels and does not establish a learned policy, development, held-out M24 or X36 claim.
