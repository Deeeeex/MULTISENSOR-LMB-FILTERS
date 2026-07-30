# M24 causal H=3 safe graph-option preflight

- Seed / anchor: `27 / 78`
- Audit mode: `quota-weight-headroom`
- Return times: `[78 79 80]`
- Source SHA-256: `3c7ef8d12001346e5acdf254558e2aaa1df488c9e360b5b9214877238643b94b`
- Source behavior: `label-set-message-passing-safe-fixed-e05-a70`
- Low-level model SHA-256: `9d6732320e9f5f846e6dbd46f1e7711da372ffbc95c39a31bbe2174788cc6af4`
- Exact message budget per step: `40`
- Restart equivalence passed: `1`
- Best option: `LLL`
- Best gain versus CCC: `3.371%`

- Registered projection failures: `0`

| Option | Status | Mean E-OSPA | Worst | Card err | Consensus | Attempted B | Byte dev. | B3 |
|:--|:--|--:|--:|--:|--:|--:|--:|--:|
| `CCC` | completed | 17.483976 | 32.310833 | 0.458333 | 20.086105 | 6031560 | +0.000% | 1 |
| `LLL` | completed | 16.894561 | 30.932400 | 0.416667 | 20.109982 | 6044496 | +0.214% | 1 |
| `Q3E10-Q3E10-Q3E10` | completed | 17.791010 | 30.916166 | 0.458333 | 20.455146 | 6039936 | +0.139% | 1 |
| `Q3E20-Q3E20-Q3E20` | completed | 18.130244 | 34.999710 | 0.500000 | 20.904096 | 6038568 | +0.116% | 1 |
| `Q3E25-Q3E25-Q3E25` | completed | 17.854130 | 31.045223 | 0.472222 | 20.848059 | 6035208 | +0.060% | 1 |
| `Q6E10-Q6E10-Q6E10` | completed | 16.917613 | 30.981631 | 0.416667 | 20.193568 | 6037392 | +0.097% | 1 |
| `Q6E20-Q6E20-Q6E20` | completed | 17.926947 | 30.979857 | 0.486111 | 21.016386 | 6023112 | -0.140% | 1 |
| `Q6E25-Q6E25-Q6E25` | completed | 19.028736 | 37.645985 | 0.541667 | 21.703308 | 6018240 | -0.221% | 1 |

## Boundary

This opened-training audit compares causal three-step safe graph options on one registered behavior-state distribution. It uses future truth only to form offline return labels and does not establish a learned policy, development, held-out M24 or X36 claim.
