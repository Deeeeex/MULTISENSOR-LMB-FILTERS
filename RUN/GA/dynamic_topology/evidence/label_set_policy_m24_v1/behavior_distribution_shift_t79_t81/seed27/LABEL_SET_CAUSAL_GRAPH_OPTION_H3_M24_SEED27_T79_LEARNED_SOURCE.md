# M24 causal H=3 safe graph-option preflight

- Seed / anchor: `27 / 79`
- Audit mode: `behavior-distribution-shift`
- Return times: `[79 80 81]`
- Source SHA-256: `3c7ef8d12001346e5acdf254558e2aaa1df488c9e360b5b9214877238643b94b`
- Source behavior: `label-set-message-passing-safe-fixed-e05-a70`
- Low-level model SHA-256: `9d6732320e9f5f846e6dbd46f1e7711da372ffbc95c39a31bbe2174788cc6af4`
- Exact message budget per step: `40`
- Restart equivalence passed: `1`
- Best option: `CCC`
- Best gain versus CCC: `0.000%`

| Option | Mean E-OSPA | Worst | Card err | Consensus | Attempted B | Byte dev. | B3 |
|:--|--:|--:|--:|--:|--:|--:|--:|
| `CCC` | 17.680101 | 34.661431 | 0.458333 | 20.633898 | 6077832 | +0.000% | 1 |
| `LLL` | 17.709502 | 31.390371 | 0.458333 | 20.649190 | 6073632 | -0.069% | 1 |

## Boundary

This opened-training audit compares causal three-step safe graph options on one registered behavior-state distribution. It uses future truth only to form offline return labels and does not establish a learned policy, development, held-out M24 or X36 claim.
