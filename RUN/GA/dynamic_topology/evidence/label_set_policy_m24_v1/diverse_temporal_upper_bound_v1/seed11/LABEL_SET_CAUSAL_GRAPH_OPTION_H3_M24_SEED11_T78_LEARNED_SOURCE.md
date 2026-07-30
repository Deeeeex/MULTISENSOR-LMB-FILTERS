# M24 causal H=3 safe graph-option preflight

- Seed / anchor: `11 / 78`
- Audit mode: `diverse-temporal-upper-bound`
- Return times: `[78 79 80]`
- Source SHA-256: `82404ba0ae5b65b9d4ddad6c1b7d91191630d6f0e98eae4a55b42875fd1665c2`
- Source behavior: `label-set-message-passing-safe-fixed-e05-a70`
- Low-level model SHA-256: `9d6732320e9f5f846e6dbd46f1e7711da372ffbc95c39a31bbe2174788cc6af4`
- Exact message budget per step: `40`
- Restart equivalence passed: `1`
- Best option: `C-G6-G6`
- Best gain versus CCC: `3.087%`

| Option | Mean E-OSPA | Worst | Card err | Consensus | Attempted B | Byte dev. | B3 |
|:--|--:|--:|--:|--:|--:|--:|--:|
| `CCC` | 15.371985 | 52.446663 | 0.430556 | 21.153136 | 6032640 | +0.000% | 1 |
| `LLL` | 17.394177 | 44.527197 | 0.555556 | 23.579707 | 6090936 | +0.966% | 1 |
| `G6-G6-G6` | 16.516952 | 45.231018 | 0.472222 | 21.765549 | 6101040 | +1.134% | 1 |
| `G6-C-C` | 16.072729 | 45.173500 | 0.472222 | 21.789462 | 6099696 | +1.112% | 1 |
| `G6-G6-C` | 16.977678 | 45.224217 | 0.513889 | 22.608991 | 6101040 | +1.134% | 1 |
| `C-G6-G6` | 14.897434 | 49.293484 | 0.402778 | 20.434694 | 6035664 | +0.050% | 1 |
| `C-C-G6` | 15.212520 | 52.441551 | 0.416667 | 20.866626 | 6032640 | +0.000% | 1 |
| `C-G6-C` | 15.019614 | 52.435148 | 0.416667 | 20.715603 | 6035664 | +0.050% | 1 |

## Boundary

This opened-training audit compares causal three-step safe graph options on one registered behavior-state distribution. It uses future truth only to form offline return labels and does not establish a learned policy, development, held-out M24 or X36 claim.
