# M24 causal H=3 safe graph-option preflight

- Seed / anchor: `19 / 81`
- Audit mode: `behavior-distribution-shift`
- Return times: `[81 82 83]`
- Source SHA-256: `c6c5ce6cab5d2283b6263082baed70d8695f689a9af30720b749082a8a632388`
- Source behavior: `label-set-message-passing-safe-fixed-e05-a70`
- Low-level model SHA-256: `9d6732320e9f5f846e6dbd46f1e7711da372ffbc95c39a31bbe2174788cc6af4`
- Exact message budget per step: `40`
- Restart equivalence passed: `1`
- Best option: `LLL`
- Best gain versus CCC: `2.977%`

| Option | Mean E-OSPA | Worst | Card err | Consensus | Attempted B | Byte dev. | B3 |
|:--|--:|--:|--:|--:|--:|--:|--:|
| `CCC` | 19.392486 | 49.063992 | 0.597222 | 22.572999 | 5876208 | +0.000% | 1 |
| `LLL` | 18.815079 | 38.100961 | 0.555556 | 21.945670 | 5892696 | +0.281% | 1 |

## Boundary

This opened-training audit compares causal three-step safe graph options on one registered behavior-state distribution. It uses future truth only to form offline return labels and does not establish a learned policy, development, held-out M24 or X36 claim.
