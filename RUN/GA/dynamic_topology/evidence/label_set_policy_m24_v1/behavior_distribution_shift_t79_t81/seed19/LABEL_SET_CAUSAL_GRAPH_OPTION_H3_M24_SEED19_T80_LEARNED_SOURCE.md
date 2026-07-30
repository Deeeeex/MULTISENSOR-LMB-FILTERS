# M24 causal H=3 safe graph-option preflight

- Seed / anchor: `19 / 80`
- Audit mode: `behavior-distribution-shift`
- Return times: `[80 81 82]`
- Source SHA-256: `c6c5ce6cab5d2283b6263082baed70d8695f689a9af30720b749082a8a632388`
- Source behavior: `label-set-message-passing-safe-fixed-e05-a70`
- Low-level model SHA-256: `9d6732320e9f5f846e6dbd46f1e7711da372ffbc95c39a31bbe2174788cc6af4`
- Exact message budget per step: `40`
- Restart equivalence passed: `1`
- Best option: `LLL`
- Best gain versus CCC: `5.427%`

| Option | Mean E-OSPA | Worst | Card err | Consensus | Attempted B | Byte dev. | B3 |
|:--|--:|--:|--:|--:|--:|--:|--:|
| `CCC` | 17.713539 | 48.992780 | 0.541667 | 21.966563 | 5906832 | +0.000% | 1 |
| `LLL` | 16.752250 | 37.619064 | 0.486111 | 21.038140 | 5917608 | +0.182% | 1 |

## Boundary

This opened-training audit compares causal three-step safe graph options on one registered behavior-state distribution. It uses future truth only to form offline return labels and does not establish a learned policy, development, held-out M24 or X36 claim.
