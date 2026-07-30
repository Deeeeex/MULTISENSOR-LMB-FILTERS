# M24 causal H=3 safe graph-option preflight

- Seed / anchor: `19 / 78`
- Audit mode: `behavior-distribution-shift`
- Return times: `[78 79 80]`
- Source SHA-256: `c6c5ce6cab5d2283b6263082baed70d8695f689a9af30720b749082a8a632388`
- Source behavior: `label-set-message-passing-safe-fixed-e05-a70`
- Low-level model SHA-256: `9d6732320e9f5f846e6dbd46f1e7711da372ffbc95c39a31bbe2174788cc6af4`
- Exact message budget per step: `40`
- Restart equivalence passed: `1`
- Best option: `LLL`
- Best gain versus CCC: `5.211%`

| Option | Mean E-OSPA | Worst | Card err | Consensus | Attempted B | Byte dev. | B3 |
|:--|--:|--:|--:|--:|--:|--:|--:|
| `CCC` | 14.876684 | 43.153308 | 0.458333 | 20.964275 | 6033000 | +0.000% | 1 |
| `LLL` | 14.101400 | 43.271086 | 0.402778 | 20.054766 | 6003936 | -0.482% | 1 |

## Boundary

This opened-training audit compares causal three-step safe graph options on one registered behavior-state distribution. It uses future truth only to form offline return labels and does not establish a learned policy, development, held-out M24 or X36 claim.
