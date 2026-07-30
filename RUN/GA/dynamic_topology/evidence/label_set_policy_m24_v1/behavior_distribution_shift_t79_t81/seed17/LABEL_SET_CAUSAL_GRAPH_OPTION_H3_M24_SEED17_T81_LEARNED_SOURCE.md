# M24 causal H=3 safe graph-option preflight

- Seed / anchor: `17 / 81`
- Audit mode: `behavior-distribution-shift`
- Return times: `[81 82 83]`
- Source SHA-256: `3bbf39866a1468f8a6b0918188110939840c6c368d0ebc57efd57943aa32ec10`
- Source behavior: `label-set-message-passing-safe-fixed-e05-a70`
- Low-level model SHA-256: `9d6732320e9f5f846e6dbd46f1e7711da372ffbc95c39a31bbe2174788cc6af4`
- Exact message budget per step: `40`
- Restart equivalence passed: `1`
- Best option: `LLL`
- Best gain versus CCC: `3.021%`

| Option | Mean E-OSPA | Worst | Card err | Consensus | Attempted B | Byte dev. | B3 |
|:--|--:|--:|--:|--:|--:|--:|--:|
| `CCC` | 21.609728 | 38.719809 | 0.652778 | 22.137901 | 6017736 | +0.000% | 1 |
| `LLL` | 20.956818 | 35.013005 | 0.625000 | 22.148340 | 6034584 | +0.280% | 1 |

## Boundary

This opened-training audit compares causal three-step safe graph options on one registered behavior-state distribution. It uses future truth only to form offline return labels and does not establish a learned policy, development, held-out M24 or X36 claim.
