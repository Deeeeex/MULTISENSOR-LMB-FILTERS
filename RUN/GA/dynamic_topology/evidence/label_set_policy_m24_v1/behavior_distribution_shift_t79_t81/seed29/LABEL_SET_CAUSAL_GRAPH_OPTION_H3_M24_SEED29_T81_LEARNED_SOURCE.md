# M24 causal H=3 safe graph-option preflight

- Seed / anchor: `29 / 81`
- Audit mode: `behavior-distribution-shift`
- Return times: `[81 82 83]`
- Source SHA-256: `e590955866b78a0f9de07e6262d588095187cb6e124cb960364dae38fd4de855`
- Source behavior: `label-set-message-passing-safe-fixed-e05-a70`
- Low-level model SHA-256: `9d6732320e9f5f846e6dbd46f1e7711da372ffbc95c39a31bbe2174788cc6af4`
- Exact message budget per step: `40`
- Restart equivalence passed: `1`
- Best option: `LLL`
- Best gain versus CCC: `2.065%`

| Option | Mean E-OSPA | Worst | Card err | Consensus | Attempted B | Byte dev. | B3 |
|:--|--:|--:|--:|--:|--:|--:|--:|
| `CCC` | 18.956201 | 39.288477 | 0.597222 | 22.879506 | 6139296 | +0.000% | 1 |
| `LLL` | 18.564805 | 46.187506 | 0.583333 | 23.058815 | 6139296 | +0.000% | 1 |

## Boundary

This opened-training audit compares causal three-step safe graph options on one registered behavior-state distribution. It uses future truth only to form offline return labels and does not establish a learned policy, development, held-out M24 or X36 claim.
