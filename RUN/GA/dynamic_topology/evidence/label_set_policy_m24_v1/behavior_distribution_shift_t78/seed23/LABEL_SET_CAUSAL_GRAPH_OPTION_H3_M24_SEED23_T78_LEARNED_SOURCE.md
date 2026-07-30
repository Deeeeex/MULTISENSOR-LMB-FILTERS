# M24 causal H=3 safe graph-option preflight

- Seed / anchor: `23 / 78`
- Audit mode: `behavior-distribution-shift`
- Return times: `[78 79 80]`
- Source SHA-256: `7c7203e151d1eba0e5553edc7933fb4538b7c7d9561a93936487bd2fbf60a3fc`
- Source behavior: `label-set-message-passing-safe-fixed-e05-a70`
- Low-level model SHA-256: `9d6732320e9f5f846e6dbd46f1e7711da372ffbc95c39a31bbe2174788cc6af4`
- Exact message budget per step: `40`
- Restart equivalence passed: `1`
- Best option: `LLL`
- Best gain versus CCC: `17.795%`

| Option | Mean E-OSPA | Worst | Card err | Consensus | Attempted B | Byte dev. | B3 |
|:--|--:|--:|--:|--:|--:|--:|--:|
| `CCC` | 6.331549 | 22.189628 | 0.041667 | 8.312687 | 6055968 | +0.000% | 1 |
| `LLL` | 5.204846 | 5.563320 | 0.000000 | 6.183233 | 6055296 | -0.011% | 1 |

## Boundary

This opened-training audit compares causal three-step safe graph options on one registered behavior-state distribution. It uses future truth only to form offline return labels and does not establish a learned policy, development, held-out M24 or X36 claim.
