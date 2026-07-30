# M24 causal H=3 safe graph-option preflight

- Seed / anchor: `27 / 80`
- Audit mode: `behavior-distribution-shift`
- Return times: `[80 81 82]`
- Source SHA-256: `4fb7f2b26d5cf47b4099becbc9219e66cf29d6506b661eca9e1af664907f3041`
- Source behavior: `fixed-counter-clockwise`
- Low-level model SHA-256: `9d6732320e9f5f846e6dbd46f1e7711da372ffbc95c39a31bbe2174788cc6af4`
- Exact message budget per step: `40`
- Restart equivalence passed: `1`
- Best option: `LLL`
- Best gain versus CCC: `0.918%`

| Option | Mean E-OSPA | Worst | Card err | Consensus | Attempted B | Byte dev. | B3 |
|:--|--:|--:|--:|--:|--:|--:|--:|
| `CCC` | 15.492271 | 31.330416 | 0.388889 | 18.799537 | 6116160 | +0.000% | 1 |
| `LLL` | 15.350000 | 31.315189 | 0.375000 | 18.315361 | 6116832 | +0.011% | 1 |

## Boundary

This opened-training audit compares causal three-step safe graph options on one registered behavior-state distribution. It uses future truth only to form offline return labels and does not establish a learned policy, development, held-out M24 or X36 claim.
