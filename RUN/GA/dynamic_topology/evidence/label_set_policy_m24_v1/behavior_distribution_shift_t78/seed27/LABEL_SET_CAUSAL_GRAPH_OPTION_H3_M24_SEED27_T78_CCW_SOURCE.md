# M24 causal H=3 safe graph-option preflight

- Seed / anchor: `27 / 78`
- Audit mode: `behavior-distribution-shift`
- Return times: `[78 79 80]`
- Source SHA-256: `4fb7f2b26d5cf47b4099becbc9219e66cf29d6506b661eca9e1af664907f3041`
- Source behavior: `fixed-counter-clockwise`
- Low-level model SHA-256: `9d6732320e9f5f846e6dbd46f1e7711da372ffbc95c39a31bbe2174788cc6af4`
- Exact message budget per step: `40`
- Restart equivalence passed: `1`
- Best option: `CCC`
- Best gain versus CCC: `0.000%`

| Option | Mean E-OSPA | Worst | Card err | Consensus | Attempted B | Byte dev. | B3 |
|:--|--:|--:|--:|--:|--:|--:|--:|
| `CCC` | 13.279994 | 26.425167 | 0.250000 | 17.501333 | 6033984 | +0.000% | 1 |
| `LLL` | 13.483250 | 31.153680 | 0.263889 | 17.768663 | 6040056 | +0.101% | 1 |

## Boundary

This opened-training audit compares causal three-step safe graph options on one registered behavior-state distribution. It uses future truth only to form offline return labels and does not establish a learned policy, development, held-out M24 or X36 claim.
