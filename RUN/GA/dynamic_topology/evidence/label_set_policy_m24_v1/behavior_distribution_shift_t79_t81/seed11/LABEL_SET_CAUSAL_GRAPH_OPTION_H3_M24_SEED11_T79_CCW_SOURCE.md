# M24 causal H=3 safe graph-option preflight

- Seed / anchor: `11 / 79`
- Audit mode: `behavior-distribution-shift`
- Return times: `[79 80 81]`
- Source SHA-256: `d3f3cd95ecf624d3705075c64e907d44761966b3adbae010143533eb4134ee44`
- Source behavior: `fixed-counter-clockwise`
- Low-level model SHA-256: `9d6732320e9f5f846e6dbd46f1e7711da372ffbc95c39a31bbe2174788cc6af4`
- Exact message budget per step: `40`
- Restart equivalence passed: `1`
- Best option: `CCC`
- Best gain versus CCC: `0.000%`

| Option | Mean E-OSPA | Worst | Card err | Consensus | Attempted B | Byte dev. | B3 |
|:--|--:|--:|--:|--:|--:|--:|--:|
| `CCC` | 18.973532 | 49.681199 | 0.583333 | 23.075424 | 5944488 | +0.000% | 1 |
| `LLL` | 19.460802 | 45.844214 | 0.625000 | 24.264681 | 5967624 | +0.389% | 1 |

## Boundary

This opened-training audit compares causal three-step safe graph options on one registered behavior-state distribution. It uses future truth only to form offline return labels and does not establish a learned policy, development, held-out M24 or X36 claim.
