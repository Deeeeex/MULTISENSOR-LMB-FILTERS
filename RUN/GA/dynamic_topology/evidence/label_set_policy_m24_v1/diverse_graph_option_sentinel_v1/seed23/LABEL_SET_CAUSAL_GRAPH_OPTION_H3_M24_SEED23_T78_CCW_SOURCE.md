# M24 causal H=3 safe graph-option preflight

- Seed / anchor: `23 / 78`
- Audit mode: `diverse-proposal-headroom`
- Return times: `[78 79 80]`
- Source SHA-256: `e7677e4962c65842a50ea196348d8380b1160b7fe925dacfa478f54c45fc5e96`
- Source behavior: `fixed-counter-clockwise`
- Low-level model SHA-256: `9d6732320e9f5f846e6dbd46f1e7711da372ffbc95c39a31bbe2174788cc6af4`
- Exact message budget per step: `40`
- Restart equivalence passed: `1`
- Best option: `G5-G5-G5`
- Best gain versus CCC: `19.690%`

| Option | Mean E-OSPA | Worst | Card err | Consensus | Attempted B | Byte dev. | B3 |
|:--|--:|--:|--:|--:|--:|--:|--:|
| `CCC` | 7.341908 | 30.494389 | 0.083333 | 10.138767 | 6055968 | +0.000% | 1 |
| `LLL` | 5.901460 | 13.736025 | 0.027778 | 7.549215 | 6055968 | +0.000% | 1 |
| `G2-G2-G2` | 5.903820 | 22.038944 | 0.027778 | 7.560536 | 6055968 | +0.000% | 1 |
| `G3-G3-G3` | 6.255610 | 21.806052 | 0.041667 | 8.187665 | 6055968 | +0.000% | 1 |
| `G4-G4-G4` | 6.265071 | 13.783188 | 0.041667 | 8.243672 | 6055968 | +0.000% | 1 |
| `G5-G5-G5` | 5.896317 | 13.782094 | 0.027778 | 7.545071 | 6055296 | -0.011% | 1 |
| `G6-G6-G6` | 5.915452 | 22.046721 | 0.027778 | 7.571156 | 6056640 | +0.011% | 1 |

## Boundary

This opened-training audit compares causal three-step safe graph options on one registered behavior-state distribution. It uses future truth only to form offline return labels and does not establish a learned policy, development, held-out M24 or X36 claim.
