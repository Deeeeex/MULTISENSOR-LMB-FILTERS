# M24 causal H=3 safe graph-option preflight

- Seed / anchor: `23 / 81`
- Audit mode: `behavior-distribution-shift`
- Return times: `[81 82 83]`
- Source SHA-256: `e7677e4962c65842a50ea196348d8380b1160b7fe925dacfa478f54c45fc5e96`
- Source behavior: `fixed-counter-clockwise`
- Low-level model SHA-256: `9d6732320e9f5f846e6dbd46f1e7711da372ffbc95c39a31bbe2174788cc6af4`
- Exact message budget per step: `40`
- Restart equivalence passed: `1`
- Best option: `LLL`
- Best gain versus CCC: `13.763%`

| Option | Mean E-OSPA | Worst | Card err | Consensus | Attempted B | Byte dev. | B3 |
|:--|--:|--:|--:|--:|--:|--:|--:|
| `CCC` | 8.734304 | 34.808719 | 0.125000 | 12.290827 | 5942736 | +0.000% | 1 |
| `LLL` | 7.532212 | 14.822117 | 0.069444 | 10.172593 | 5937192 | -0.093% | 1 |

## Boundary

This opened-training audit compares causal three-step safe graph options on one registered behavior-state distribution. It uses future truth only to form offline return labels and does not establish a learned policy, development, held-out M24 or X36 claim.
