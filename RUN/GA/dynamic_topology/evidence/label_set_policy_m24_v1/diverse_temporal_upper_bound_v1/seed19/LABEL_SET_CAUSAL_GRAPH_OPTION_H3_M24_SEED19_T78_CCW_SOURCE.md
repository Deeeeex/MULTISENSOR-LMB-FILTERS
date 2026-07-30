# M24 causal H=3 safe graph-option preflight

- Seed / anchor: `19 / 78`
- Audit mode: `diverse-temporal-upper-bound`
- Return times: `[78 79 80]`
- Source SHA-256: `d1d4a88021ceea4d547668dbfa1c5f31db2d82e245d553c1c5e3d64e2ffa02b9`
- Source behavior: `fixed-counter-clockwise`
- Low-level model SHA-256: `9d6732320e9f5f846e6dbd46f1e7711da372ffbc95c39a31bbe2174788cc6af4`
- Exact message budget per step: `40`
- Restart equivalence passed: `1`
- Best option: `G5-G5-G5`
- Best gain versus CCC: `2.094%`

| Option | Mean E-OSPA | Worst | Card err | Consensus | Attempted B | Byte dev. | B3 |
|:--|--:|--:|--:|--:|--:|--:|--:|
| `CCC` | 19.856173 | 38.877235 | 0.638889 | 23.436646 | 5966592 | +0.000% | 1 |
| `LLL` | 20.285314 | 40.545148 | 0.680556 | 24.067267 | 5961048 | -0.093% | 1 |
| `G5-G5-G5` | 19.440435 | 38.878001 | 0.611111 | 23.022363 | 5945400 | -0.355% | 1 |
| `G5-C-C` | 19.772924 | 38.878001 | 0.625000 | 23.196447 | 5944560 | -0.369% | 1 |
| `G5-G5-C` | 19.601811 | 38.878001 | 0.625000 | 23.453348 | 5945400 | -0.355% | 1 |
| `C-G5-G5` | 19.504182 | 38.876664 | 0.625000 | 23.233910 | 5966424 | -0.003% | 1 |
| `C-C-G5` | 19.573024 | 38.877235 | 0.625000 | 22.854817 | 5966592 | +0.000% | 1 |
| `C-G5-C` | 19.669693 | 38.876664 | 0.638889 | 23.677720 | 5966424 | -0.003% | 1 |

## Boundary

This opened-training audit compares causal three-step safe graph options on one registered behavior-state distribution. It uses future truth only to form offline return labels and does not establish a learned policy, development, held-out M24 or X36 claim.
