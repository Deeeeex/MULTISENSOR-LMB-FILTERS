# M24 causal H=3 safe graph-option preflight

- Seed / anchor: `19 / 78`
- Audit mode: `diverse-proposal-headroom`
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
| `G2-G2-G2` | 19.519783 | 38.875792 | 0.611111 | 22.720701 | 5944056 | -0.378% | 1 |
| `G3-G3-G3` | 19.690319 | 38.877968 | 0.638889 | 23.578577 | 5910624 | -0.938% | 1 |
| `G4-G4-G4` | 19.507156 | 38.847616 | 0.625000 | 23.270788 | 5962248 | -0.073% | 1 |
| `G5-G5-G5` | 19.440435 | 38.878001 | 0.611111 | 23.022363 | 5945400 | -0.355% | 1 |
| `G6-G6-G6` | 19.992796 | 38.848775 | 0.666667 | 23.463885 | 5951400 | -0.255% | 1 |

## Boundary

This opened-training audit compares causal three-step safe graph options on one registered behavior-state distribution. It uses future truth only to form offline return labels and does not establish a learned policy, development, held-out M24 or X36 claim.
