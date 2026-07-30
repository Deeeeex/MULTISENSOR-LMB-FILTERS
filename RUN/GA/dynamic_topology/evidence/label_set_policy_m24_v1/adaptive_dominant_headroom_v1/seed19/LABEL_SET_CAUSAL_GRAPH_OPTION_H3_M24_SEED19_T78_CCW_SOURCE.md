# M24 causal H=3 safe graph-option preflight

- Seed / anchor: `19 / 78`
- Audit mode: `adaptive-dominant-headroom`
- Return times: `[78 79 80]`
- Source SHA-256: `d1d4a88021ceea4d547668dbfa1c5f31db2d82e245d553c1c5e3d64e2ffa02b9`
- Source behavior: `fixed-counter-clockwise`
- Low-level model SHA-256: `9d6732320e9f5f846e6dbd46f1e7711da372ffbc95c39a31bbe2174788cc6af4`
- Exact message budget per step: `40`
- Restart equivalence passed: `1`
- Best option: `D-U-D-U-D-U`
- Best gain versus CCC: `18.908%`

- Registered projection failures: `0`

| Option | Status | Mean E-OSPA | Worst | Card err | Consensus | Attempted B | Byte dev. | B3 |
|:--|:--|--:|--:|--:|--:|--:|--:|--:|
| `CCC` | completed | 19.856173 | 38.877235 | 0.638889 | 23.436646 | 5966592 | +0.000% | 1 |
| `LLL` | completed | 20.285314 | 40.545148 | 0.680556 | 24.067267 | 5961048 | -0.093% | 1 |
| `D-R2-D-R2-D-R2` | completed | 22.153375 | 42.834977 | 0.805556 | 26.419644 | 5958696 | -0.132% | 1 |
| `D-R3-D-R3-D-R3` | completed | 23.084935 | 46.140252 | 0.847222 | 26.601797 | 5999136 | +0.545% | 1 |
| `D-R4-D-R4-D-R4` | completed | 22.409371 | 43.198478 | 0.819444 | 26.219451 | 6025320 | +0.984% | 1 |
| `D-R5-D-R5-D-R5` | completed | 21.386882 | 47.555881 | 0.736111 | 26.134392 | 6015552 | +0.821% | 1 |
| `D-R6-D-R6-D-R6` | completed | 20.078667 | 38.770940 | 0.694444 | 24.357030 | 5989608 | +0.386% | 1 |
| `D-SQ-D-SQ-D-SQ` | completed | 17.979590 | 42.911730 | 0.625000 | 23.898952 | 5976240 | +0.162% | 1 |
| `D-T-D-T-D-T` | completed | 16.968087 | 42.983771 | 0.583333 | 22.971444 | 6010776 | +0.741% | 1 |
| `D-C-D-C-D-C` | completed | 21.142836 | 43.007022 | 0.750000 | 26.180581 | 5969256 | +0.045% | 1 |
| `D-B-D-B-D-B` | completed | 17.362118 | 38.955054 | 0.541667 | 22.691820 | 6006528 | +0.669% | 1 |
| `D-U-D-U-D-U` | completed | 16.101727 | 42.982490 | 0.527778 | 22.304745 | 6020400 | +0.902% | 1 |

## Boundary

This opened-training audit compares causal three-step safe graph options on one registered behavior-state distribution. It uses future truth only to form offline return labels and does not establish a learned policy, development, held-out M24 or X36 claim.
