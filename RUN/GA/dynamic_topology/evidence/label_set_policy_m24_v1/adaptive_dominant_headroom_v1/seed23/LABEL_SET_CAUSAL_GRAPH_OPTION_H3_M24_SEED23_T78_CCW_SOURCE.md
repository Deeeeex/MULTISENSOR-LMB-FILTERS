# M24 causal H=3 safe graph-option preflight

- Seed / anchor: `23 / 78`
- Audit mode: `adaptive-dominant-headroom`
- Return times: `[78 79 80]`
- Source SHA-256: `e7677e4962c65842a50ea196348d8380b1160b7fe925dacfa478f54c45fc5e96`
- Source behavior: `fixed-counter-clockwise`
- Low-level model SHA-256: `9d6732320e9f5f846e6dbd46f1e7711da372ffbc95c39a31bbe2174788cc6af4`
- Exact message budget per step: `40`
- Restart equivalence passed: `1`
- Best option: `D-T-D-T-D-T`
- Best gain versus CCC: `32.815%`

- Registered projection failures: `0`

| Option | Status | Mean E-OSPA | Worst | Card err | Consensus | Attempted B | Byte dev. | B3 |
|:--|:--|--:|--:|--:|--:|--:|--:|--:|
| `CCC` | completed | 7.341908 | 30.494389 | 0.083333 | 10.138767 | 6055968 | +0.000% | 1 |
| `LLL` | completed | 5.901460 | 13.736025 | 0.027778 | 7.549215 | 6055968 | +0.000% | 1 |
| `D-R2-D-R2-D-R2` | completed | 5.716205 | 18.455264 | 0.027778 | 7.165556 | 5836368 | -3.626% | 1 |
| `D-R3-D-R3-D-R3` | completed | 6.331841 | 13.891462 | 0.055556 | 8.441109 | 6011952 | -0.727% | 1 |
| `D-R4-D-R4-D-R4` | completed | 5.729218 | 13.631085 | 0.027778 | 7.153076 | 5826144 | -3.795% | 1 |
| `D-R5-D-R5-D-R5` | completed | 5.454994 | 7.284747 | 0.000000 | 6.349055 | 6004896 | -0.843% | 1 |
| `D-R6-D-R6-D-R6` | completed | 6.767319 | 13.635304 | 0.055556 | 8.846132 | 6067392 | +0.189% | 1 |
| `D-SQ-D-SQ-D-SQ` | completed | 5.453972 | 17.379878 | 0.027778 | 6.918534 | 6061992 | +0.099% | 1 |
| `D-T-D-T-D-T` | completed | 4.932660 | 5.914212 | 0.000000 | 6.043603 | 6016824 | -0.646% | 1 |
| `D-C-D-C-D-C` | completed | 5.229346 | 13.589142 | 0.013889 | 6.700089 | 5976000 | -1.320% | 1 |
| `D-B-D-B-D-B` | completed | 5.404500 | 13.146922 | 0.013889 | 7.055460 | 6021528 | -0.569% | 1 |
| `D-U-D-U-D-U` | completed | 5.239797 | 13.058234 | 0.013889 | 6.591371 | 6046560 | -0.155% | 1 |

## Boundary

This opened-training audit compares causal three-step safe graph options on one registered behavior-state distribution. It uses future truth only to form offline return labels and does not establish a learned policy, development, held-out M24 or X36 claim.
