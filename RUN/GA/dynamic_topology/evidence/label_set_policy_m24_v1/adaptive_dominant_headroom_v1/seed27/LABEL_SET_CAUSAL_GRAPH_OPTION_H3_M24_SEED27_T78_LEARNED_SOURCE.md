# M24 causal H=3 safe graph-option preflight

- Seed / anchor: `27 / 78`
- Audit mode: `adaptive-dominant-headroom`
- Return times: `[78 79 80]`
- Source SHA-256: `3c7ef8d12001346e5acdf254558e2aaa1df488c9e360b5b9214877238643b94b`
- Source behavior: `label-set-message-passing-safe-fixed-e05-a70`
- Low-level model SHA-256: `9d6732320e9f5f846e6dbd46f1e7711da372ffbc95c39a31bbe2174788cc6af4`
- Exact message budget per step: `40`
- Restart equivalence passed: `1`
- Best option: `D-U-D-U-D-U`
- Best gain versus CCC: `19.484%`

- Registered projection failures: `0`

| Option | Status | Mean E-OSPA | Worst | Card err | Consensus | Attempted B | Byte dev. | B3 |
|:--|:--|--:|--:|--:|--:|--:|--:|--:|
| `CCC` | completed | 17.483976 | 32.310833 | 0.458333 | 20.086105 | 6031560 | +0.000% | 1 |
| `LLL` | completed | 16.894561 | 30.932400 | 0.416667 | 20.109982 | 6044496 | +0.214% | 1 |
| `D-R2-D-R2-D-R2` | completed | 15.664076 | 37.876076 | 0.361111 | 19.719685 | 6054168 | +0.375% | 1 |
| `D-R3-D-R3-D-R3` | completed | 15.882952 | 30.749490 | 0.375000 | 19.112487 | 5620968 | -6.807% | 1 |
| `D-R4-D-R4-D-R4` | completed | 16.567846 | 30.700556 | 0.444444 | 18.987260 | 5883984 | -2.447% | 1 |
| `D-R5-D-R5-D-R5` | completed | 15.479552 | 30.702206 | 0.402778 | 18.306421 | 5984112 | -0.787% | 1 |
| `D-R6-D-R6-D-R6` | completed | 14.708819 | 30.740452 | 0.375000 | 17.909904 | 5990136 | -0.687% | 1 |
| `D-SQ-D-SQ-D-SQ` | completed | 15.693747 | 30.714952 | 0.416667 | 18.234546 | 5955936 | -1.254% | 1 |
| `D-T-D-T-D-T` | completed | 14.080226 | 26.553151 | 0.277778 | 18.297173 | 5999856 | -0.526% | 1 |
| `D-C-D-C-D-C` | completed | 17.362557 | 30.721421 | 0.458333 | 20.170074 | 5972880 | -0.973% | 1 |
| `D-B-D-B-D-B` | completed | 15.939628 | 30.690824 | 0.402778 | 19.557828 | 5943360 | -1.462% | 1 |
| `D-U-D-U-D-U` | completed | 14.077348 | 26.491052 | 0.277778 | 18.282062 | 5975640 | -0.927% | 1 |

## Boundary

This opened-training audit compares causal three-step safe graph options on one registered behavior-state distribution. It uses future truth only to form offline return labels and does not establish a learned policy, development, held-out M24 or X36 claim.
