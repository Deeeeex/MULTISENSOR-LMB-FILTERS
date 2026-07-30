# M24 causal H=3 safe graph-option preflight

- Seed / anchor: `11 / 78`
- Audit mode: `adaptive-dominant-headroom`
- Return times: `[78 79 80]`
- Source SHA-256: `82404ba0ae5b65b9d4ddad6c1b7d91191630d6f0e98eae4a55b42875fd1665c2`
- Source behavior: `label-set-message-passing-safe-fixed-e05-a70`
- Low-level model SHA-256: `9d6732320e9f5f846e6dbd46f1e7711da372ffbc95c39a31bbe2174788cc6af4`
- Exact message budget per step: `40`
- Restart equivalence passed: `1`
- Best option: `D-U-D-U-D-U`
- Best gain versus CCC: `15.962%`

- Registered projection failures: `0`

| Option | Status | Mean E-OSPA | Worst | Card err | Consensus | Attempted B | Byte dev. | B3 |
|:--|:--|--:|--:|--:|--:|--:|--:|--:|
| `CCC` | completed | 15.371985 | 52.446663 | 0.430556 | 21.153136 | 6032640 | +0.000% | 1 |
| `LLL` | completed | 17.394177 | 44.527197 | 0.555556 | 23.579707 | 6090936 | +0.966% | 1 |
| `D-R2-D-R2-D-R2` | completed | 16.209521 | 52.246779 | 0.513889 | 22.568394 | 6075504 | +0.711% | 1 |
| `D-R3-D-R3-D-R3` | completed | 16.798688 | 52.148878 | 0.555556 | 22.896104 | 5899080 | -2.214% | 1 |
| `D-R4-D-R4-D-R4` | completed | 16.098300 | 54.845658 | 0.527778 | 22.477621 | 5987976 | -0.740% | 1 |
| `D-R5-D-R5-D-R5` | completed | 16.177137 | 52.181659 | 0.513889 | 21.723170 | 5964432 | -1.131% | 1 |
| `D-R6-D-R6-D-R6` | completed | 14.355415 | 52.194457 | 0.430556 | 20.430510 | 6002928 | -0.493% | 1 |
| `D-SQ-D-SQ-D-SQ` | completed | 13.231049 | 49.077980 | 0.388889 | 18.923621 | 6029976 | -0.044% | 1 |
| `D-T-D-T-D-T` | completed | 13.028754 | 49.074153 | 0.375000 | 18.725985 | 6041400 | +0.145% | 1 |
| `D-C-D-C-D-C` | completed | 13.731077 | 48.284227 | 0.402778 | 19.846468 | 5984592 | -0.796% | 1 |
| `D-B-D-B-D-B` | completed | 14.913963 | 49.122295 | 0.486111 | 21.983289 | 6026472 | -0.102% | 1 |
| `D-U-D-U-D-U` | completed | 12.918320 | 49.074153 | 0.361111 | 18.485525 | 6034680 | +0.034% | 1 |

## Boundary

This opened-training audit compares causal three-step safe graph options on one registered behavior-state distribution. It uses future truth only to form offline return labels and does not establish a learned policy, development, held-out M24 or X36 claim.
