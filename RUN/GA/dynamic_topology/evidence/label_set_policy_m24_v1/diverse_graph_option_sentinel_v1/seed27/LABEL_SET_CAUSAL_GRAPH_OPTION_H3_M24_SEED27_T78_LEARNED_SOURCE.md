# M24 causal H=3 safe graph-option preflight

- Seed / anchor: `27 / 78`
- Audit mode: `diverse-proposal-headroom`
- Return times: `[78 79 80]`
- Source SHA-256: `3c7ef8d12001346e5acdf254558e2aaa1df488c9e360b5b9214877238643b94b`
- Source behavior: `label-set-message-passing-safe-fixed-e05-a70`
- Low-level model SHA-256: `9d6732320e9f5f846e6dbd46f1e7711da372ffbc95c39a31bbe2174788cc6af4`
- Exact message budget per step: `40`
- Restart equivalence passed: `1`
- Best option: `G4-G4-G4`
- Best gain versus CCC: `5.238%`

| Option | Mean E-OSPA | Worst | Card err | Consensus | Attempted B | Byte dev. | B3 |
|:--|--:|--:|--:|--:|--:|--:|--:|
| `CCC` | 17.483976 | 32.310833 | 0.458333 | 20.086105 | 6031560 | +0.000% | 1 |
| `LLL` | 16.894561 | 30.932400 | 0.416667 | 20.109982 | 6044496 | +0.214% | 1 |
| `G2-G2-G2` | 17.007279 | 30.980410 | 0.430556 | 20.436711 | 6042960 | +0.189% | 1 |
| `G3-G3-G3` | 16.868032 | 30.940514 | 0.416667 | 20.093679 | 6042288 | +0.178% | 1 |
| `G4-G4-G4` | 16.568242 | 30.889321 | 0.388889 | 20.110766 | 6042792 | +0.186% | 1 |
| `G5-G5-G5` | 17.077463 | 30.948513 | 0.430556 | 20.025985 | 6037920 | +0.105% | 1 |
| `G6-G6-G6` | 16.898134 | 30.932763 | 0.416667 | 20.102275 | 6044160 | +0.209% | 1 |

## Boundary

This opened-training audit compares causal three-step safe graph options on one registered behavior-state distribution. It uses future truth only to form offline return labels and does not establish a learned policy, development, held-out M24 or X36 claim.
