# M24 causal H=3 safe graph-option preflight

- Seed / anchor: `27 / 78`
- Audit mode: `diverse-temporal-upper-bound`
- Return times: `[78 79 80]`
- Source SHA-256: `3c7ef8d12001346e5acdf254558e2aaa1df488c9e360b5b9214877238643b94b`
- Source behavior: `label-set-message-passing-safe-fixed-e05-a70`
- Low-level model SHA-256: `9d6732320e9f5f846e6dbd46f1e7711da372ffbc95c39a31bbe2174788cc6af4`
- Exact message budget per step: `40`
- Restart equivalence passed: `1`
- Best option: `G4-C-C`
- Best gain versus CCC: `5.469%`

| Option | Mean E-OSPA | Worst | Card err | Consensus | Attempted B | Byte dev. | B3 |
|:--|--:|--:|--:|--:|--:|--:|--:|
| `CCC` | 17.483976 | 32.310833 | 0.458333 | 20.086105 | 6031560 | +0.000% | 1 |
| `LLL` | 16.894561 | 30.932400 | 0.416667 | 20.109982 | 6044496 | +0.214% | 1 |
| `G4-G4-G4` | 16.568242 | 30.889321 | 0.388889 | 20.110766 | 6042792 | +0.186% | 1 |
| `G4-C-C` | 16.527847 | 30.889321 | 0.388889 | 20.070960 | 6042792 | +0.186% | 1 |
| `G4-G4-C` | 16.566015 | 30.889321 | 0.388889 | 20.111071 | 6042792 | +0.186% | 1 |
| `C-G4-G4` | 17.479986 | 32.144161 | 0.458333 | 20.061694 | 6031560 | +0.000% | 1 |
| `C-C-G4` | 17.489859 | 32.310833 | 0.458333 | 20.089546 | 6031560 | +0.000% | 1 |
| `C-G4-C` | 17.482973 | 32.144161 | 0.458333 | 20.074535 | 6031560 | +0.000% | 1 |

## Boundary

This opened-training audit compares causal three-step safe graph options on one registered behavior-state distribution. It uses future truth only to form offline return labels and does not establish a learned policy, development, held-out M24 or X36 claim.
