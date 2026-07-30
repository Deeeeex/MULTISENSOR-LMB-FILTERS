# M24 causal H=3 safe graph-option preflight

- Seed / anchor: `11 / 78`
- Audit mode: `diverse-proposal-headroom`
- Return times: `[78 79 80]`
- Source SHA-256: `82404ba0ae5b65b9d4ddad6c1b7d91191630d6f0e98eae4a55b42875fd1665c2`
- Source behavior: `label-set-message-passing-safe-fixed-e05-a70`
- Low-level model SHA-256: `9d6732320e9f5f846e6dbd46f1e7711da372ffbc95c39a31bbe2174788cc6af4`
- Exact message budget per step: `40`
- Restart equivalence passed: `1`
- Best option: `CCC`
- Best gain versus CCC: `0.000%`

| Option | Mean E-OSPA | Worst | Card err | Consensus | Attempted B | Byte dev. | B3 |
|:--|--:|--:|--:|--:|--:|--:|--:|
| `CCC` | 15.371985 | 52.446663 | 0.430556 | 21.153136 | 6032640 | +0.000% | 1 |
| `LLL` | 17.394177 | 44.527197 | 0.555556 | 23.579707 | 6090936 | +0.966% | 1 |
| `G2-G2-G2` | 17.043053 | 47.676071 | 0.527778 | 22.641062 | 6099024 | +1.100% | 1 |
| `G3-G3-G3` | 17.603392 | 38.816692 | 0.513889 | 22.292130 | 6096312 | +1.055% | 1 |
| `G4-G4-G4` | 17.874933 | 44.513257 | 0.569444 | 23.766675 | 6085248 | +0.872% | 1 |
| `G5-G5-G5` | 17.061791 | 42.520672 | 0.500000 | 22.365858 | 6099696 | +1.112% | 1 |
| `G6-G6-G6` | 16.516952 | 45.231018 | 0.472222 | 21.765549 | 6101040 | +1.134% | 1 |

## Boundary

This opened-training audit compares causal three-step safe graph options on one registered behavior-state distribution. It uses future truth only to form offline return labels and does not establish a learned policy, development, held-out M24 or X36 claim.
