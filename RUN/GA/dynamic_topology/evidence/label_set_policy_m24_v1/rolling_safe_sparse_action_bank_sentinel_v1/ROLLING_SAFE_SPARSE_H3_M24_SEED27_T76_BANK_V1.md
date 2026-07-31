# M24 truth-free sparse action-bank H=3 preflight

- Seed / anchor: `27 / 76`
- Return times: `[76 77 78]`
- Source SHA-256: `7debd9a7907840e4c56e0d320469c6cd7094a9a30db7e9a410f0d7e6e7a501cb`
- Reference action: `24`
- Best action: `63`
- Best gain versus reference: `4.521%`

- Best admissible action: `83`
- Best admissible gain versus reference: `2.019%`
- Completed / unique graphs: `8 / 8`

| Code | Mean E-OSPA | Mean gain | Worst gain | Consensus gain | Attempted B | Byte dev. | Admissible |
|--:|--:|--:|--:|--:|--:|--:|--:|
| 24 | 16.704123 | +0.000% | +0.000% | +0.000% | 3562680 | 0.000% | 1 |
| 22 | 16.714564 | -0.063% | +26.431% | +4.875% | 3500760 | 1.738% | 1 |
| 61 | 17.001403 | -1.780% | +0.000% | -1.201% | 3565368 | 0.075% | 0 |
| 63 | 15.948988 | +4.521% | +0.000% | -0.027% | 3555912 | 0.190% | 0 |
| 67 | 16.673296 | +0.185% | +0.000% | +0.078% | 3563520 | 0.024% | 1 |
| 77 | 17.804094 | -6.585% | -0.044% | -6.353% | 3518400 | 1.243% | 0 |
| 80 | 16.768485 | -0.385% | +7.671% | -2.729% | 3544152 | 0.520% | 0 |
| 83 | 16.366789 | +2.019% | +0.000% | +0.195% | 3564360 | 0.047% | 1 |

## Boundary

This opened-training preflight tests paired H=3 headroom inside a truth-free 24-message sparse action family. Its predecision state was produced by the earlier 40-message dense CCW behavior trajectory, so it is not a same-budget sparse-versus-dense comparison. It is not a learned selector, does not use the new formation-FoV scenes, and supports no development, held-out M24, X36, or scale-generalization claim.
