# M24 truth-free causal H=3 option preflight

- Seed / anchor: `19 / 78`
- Return times: `[78 79 80]`
- Source SHA-256: `d1d4a88021ceea4d547668dbfa1c5f31db2d82e245d553c1c5e3d64e2ffa02b9`
- Best admissible option: `reference-only-safe-switch` (`[24 24 24]`)
- Best admissible gain: `+0.000%`
- Completed / unique trajectories: `6 / 6`

| Option | Codes | Mean E-OSPA | Mean gain | Worst gain | Consensus gain | Byte dev. | Later policy repair | Admissible |
|:--|:--|--:|--:|--:|--:|--:|--:|--:|
| reference-only-safe-switch | `[24 24 24]` | 21.026617 | +0.000% | +0.000% | +0.000% | 0.000% | 0 | 1 |
| deterministic-burst-reference-preferred-switch | `[22 22 22]` | 20.924766 | +0.484% | +3.367% | -0.251% | 1.185% | 0 | 0 |
| link-advantage-reference-preferred-switch | `[61 61 61]` | 21.841197 | -3.874% | +0.000% | -3.426% | 0.485% | 0 | 0 |
| posterior-gain-reference-preferred-switch | `[63 63 63]` | unavailable | unavailable | unavailable | unavailable | unavailable | unavailable | 0 |
| compatibility-reference-preferred-switch | `[67 67 67]` | unavailable | unavailable | unavailable | unavailable | unavailable | unavailable | 0 |
| history-continuity-reference-preferred-switch | `[77 77 77]` | 23.621055 | -12.339% | -12.019% | -10.408% | 1.411% | 0 | 0 |
| posterior-analytic-reference-preferred-switch | `[80 80 80]` | 25.206341 | -19.878% | -6.277% | -14.974% | 2.589% | 0 | 0 |
| diverse-analytic-reference-preferred-switch | `[83 83 83]` | 25.189957 | -19.800% | -1.412% | -16.969% | 2.130% | 0 | 0 |

## Boundary

This opened-training preflight tests a registered causal reference-preferred switching family on four already opened H=3 M24 states. At each later step, candidate and reference are recomputed from the same live posterior and selected history; reference is chosen when its strict nominal projection is repair-free, otherwise the repair-free candidate is chosen. The switch is non-absorbing and may return to the candidate later. It is an experimental safety screen, not a proof of recursive feasibility or a deployable fallback. The first action remains an offline best-of-eight diagnostic, so this is not a learned selector or evidence on formation-FoV M24, X36, or X48. Central coordinator metadata bytes are not charged.
