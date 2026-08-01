# M24 truth-free causal H=3 option preflight

- Seed / anchor: `11 / 78`
- Return times: `[78 79 80]`
- Source SHA-256: `82404ba0ae5b65b9d4ddad6c1b7d91191630d6f0e98eae4a55b42875fd1665c2`
- Best admissible option: `reference-only-safe-switch` (`[24 24 24]`)
- Best admissible gain: `+0.000%`
- Completed / unique trajectories: `7 / 7`

| Option | Codes | Mean E-OSPA | Mean gain | Worst gain | Consensus gain | Byte dev. | Later policy repair | Admissible |
|:--|:--|--:|--:|--:|--:|--:|--:|--:|
| reference-only-safe-switch | `[24 24 24]` | 14.983228 | +0.000% | +0.000% | +0.000% | 0.000% | 0 | 1 |
| deterministic-burst-reference-preferred-switch | `[22 22 22]` | 13.955819 | +6.857% | -19.722% | +6.507% | 1.123% | 0 | 0 |
| link-advantage-reference-preferred-switch | `[61 61 61]` | 15.847324 | -5.767% | +5.425% | -5.416% | 0.635% | 0 | 0 |
| posterior-gain-reference-preferred-switch | `[63 63 63]` | 17.975748 | -19.972% | -8.972% | -19.295% | 0.427% | 0 | 0 |
| compatibility-reference-preferred-switch | `[67 67 67]` | unavailable | unavailable | unavailable | unavailable | unavailable | unavailable | 0 |
| history-continuity-reference-preferred-switch | `[77 77 77]` | 17.085696 | -14.032% | +5.425% | -9.807% | 2.103% | 0 | 0 |
| posterior-analytic-reference-preferred-switch | `[80 80 80]` | 19.688306 | -31.402% | -23.157% | -26.129% | 1.410% | 0 | 0 |
| diverse-analytic-reference-preferred-switch | `[83 83 83]` | 19.200660 | -28.148% | -23.436% | -22.562% | 1.250% | 0 | 0 |

## Boundary

This opened-training preflight tests a registered causal reference-preferred switching family on four already opened H=3 M24 states. At each later step, candidate and reference are recomputed from the same live posterior and selected history; reference is chosen when its strict nominal projection is repair-free, otherwise the repair-free candidate is chosen. The switch is non-absorbing and may return to the candidate later. It is an experimental safety screen, not a proof of recursive feasibility or a deployable fallback. The first action remains an offline best-of-eight diagnostic, so this is not a learned selector or evidence on formation-FoV M24, X36, or X48. Central coordinator metadata bytes are not charged.
