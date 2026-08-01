# M24 truth-free causal H=3 option preflight

- Seed / anchor: `27 / 78`
- Return times: `[78 79 80]`
- Source SHA-256: `3c7ef8d12001346e5acdf254558e2aaa1df488c9e360b5b9214877238643b94b`
- Best admissible option: `history-continuity-reference-preferred-switch` (`[77 77 77]`)
- Best admissible gain: `+6.164%`
- Completed / unique trajectories: `5 / 5`

| Option | Codes | Mean E-OSPA | Mean gain | Worst gain | Consensus gain | Byte dev. | Later policy repair | Admissible |
|:--|:--|--:|--:|--:|--:|--:|--:|--:|
| reference-only-safe-switch | `[24 24 24]` | 16.648442 | +0.000% | +0.000% | +0.000% | 0.000% | 0 | 1 |
| deterministic-burst-reference-preferred-switch | `[22 22 22]` | 17.348092 | -4.202% | -30.397% | -7.035% | 0.210% | 0 | 0 |
| link-advantage-reference-preferred-switch | `[61 61 61]` | unavailable | unavailable | unavailable | unavailable | unavailable | unavailable | 0 |
| posterior-gain-reference-preferred-switch | `[63 63 63]` | unavailable | unavailable | unavailable | unavailable | unavailable | unavailable | 0 |
| compatibility-reference-preferred-switch | `[67 67 67]` | unavailable | unavailable | unavailable | unavailable | unavailable | unavailable | 0 |
| history-continuity-reference-preferred-switch | `[77 77 77]` | 15.622186 | +6.164% | +10.413% | +5.439% | 0.101% | 0 | 1 |
| posterior-analytic-reference-preferred-switch | `[80 80 80]` | 17.429567 | -4.692% | -9.664% | +0.437% | 0.131% | 0 | 0 |
| diverse-analytic-reference-preferred-switch | `[83 83 83]` | 16.243672 | +2.431% | +10.413% | +3.447% | 1.311% | 0 | 1 |

## Boundary

This opened-training preflight tests a registered causal reference-preferred switching family on four already opened H=3 M24 states. At each later step, candidate and reference are recomputed from the same live posterior and selected history; reference is chosen when its strict nominal projection is repair-free, otherwise the repair-free candidate is chosen. The switch is non-absorbing and may return to the candidate later. It is an experimental safety screen, not a proof of recursive feasibility or a deployable fallback. The first action remains an offline best-of-eight diagnostic, so this is not a learned selector or evidence on formation-FoV M24, X36, or X48. Central coordinator metadata bytes are not charged.
