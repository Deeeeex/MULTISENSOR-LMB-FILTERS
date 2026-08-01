# M24 truth-free causal H=3 option preflight

- Seed / anchor: `23 / 78`
- Return times: `[78 79 80]`
- Source SHA-256: `e7677e4962c65842a50ea196348d8380b1160b7fe925dacfa478f54c45fc5e96`
- Best admissible option: `deterministic-burst-reference-preferred-switch` (`[22 22 22]`)
- Best admissible gain: `+29.259%`
- Completed / unique trajectories: `6 / 6`

| Option | Codes | Mean E-OSPA | Mean gain | Worst gain | Consensus gain | Byte dev. | Later policy repair | Admissible |
|:--|:--|--:|--:|--:|--:|--:|--:|--:|
| reference-only-safe-switch | `[24 24 24]` | 8.427007 | +0.000% | +0.000% | +0.000% | 0.000% | 0 | 1 |
| deterministic-burst-reference-preferred-switch | `[22 22 22]` | 5.961354 | +29.259% | +40.949% | +34.236% | 0.209% | 0 | 1 |
| link-advantage-reference-preferred-switch | `[61 61 61]` | 8.948384 | -6.187% | +0.000% | -7.762% | 0.362% | 0 | 0 |
| posterior-gain-reference-preferred-switch | `[63 63 63]` | unavailable | unavailable | unavailable | unavailable | unavailable | unavailable | 0 |
| compatibility-reference-preferred-switch | `[67 67 67]` | unavailable | unavailable | unavailable | unavailable | unavailable | unavailable | 0 |
| history-continuity-reference-preferred-switch | `[77 77 77]` | 10.339186 | -22.691% | +0.000% | -23.371% | 0.391% | 0 | 0 |
| posterior-analytic-reference-preferred-switch | `[80 80 80]` | 10.127892 | -20.184% | -28.535% | -22.969% | 0.569% | 0 | 0 |
| diverse-analytic-reference-preferred-switch | `[83 83 83]` | 10.967695 | -30.149% | -42.068% | -32.155% | 0.439% | 0 | 0 |

## Boundary

This opened-training preflight tests a registered causal reference-preferred switching family on four already opened H=3 M24 states. At each later step, candidate and reference are recomputed from the same live posterior and selected history; reference is chosen when its strict nominal projection is repair-free, otherwise the repair-free candidate is chosen. The switch is non-absorbing and may return to the candidate later. It is an experimental safety screen, not a proof of recursive feasibility or a deployable fallback. The first action remains an offline best-of-eight diagnostic, so this is not a learned selector or evidence on formation-FoV M24, X36, or X48. Central coordinator metadata bytes are not charged.
