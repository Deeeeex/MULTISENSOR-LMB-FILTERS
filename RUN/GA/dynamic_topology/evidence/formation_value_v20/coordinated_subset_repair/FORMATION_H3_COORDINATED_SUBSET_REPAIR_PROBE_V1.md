# Formation H=3 coordinated-subset repair probe

- Contract / protocol: `formation-h3-coordinated-subset-repair-probe-result-v1 / formation-h3-coordinated-subset-repair-sequence-v1`
- Generation commit: `402b9a98fbee3cf785a53a54c8df650647fe8383`
- Cache protocol / commit: `formation-h3-event-conditioned-sentinel-v1 / c9c6d4dcdc7ad1cb04fb88a22823e99c7fc5bc53`
- Preset / seed / time: `m24-formation-fov / 211 / 72`
- Strict feasible: `1/7`
- Strict oracle / gain: `[1 1 1] / +0.000000%`
- Strong safe sequence found: `0`

| Sequence | Third action | Order | Mean | Min. formation | Worst sensor | Consensus | Attempted bytes | Delivered bytes | Strict |
|:--|:--|--:|--:|--:|--:|--:|--:|--:|:--:|
| `1 1 1` | reference | 0 | +0.000% | +0.000% | +0.000% | +0.000% | +0.000% | +0.000% | 1 |
| `9 13 1` | reference | 0 | +7.911% | +0.000% | -0.001% | -6.069% | -0.844% | -0.883% | 0 |
| `9 13 20` | formations-1-2-3-dynamic-trust-0.30 | 3 | +5.726% | -0.041% | +0.031% | -5.125% | +0.981% | +1.026% | 0 |
| `9 13 21` | formations-1-2-4-dynamic-trust-0.30 | 3 | +8.647% | -0.041% | +0.031% | -3.778% | +1.059% | +1.108% | 0 |
| `9 13 22` | formations-1-3-4-dynamic-trust-0.30 | 3 | +6.407% | -0.041% | -0.001% | -2.837% | +1.633% | +1.707% | 0 |
| `9 13 23` | formations-2-3-4-dynamic-trust-0.30 | 3 | +6.448% | +0.000% | +0.031% | -2.395% | +1.027% | +1.074% | 0 |
| `9 13 24` | formations-1-2-3-4-dynamic-trust-0.30 | 4 | +6.439% | -0.041% | +0.031% | -2.637% | +1.848% | +1.933% | 0 |

## Evidence boundary

The prefix [9,13] is selected after inspecting the v18 outcomes. The third-step bank is fixed to the four three-formation subsets and the all-formation subset at trust 0.30 before execution. This privileged, single-state probe may test coordination order only; it cannot validate a selector or support M24/X36/final-seed claims.
