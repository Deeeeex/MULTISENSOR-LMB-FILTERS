# Formation H=3 pair-repair sequence probe

- Contract / protocol: `formation-h3-pair-repair-sequence-probe-result-v1 / formation-h3-pair-repair-sequence-v1`
- Generation commit: `ec9541cf580c616456c44d1dea5d28d0a4d8b631`
- Cache protocol / commit: `formation-h3-event-conditioned-sentinel-v1 / c9c6d4dcdc7ad1cb04fb88a22823e99c7fc5bc53`
- Preset / seed / time: `m24-formation-fov / 211 / 72`
- Strict feasible: `1/8`
- Strict oracle / gain: `[1 1 1] / +0.000000%`
- Strong safe sequence found: `0`

| Sequence | Third action | Mean | Min. formation | Worst sensor | Consensus | Attempted bytes | Delivered bytes | Strict |
|:--|:--|--:|--:|--:|--:|--:|--:|:--:|
| `1 1 1` | reference | +0.000% | +0.000% | +0.000% | +0.000% | +0.000% | +0.000% | 1 |
| `9 13 1` | reference | +7.911% | +0.000% | -0.001% | -6.069% | -0.844% | -0.883% | 0 |
| `9 13 14` | formations-1-2-dynamic-trust-0.30 | +7.934% | -0.041% | +0.031% | -6.138% | +0.192% | +0.201% | 0 |
| `9 13 15` | formations-1-3-dynamic-trust-0.30 | +5.694% | -0.041% | -0.001% | -5.262% | +0.766% | +0.801% | 0 |
| `9 13 16` | formations-1-4-dynamic-trust-0.30 | +8.615% | -0.041% | -0.001% | -3.968% | +0.844% | +0.882% | 0 |
| `9 13 17` | formations-2-3-dynamic-trust-0.30 | +5.735% | +0.000% | +0.031% | -4.914% | +0.160% | +0.167% | 0 |
| `9 13 18` | formations-2-4-dynamic-trust-0.30 | +8.656% | +0.000% | +0.031% | -3.554% | +0.238% | +0.249% | 0 |
| `9 13 19` | formations-3-4-dynamic-trust-0.30 | +6.416% | +0.000% | -0.001% | -2.589% | +0.812% | +0.849% | 0 |

## Evidence boundary

The prefix [9,13] is selected after inspecting the v18 outcomes. Only the six pre-existing conservative pair actions are expanded at step three. This privileged, single-state probe may test whether pair repair closes the known consensus debt; it cannot validate a selector or support M24/X36/final-seed claims.
