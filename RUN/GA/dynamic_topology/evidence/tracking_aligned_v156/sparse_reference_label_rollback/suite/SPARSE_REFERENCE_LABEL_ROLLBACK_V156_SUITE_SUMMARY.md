# V156 sparse reference-label capacity summary

All arms reuse the paired static X36 seed-211 t=72 H=8 baseline. They are privileged representation oracles, not deployable policies.

| Maximum labels K | Mean gain | Minimum formation-time gain | Adjusted byte saving | Gate |
|--:|--:|--:|--:|:--:|
| 1 | +6.024% | -6.017% | +5.845% | 0 |
| 2 | +5.738% | -1.353% | +5.856% | 0 |
| 4 | +5.478% | -1.085% | +5.885% | 0 |
| 8 | +5.871% | -0.091% | +4.752% | 0 |

No uniform top-K arm passes. K=1 gives the strongest mean gain, while larger K progressively repairs the worst local tail but can over-correct other node-time cells. The fixed-capacity, reference-discrepancy ranking is therefore closed. The next bounded question is whether a label-value oracle that accepts only positive marginal edits can satisfy both objectives with variable capacity.
