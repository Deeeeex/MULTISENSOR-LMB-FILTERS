# V119 finding: a second local provenance path amplifies F6 loss

## Result

V119 moves one existing `0.05` residual token to F6 for exactly page 5 while
retaining the original F5-to-F6 input.  Each dual-path arm preserves every
sender's message count, the 60-message network budget, row stochasticity,
physicality and rolling B3.  Its paired donor-only arm removes the same token
without adding the F6 input.  No dual-path arm passes.

| Source | Route | Compatibility | Donor-only | Dual-path | Dual vs donor | Gain vs CW | Min. formation | F6 peers |
|:--|:--|--:|--:|--:|--:|--:|--:|--:|
| F1 | `3→33` | 0.473355 | 78.480842 | 78.501408 | -0.026% | +4.037% | -1.647% | -5.217% |
| F2 | `9→34` | 0.099777 | 78.479771 | 78.642521 | -0.207% | +3.864% | -2.697% | -5.208% |
| F3 | `15→34` | 0.073400 | 78.479616 | 78.608275 | -0.164% | +3.906% | -2.445% | -5.209% |
| F4 | `21→34` | 0.190296 | 78.479654 | 78.603258 | -0.157% | +3.912% | -2.548% | -5.215% |

All four donor-only outcomes are within `0.001153` E-OSPA of V113
(`78.479689`).  The deletion ablation is therefore effectively neutral.  The
dual-path regression is attributable to adding the new F6 posterior input,
not to temporarily removing the registered donor edge.

## Interpretation

Current KLA compatibility ranks F1 as the least harmful new source and the two
low-compatibility sources among the most harmful.  It therefore has some
screening value, but even its best-ranked action is negative relative to
donor-only and V113.  Compatibility cannot turn this action family into a
useful controller.

There is also no hidden F6 repair behind a small network-mean loss.  Every new
path makes its target receiver about 9.2--9.4% worse at the terminal page, and
the other F6 receivers about 5.2% worse.  Consensus, worst-sensor performance,
bytes and rolling connectivity remain acceptable, so the binding failure is
again the recursive tracking effect inside F6.

V116--V119 now close all local controls around the delayed boundary: label
content, F5 sender, F6 receiver and an additional one-page formation source.
A GNN should not rank these actions.  The next action must alter the complete
formation-level influence direction over several pages.

## Next bounded screen

V120 will compare three abstention carriers under the same 60-message budget:

1. fixed counter-clockwise for all eight pages;
2. clockwise for pages 1--4, then counter-clockwise before the F6 loss;
3. the reciprocal counter-clockwise-to-clockwise switch.

The fixed V113 clockwise-abstention arm remains the primary baseline.  Fixed
counter-clockwise separates orientation from switching, and the reciprocal
sequence separates a genuine time-aligned intervention from generic topology
churn.  If the clockwise-to-counter-clockwise switch does not beat both fixed
directions with nonnegative formation tails, the current whole-carrier
orientation family is closed.

V119 is privileged opened-development evidence at X36 seed 211, t=72, H=8.
It is not deployable, validation, or generalization evidence.
