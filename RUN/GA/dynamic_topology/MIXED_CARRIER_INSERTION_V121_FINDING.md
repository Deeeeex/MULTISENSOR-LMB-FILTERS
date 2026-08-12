# V121 finding: a static carrier permutation only relocates X36 tail risk

## Result

V121 keeps the V113 F2--F5 abstention schedule and exact 60-message budget,
but moves F6 to four alternative positions in the clockwise formation cycle.
All candidates preserve physicality, the fusion-weight multiset, one strongly
connected sensor/formation cycle and rolling B3.  None passes.

| F6 position | Mean E-OSPA | Gain vs CW full | vs fixed CW | Min. formation | Terminal min. formation | F6 peers | Byte saving |
|:--|--:|--:|--:|--:|--:|--:|--:|
| after F1 | 78.782991 | +3.692% | -0.386% | -6.219% | -12.605% | -13.886% | +3.394% |
| after F2 | 79.255185 | +3.115% | -0.988% | -13.151% | -27.851% | -23.731% | +5.826% |
| after F3 | 79.705439 | +2.565% | -1.562% | -13.029% | -19.623% | -21.074% | +4.977% |
| after F4 | 78.812743 | +3.656% | -0.424% | -6.439% | -9.751% | -4.979% | +3.804% |

The least-regret arm is F6 after F4.  It improves five formations relative to
clockwise full, including F1 by `+2.558%`, but degrades F6 by `-6.439%` over
the window and `-9.751%` at the terminal page.  Thus the new formation order
removes the earlier F1/F6 global-direction coupling only by concentrating the
remaining loss in F6; it does not create a uniformly useful route.

## Decision

Static placement of F6 within a single Hamiltonian formation cycle is closed.
The four actions have observable effects, but every one is worse than the
fixed-clockwise abstention control and violates formation-tail safety.  A GNN
that ranks these static permutations would select among actions with no valid
tracking upper bound.

Before defining another learned controller, inspect the page-resolved F6
response.  A time-local complementary carrier is justified only if one of the
V121 orders shows a positive F6 response during the exact pages where V113's
delayed loss emerges.  Otherwise the next representation must leave the
single-cycle family rather than enumerate more temporal schedules.

V121 is privileged opened-development evidence at X36 seed 211, t=72, H=8.
It is not deployable, validation or generalization evidence.
