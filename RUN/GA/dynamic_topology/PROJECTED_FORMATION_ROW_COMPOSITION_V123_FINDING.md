# V123 finding: independent row values are not compositionally stable

## Result

V123 combines receiver-formation rows selected from opened CW/V113/V121/V122
outcomes, then applies a minimum rolling-B3 fallback on pages five and eight.
Every page preserves 60 messages, physicality, row stochasticity and the
fusion-weight multiset; every three-page sensor and formation union is strongly
connected.  The executable composition does not pass.

| Mean E-OSPA | Gain vs CW full | vs V113 | Mature minimum | Min. formation | F6 | F6 terminal | Byte saving |
|--:|--:|--:|--:|--:|--:|--:|--:|
| 79.284652 | +3.079% | -1.026% | +0.702% | -6.894% | -6.894% | -7.244% | +2.490% |

F1--F5 remain mostly positive, but F6 reverses sharply.  Its F1-sourced row
improves the first two pages by `+3.586%` and `+5.296%`, then produces losses
from `-9.389%` to `-18.884%` on pages three through seven.  A row that is good
inside one previously executed trajectory therefore cannot be assigned an
independent value and safely composed with other rows; recursive KLA state
changes create strong temporal interactions.

## Decision

Formation-row value learning is not yet justified as an additive ranking
problem.  The next bounded correction removes only the identified harmful
F6 row switch and keeps F6 on the clockwise reference row for all pages.  All
other V123 row and payload decisions remain frozen.  If that arm still fails,
the learning target must be sequence-level finite-horizon regret rather than
independent edge or formation-row value.

V123 is privileged opened-development evidence at X36 seed 211, t=72, H=8.
It is not deployable, validation or generalization evidence.
