# V121: exact-budget mixed formation carrier

V120 shows that reversing the complete ring after four pages slightly repairs
F6 but creates a new F1 loss.  The next action must break this global coupling,
not scan more switch times.

A naive per-edge choice between clockwise and counter-clockwise neighbors is
structurally invalid: on a ring, mixed local directions generally form short
cycles or branches, while a residual carrier with one source and one receiver
per formation must be a single directed Hamiltonian cycle.  V121 therefore
changes the formation order of that cycle while retaining every sensor-level
gateway cut from V113.

The relative clockwise order of F1--F5 stays fixed.  F6 is removed from its
original position after F5 and inserted after F1, F2, F3 or F4:

| Candidate | Formation order | Changed causal role |
|:--|:--|:--|
| after F1 | `F1→F6→F2→F3→F4→F5→F1` | F1 feeds F6; F5 feeds F1 |
| after F2 | `F1→F2→F6→F3→F4→F5→F1` | F2 feeds F6; F5 feeds F1 |
| after F3 | `F1→F2→F3→F6→F4→F5→F1` | F3 feeds F6; F5 feeds F1 |
| after F4 | `F1→F2→F3→F4→F6→F5→F1` | F4 feeds F6; F5 feeds F1 |

This is the complete bounded family that relocates F6 without changing the
relative order of F1--F5.  Every arm runs for all eight pages with the V113
F2--F5 abstention schedule.  Before tracking it must preserve physicality at
every page, one residual sensor cycle, one formation cycle, 60 messages, row
stochasticity, the fusion-weight multiset and rolling sensor/formation B3.

An arm passes only if it improves mean E-OSPA by at least 5% over CW full,
beats both V113 fixed-CW abstention and V114 early F6 shield, keeps every
mature page and formation nonnegative, does not degrade the F6 non-gateway
terminal tail, worst sensor or consensus, and does not increase attempted
bytes.  A pass authorizes segment-level formation-order value learning.  If
all four fail, arbitrary local edge ranking is not justified; the carrier
action must then change its one-cycle/permutation representation.

V121 is opened X36 seed-211 t=72 H=8 development evidence only.
