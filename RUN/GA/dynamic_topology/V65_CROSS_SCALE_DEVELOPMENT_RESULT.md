# V65 cross-scale development result

V65 uses one network-wide existence denominator, a fixed 1% event budget, an
80% rescue-coverage constraint, and the same useful-information guard on M24
and X36.  Four activated radial states have direct or schedule-identical
paired outcomes and pass the complete strong gate.  The first freshly frozen
non-radial event is safe and positive but does not pass the 5% gain gate.

| Scale | State | Network risk | Selected formations | Mean tracking | Worst sensor | Window consensus | Attempted bytes |
|:--|:--|--:|:--|--:|--:|--:|--:|
| M24 | t=104 | `2.515%` | `[1,3]` | `+6.405%` | `+0.000%` | `+13.238%` | `+2.835%` |
| M24 | t=124 | `2.099%` | `[2,3]` | `+8.446%` | `+30.177%` | `+19.149%` | `+2.653%` |
| X36 | t=72 | `1.512%` | `[2,3,4,5]` | `+5.847%` | `+27.843%` | `+15.719%` | `+3.623%` |
| X36 | t=100 | `2.267%` | `[2,4,5,6]` | `+9.329%` | `+11.723%` | `+21.467%` | `+4.533%` |
| M24 convoy | t=80 | `1.030%` | `[1]` | `+0.315%` | `+0.000%` | `+0.076%` | `+3.037%` |

The fixed event gate also falls back on X36 t=128 (`0.428%` risk), M24 radial
t=60 and t=72 (`0.621%` and `0.828%`), and all six opened M24 convoy weak
states (`0.051%--0.455%`).  Those fallback decisions were made without opening
new tracking outcomes.

This evidence supports the mechanism and its cross-scale parameterization,
but rejects scene generalization of the V65 event score.  The convoy action
changes only one sensor materially and its benefit does not spread over the
three-step horizon.  Additive existence rescue therefore measures the amount
of dilution removed but not the breadth or causal reach of the affected
receiver decisions.  The next method must add current-only decision-breadth
and time-expanded influence before freezing a fresh relay, merge-split, or
curved-corridor tracking case; the 1% V65 threshold is not retuned after this
outcome.
