# V65 cross-scale development result

V65 uses one network-wide existence denominator, a fixed 1% event budget, an
80% rescue-coverage constraint, and the same useful-information guard on M24
and X36.  Four activated radial states now have direct or schedule-identical
paired outcomes, and every row passes the complete strict gate.

| Scale | State | Network risk | Selected formations | Mean tracking | Worst sensor | Window consensus | Attempted bytes |
|:--|:--|--:|:--|--:|--:|--:|--:|
| M24 | t=104 | `2.515%` | `[1,3]` | `+6.405%` | `+0.000%` | `+13.238%` | `+2.835%` |
| M24 | t=124 | `2.099%` | `[2,3]` | `+8.446%` | `+30.177%` | `+19.149%` | `+2.653%` |
| X36 | t=72 | `1.512%` | `[2,3,4,5]` | `+5.847%` | `+27.843%` | `+15.719%` | `+3.623%` |
| X36 | t=100 | `2.267%` | `[2,4,5,6]` | `+9.329%` | `+11.723%` | `+21.467%` | `+4.533%` |

The fixed event gate also falls back on X36 t=128 (`0.428%` risk), M24 radial
t=60 and t=72 (`0.621%` and `0.828%`), and all six opened M24 convoy weak
states (`0.051%--0.455%`).  Those fallback decisions were made without opening
new tracking outcomes.

This evidence supports the mechanism and its cross-scale parameterization,
not generalization: all positive rows are radial seed-211 development states.
The next decisive test is to freeze event discovery and paired evaluation on
convoy, relay, merge-split, and curved-corridor scene families without changing
the V65 selector.
