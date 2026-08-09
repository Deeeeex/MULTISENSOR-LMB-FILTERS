# V63 X36 observable schedule result

V63 tested whether the M24 control-plane/data-plane mechanism survives at
X36 when protected formations are ranked from current measurement-supported
existence rescue.  The first state was the historically weak
`x36-formation-fov / seed 211 / t=72` H=3 window.

| Schedule | Mean tracking | Worst sensor | Minimum formation | Window consensus | Terminal consensus | Attempted bytes | Strict |
|:--|--:|--:|--:|--:|--:|--:|:--:|
| old V37 `[3,5,6] -> 5 -> 2` | +1.794% | +4.969% | -0.370% | +5.393% | +4.810% | +1.031% | no |
| top-1 `[4] x 3` | +1.408% | +0.000% | +0.000% | +3.417% | +3.602% | +0.930% | yes |
| top-2 `[4,2] x 3` | +3.081% | +0.000% | +0.000% | +6.888% | +7.206% | +1.653% | yes |
| top-3 `[4,2,3] x 3` | **+4.703%** | **+14.644%** | +0.000% | **+12.475%** | **+13.255%** | **+2.473%** | yes |
| nested `[4,2,3] -> [4,2] -> 4` | +3.152% | +11.693% | +0.000% | +7.013% | +1.828% | +0.808% | yes |
| fast recovery `[4,2,3] -> [4,2] -> []` | +2.746% | +11.693% | +0.000% | +5.745% | -2.220% | +0.187% | no |

The registered `5%` strong mean-gain gate therefore fails by `0.297`
percentage points.  The result is not rounded into a pass and the remaining
X36 outcomes stay closed.

## First-principles interpretation

The three persistent arms isolate intervention coverage while holding the
carrier graph, payload mode, and duration fixed.  Their gains increase
monotonically from `1.408%` to `3.081%` to `4.703%`.  The problem at X36 is
therefore not simply that the M24 mechanism disappears; a one-formation
intervention affects too little of a six-formation network.

The two release arms isolate duration.  Returning cross-formation posterior
participation early reduces tracking, terminal consensus, and byte savings.
The damaging input is persistent over this H=3 state rather than a single
impulse.

Finally, the current observable scores are
`[0.0212754, 0.0190650, 0.0180059, 0.0173466, 0.0083419, 0.0078573]` for the
ranked formations `[4,2,3,5,1,6]`.  The first four carry `82.4%` of total
positive supported rescue mass, with a clear drop before formations 1 and 6.
This motivates a scale-independent next selector: choose the smallest ranked
prefix covering at least `80%` of current positive risk mass and hold it for
the complete H=3 window.  It selects `[4,2,3,5]` at t=72 without hard-coding a
formation count or identifier.

## Boundary

This is opened seed-211 development evidence.  The 80% selector must be
frozen before its paired t=72 outcome; t=100 and t=128 remain unopened until
that selector passes the same strict 5% gate.
