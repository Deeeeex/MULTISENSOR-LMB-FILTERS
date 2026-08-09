# V62 layered formation-label result

## Positive-control result

V62 passes the M24 `t=104` positive-control gate with the control-only
data-plane schedule.

| Action | H=3 tracking | Worst sensor | Window consensus | Terminal consensus | Attempted-byte saving | Strict |
|:--|--:|--:|--:|--:|--:|:--:|
| control only | `+10.393%` | `+36.402%` | `+24.336%` | `+8.820%` | `+0.225%` | yes |
| sender supported only | `+1.694%` | `+5.184%` | `+3.931%` | `+2.285%` | `-0.347%` | no |
| sender supported or existence >= 0.50 | `-0.000%` | `-0.000%` | `-0.002%` | `-0.005%` | `+0.100%` | no |
| receiver need aware | `+0.517%` | `+0.001%` | `+1.248%` | `+0.279%` | `-0.240%` | no |

The control-only arm suppresses `[48, 32, 16]` label inputs over the three
scheduled steps while preserving the registered physical carrier graph.  Its
per-step tracking gains are `[14.324%, 12.604%, 4.369%]`.  The first two steps
save `[139,392, 69,432]` bytes; the terminal recovery adds `198,216` bytes, but
the complete H=3 window still saves `10,608` bytes.

## Interpretation

The strong effect is not produced by transmitting a smaller set of allegedly
better labels.  Restoring 36, 23 and 10 sender-supported labels across the
same schedule collapses the gain from `10.393%` to `1.694%`.  Keeping nearly
all high-existence labels removes the effect entirely.  At this state, the
useful intervention is temporary exclusion of complete cross-formation
posterior inputs from the fusion data plane.

This yields a cleaner architecture: the physical graph carries control and
maintains reachability, while a time-varying effective KLA graph determines
which posterior inputs actually enter fusion.  X36 success now depends on
selecting the correct multi-step formation schedule.  V62 remains opened M24
development evidence and does not authorize a cross-scale or validation
claim.
