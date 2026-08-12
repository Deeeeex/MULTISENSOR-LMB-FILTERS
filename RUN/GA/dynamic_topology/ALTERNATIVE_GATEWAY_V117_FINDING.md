# V117 finding: local F6 gateway migration amplifies delayed harm

## Result

V117 keeps the clockwise formation cycle, V113 F2--F5 abstention schedule,
message count and fusion-weight multiset fixed.  Each candidate moves the
F5-to-F6 entry and simultaneously transfers the F6-to-F1 return source so
that every sensor retains an outward influence path.  All three arms pass
row-stochasticity, weight-parity, static strong-connectivity and rolling-B3
checks, but none passes the tracking gate.

| Entry / return | Mean E-OSPA | Gain vs CW | vs V113 | vs V114 | vs V116 top-5 | Mature min. | Min. formation | F6 peers | Byte saving |
|:--|--:|--:|--:|--:|--:|--:|--:|--:|--:|
| `27→33 / 34→2` | 78.893034 | +3.558% | -0.527% | -0.733% | -0.626% | +2.037% | -4.331% | -1.976% | +3.361% |
| `27→35 / 36→2` | 79.185942 | +3.200% | -0.900% | -1.107% | -1.000% | +1.362% | -5.501% | -10.048% | +3.241% |
| `27→34 / 35→2` | 78.944955 | +3.494% | -0.593% | -0.799% | -0.693% | +2.916% | -4.329% | -10.046% | +3.400% |

The best entry-33 arm improves worst-sensor error, consensus and bytes, yet
its gateway sensor becomes `17.936%` worse at the terminal page and F6 is the
binding formation at `-4.331%`.  Current receiver quality therefore does not
identify a useful entry.  Moving the same sender posterior deeper into F6's
internal influence graph worsens both network mean and downstream tail.

## Structural lesson

A first entry-only implementation was rejected before producing a result:
replacing an internal residual slot removed that source's only outward
influence path and violated sensor-level rolling B3.  The corrected paired
entry-return action repaired this structural defect without relaxing any
safety gate.  Its subsequent tracking failure is therefore informative, not
a connectivity artifact.

V117 closes same-source within-F6 gateway placement.  It also rejects current
receiver E-OSPA as a gateway-value target.  No GNN should be trained to rank
this action family.

## Next bounded screen

The remaining local variable is the source posterior, not the entry.  V118
will keep receiver 32, the F5-to-F6 formation edge, the return edge and all
weights fixed, and exhaust the other five F5 senders.  This differs from the
earlier generic V68/V83/V95 alternative-source experiments: it is confined to
the opened X36 delayed-loss boundary and uses the full H=8 return gate.  If no
F5 source passes, all single-source local controls at this boundary are
closed and the method must change the formation-level carrier or introduce a
budget-matched multi-source entry.

V117 is privileged opened-development evidence at X36 seed 211, t=72, H=8.
It is not deployable, validation, or generalization evidence.
