# V242 causal minimum formation-backbone finding

## Joint M24 result

V242 keeps one current physical directed cycle inside every formation and one
gateway message in each direction for every edge of the causal formation tree.
It therefore executes exactly `N+2(F-1)=30` directed posterior messages per
round in M24, compared with 48 for the V241 full causal route.  Omitted local
residual weight returns to the receiver self posterior.

| Metric | Fixed formation tree | V241 causal 2N | V242 minimum backbone | V242 vs fixed |
|:--|--:|--:|--:|--:|
| Full E-OSPA | 123.211 | 121.074 | 122.621 | +0.479% |
| Full position RMSE | 8.906 | 8.794 | 8.237 | +7.513% |
| Focus inter-formation disagreement | 138.025 | 131.664 | 134.841 | +2.307% |
| Attempted posterior bytes | 35,469,296 | 39,171,480 | 30,371,232 | +14.373% saving |
| Directed messages per round | 46--48 | 48 | 30 | 37.5% structural reduction from 2N |
| Strong-connectivity fraction | 0.431 | 1.000 | 1.000 | -- |

V242 passes the registered goal-direction gate: E-OSPA, RMSE, consistency, and
attempted bytes all improve over the matched fixed-tree reference.  It does
not pass the paper threshold because the E-OSPA gain is below 5% and the
weakest formation E-OSPA regresses.

## Local and temporal diagnosis

All four formation-level RMSE values improve relative to the fixed tree.  The
per-formation RMSE gains are `[+2.866,+7.270,+10.891,+3.740]%`, and the worst
sensor RMSE improves by 8.462%.  Per-formation E-OSPA gains are
`[+0.646,+1.758,-2.036,+1.317]%`; F3 is the remaining E-OSPA tail.

V242 differs from both references from the first page because it removes the
local residual messages at all times.  Before the first tree repair (`t<70`),
this lowers RMSE by 10.856% but worsens E-OSPA and disagreement by 0.817% and
0.680%.  After repair, all three outcome directions improve:

| Interval | E-OSPA gain | RMSE gain | Disagreement gain |
|:--|--:|--:|--:|
| `t=70..150` | +1.091% | +2.270% | +4.720% |
| `t=151..160` | +4.622% | +18.989% | +6.717% |

The full causal route remains better for E-OSPA and consistency, while the
minimum backbone is substantially better for RMSE and communication.  Relative
to V241, V242 saves 22.466% attempted bytes and improves RMSE by 6.329%, but
loses 1.277% E-OSPA and 2.413% focus consistency.

## Method decision

The result rejects both extremes as the final method.  Always restoring all 48
messages preserves more E-OSPA/consensus value but creates communication
overhead and an RMSE tail.  Always using the 30-message minimum backbone closes
RMSE and communication but removes some useful E-OSPA support before the first
repair and in F3.

The next method should therefore treat V242 as a guaranteed minimum backbone
and selectively add a small number of residual edges only when their causal
information value justifies the payload.  The observable value should combine:

- receiver/sender label-support and existence disagreement;
- spatial uncertainty and posterior novelty;
- receiver FoV loss versus sender FoV retention;
- current link reliability and predicted persistence; and
- residual payload bytes.

Physical reachability, instantaneous strong connectivity, the residual-message
budget, and deterministic fallback remain hard constraints.  A later GNN may
learn a residual correction to this analytic edge value, but it should not own
the safety projection or generate unconstrained graphs.

The original formation-braid scene also has a task-coupling limitation: target
handoffs are confined to the same independent formation pairs as the platform
overtakes.  V244 keeps the method frozen while testing a target mapping that
crosses every initial formation-tree cut.  This scene check precedes any
learned-value implementation so that method and scene effects remain separable.

## Evidence boundary

This is one opened M24 development seed on an exploratory scene.  V242 is the
current best joint-direction result for the formation-braid main line, not an
X36 result or a paper-level generalization claim.
