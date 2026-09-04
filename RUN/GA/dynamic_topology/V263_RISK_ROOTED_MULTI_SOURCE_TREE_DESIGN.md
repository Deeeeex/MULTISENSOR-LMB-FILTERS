# V263 risk-rooted multi-source shortest-path tree

## Why V262 closes the single-donor formulation

The corrected V262 continuation separated two effects that had previously
been mixed together.  Holding the F1-to-F4 shortcut stable improved network
RMSE by 7.878% and F4 event RMSE by 24.311%, so stable formation-level path
shortening has real localization leverage.  Yet F4 event E-OSPA regressed by
0.567%, its mean absolute cardinality error rose from 10.677 to 10.885, and F2
RMSE regressed by 6.464%.  Nine donor switches were actually suppressed, so
these failures cannot be attributed to controller chatter.

At the V259 entry state, both F1 and F2 carry supported, lower-covariance
copies of F4's highest-risk label.  The current physical graph permits F1 to
reach F4 in two hops and F2 to reach it in two.  The V242 tree uses distances
three and two, respectively; the V262 tree improves F1 to two but lengthens
F2 to three.  It therefore selects one useful source by sacrificing another.
This is a tree-objective error, not a threshold error.

## Componentwise shortest-path principle

V263 selects the same at-risk formation and label using the frozen V261/V262
posterior-only gate.  It intervenes only when at least two other formations
meet the same coverage, existence and covariance-ratio source conditions.
Instead of choosing one donor, it builds a breadth-first spanning tree rooted
at the at-risk formation on the current physical formation graph.

For a root `r`, every breadth-first tree satisfies

```text
d_T(i,r) = d_G(i,r)  for every formation i,
```

where `G` is the current physical formation graph.  Because no spanning tree
can make any `d_T(i,r)` smaller than `d_G(i,r)`, this tree minimizes the whole
source-to-target hop-distance vector componentwise.  Consequently it also
minimizes every nonnegative weighted sum of localization-source and
set-support-source delays without introducing a tuned trade-off coefficient.

Each non-root formation chooses a parent one layer closer to the root.
Ties retain an incumbent edge first, then prefer higher current bidirectional
link reliability and finally the immutable formation UID.  This construction
is linear in the physical graph after sorting local ties and therefore extends
to more formations without enumerating spanning trees.

## Communication and evidence boundary

The selected formation tree is projected through the existing V249/V242
gateway assignment.  Local cycles, row-stochastic KLA weights, physical
feasibility and strong connectivity are unchanged.  The route still uses
exactly `N + 2(F-1)` posterior messages: 30 messages for M24.  V263 reallocates
tree edges; it does not add a residual payload.

The controller reads current LMB posterior moments, current physical links,
current link reliability and past selected topology only.  It reads no target
truth, future measurement, future link page or future tracking outcome.  Its
current-network synopsis is centralized and uncharged, so this is a mechanism
screen rather than deployment evidence.

## Frozen continuation gate

Run one paired continuation from the same V259/V242 state at `t=57`.  Before
tracking results are interpreted, the run must initiate a non-reference tree,
serve at least two eligible sources, strictly shorten at least one of them and
worsen none, keep every formation at its physical shortest distance to the
active risk root, remain physically and strongly connected, and use exactly
30 messages per page.

The action may proceed to a complete M24 run only if F4 event E-OSPA and RMSE
both improve, network E-OSPA, RMSE and consistency stay within the registered
2% guards, no formation regresses by more than 3%, and spliced full-episode
traffic remains below the static baseline.  The thresholds are inherited and
will not be retuned after observing this continuation.
