# V87 time-expanded effective KLA graph design

## Why graph reachability is insufficient

V86 changes five downstream receiver rows, yet M24 remains almost identical to
the one-edge V84 arm.  X36 improves its affected formation by `1.635%`, but
the network gain is only `0.298%` and position consensus worsens.  A physical
path therefore has no useful value merely because it reaches more nodes.

KLA is nonlinear in the posterior.  The information sent on the second edge
is not the original source posterior: it is the gateway posterior after the
first KLA.  Edge scores cannot consequently be added or multiplied without
checking what survives the composition.

## Current-only two-hop counterfactual

For each frozen safe V84 source--gateway edge, V87 constructs two parallel
two-round posterior paths from the same opened current state.

- The reference path applies the registered KLA reference for two rounds.
- The candidate path changes only the gateway's first-round residual source,
  then promotes the acquired gateway into one downstream receiver's existing
  `0.70` slot while retaining the displaced dominant sender at `0.05`.

The second-round inputs are the first-round fused posteriors, not the original
local posteriors.  This makes the path value a property of composed KLA
operators rather than static graph reachability.  Prediction, future
measurements, truth and tracking outcomes are not read.

## Label-wise path value

V87 measures two complementary effects for every physically reachable
downstream receiver.

1. **Existence transport.** Labels are novel only when the candidate source
   has current detection-associated support that both the gateway and its
   incumbent source lack.  V87 measures how much of the gateway's first-hop
   existence gain survives at the downstream receiver, subtracts loss on
   receiver- or incumbent-supported labels, and rejects downward `r=0.5`
   crossings.
2. **State alignment.** For novel labels, a useful path moves the downstream
   fused position toward the uniquely observation-supported source.  For
   protected labels, it must not move farther from the support-weighted
   receiver/incumbent position anchor.  Position changes are normalized by
   the common OSPA cutoff.

A path is enabled only when it has positive normalized existence net, at least
`5%` first-hop existence survival, nonnegative state-alignment net, no
protected downward crossing, and supported harm no larger than supported
gain.  These thresholds are frozen before the primary source probe.

## Evidence sequence

The first probe uses only the already opened M24 t=104 and X36 t=112 source
states.  It reports all five downstream rows and does not execute a route or
read V86 tracking outcomes.  If both scales expose at least one positive path,
the unchanged calculation expands across times and scene styles and measures
formation coverage and action conflicts.  Only then may a repeated
full-episode policy be opened.  GNN training remains closed until the
time-expanded action space has positive cross-scale source headroom.
