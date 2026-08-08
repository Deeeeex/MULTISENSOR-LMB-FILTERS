# Posterior-aware cycle routing V50 design

## Decision being tested

V49 asks a purely structural question: among physically feasible formation
cycles, which route gives the best expected graph contraction while preserving
the synchronized B4 communication budget?  This is scalable and causal, but it
does not ask whether the selected cross-formation messages contain information
that is useful to the receiving trackers.

The X36 convoy paired run first tests whether graph contraction alone translates
into tracking improvement.  If V49 does not produce a clear tracking gain over
V46, V50 will keep the feasible-cycle action space and deterministic fallback,
but replace the cycle ranking objective with current-posterior information
value.  The 1% V49 contraction threshold will not be tuned against the tracking
outcome, because that would not repair the mismatch between graph disagreement
and estimator utility.

## What V50 keeps

- the same local filtering and full LMB posterior fusion as the baseline;
- the same physical graph, incoming-degree limit, synchronized B4 schedule and
  per-window transmission budget;
- formation-cycle candidates, so search grows with formation structure rather
  than arbitrary sensor-level graphs;
- reliability and contraction as hard feasibility/safety checks;
- a deterministic V46 fallback when no candidate has material posterior value.

Posterior features are used only to choose a route.  They do not replace the
posterior exchanged by the tracker and do not imply full/light-posterior
equivalence.

## V50 online decision

At the first phase of each four-step B4 window:

1. Enumerate the same physically feasible formation cycles as V49 and
   materialize their exact sensor-level routes.
2. Score every physically available directed cross-formation edge from the
   current observable LMB posteriors.  The initial analytic score pools
   label-wise existence transfer, uncertainty reduction, association quality,
   posterior compatibility and link reliability.  It uses no target truth,
   future measurement or realized delivery outcome.
3. For each cycle, aggregate the selected incoming edge values per formation.
   Rank candidates lexicographically by worst-formation value, then mean value,
   then expected contraction.  This prevents a gain in one formation from
   concealing harm in another.
4. Reject candidates that violate the physical/budget/degree contract or are
   structurally worse than V46.  Select a cycle only when its posterior value is
   materially better than the V46 route; otherwise use V46 unchanged.
5. Hold the selected route for the complete four-step window.  Current
   posterior features are consulted again only at the next decision boundary.

This separates three roles cleanly: posterior features estimate message value,
the cycle projector guarantees a scalable feasible topology, and the baseline
fallback limits avoidable regressions.

## Data-driven extension

The first V50 implementation should remain analytic so that the source of any
gain is identifiable.  A GNN is justified only after the constrained action
space shows useful tracking headroom.  It would predict edge or cycle value from
the same permutation-invariant label-set features, trained with offline paired
counterfactual tracking returns.  The learned model would rank candidates; it
would not bypass the deterministic feasibility, formation-tail and fallback
rules.

## Evidence sequence

1. **Current X36 convoy pair:** compare V49 with V46 using paired measurements,
   link-delivery draws and filter seed.  A graph-only structural score is not a
   success metric by itself.
2. **Method decision:** retain V49 only if it gives a clear tracking improvement
   without worse formation-tail behavior or increased communication.  Otherwise
   implement the analytic V50 ranker above.
3. **Development check:** one paired X36 convoy run establishes whether V50
   repairs the graph/estimator mismatch.  Avoid multi-seed tuning at this stage.
4. **Scene transfer:** after freezing the method, evaluate the X36 merge-split
   and curved-corridor scenes, followed by fresh M24/X36 seeds.

The target remains a stable, practically meaningful tracking gain at M24 and
X36 scale, not merely a better graph proxy.

## Implementation checkpoint

The analytic V50 arm is now implemented as an isolated runtime policy.  For
each receiver it compares the candidate residual sender with the V46 residual
sender using current label-set posterior transfer features.  Receiver deltas
are pooled by formation using equal mean and lower-quartile weights; cycle
ranking then uses equal worst-formation and network-mean weights.  Up to eight
posterior-ranked proposals receive the exact synchronized-B4 structural check.
The candidate must have positive combined posterior value, no formation worse
than `-0.05` on the receiver-conditional percentile scale, and no worse graph
contraction than V46.

A minimal eight-step real-filter smoke run confirms that V46 and V50 retain the
same `2N,N,N,N` schedule and that V50 executes its phase-one cycle decisions.
This is an execution check, not tracking evidence.  The candidate-only tracking
runner will reuse the completed V49 pair's V46 summary, delivery seed and filter
seed so the baseline is not rerun.

## X36 tracking-difficulty interpretation

The first X36 convoy V46 arm has full-horizon position E-OSPA `126.370`, focus
E-OSPA `123.494`, and mean absolute cardinality error `14.534` in a 24-target
scene.  This is not explained by an impossible sensing layout.  Across the five
frozen geometry-calibration seeds, X36 convoy has `0.0%` focus blackout, only
`0.4%--0.5%` overall blackout, about `7.08--7.14` visible sensors and
`5.30--5.34` expected detections per active target in the focus window.  Each
sensor sees about `4.72--4.76` targets on average, so local measurement load is
substantial but not near-global.

The current working hypothesis is therefore estimator-side: useful label
evidence is created by a small subset of sensors but is not maintained and
propagated effectively through the fixed dominant/residual routes.  This makes
the convoy run a meaningful test of posterior-aware routing rather than a reason
to widen the FoV or reduce the target count.  After V49/V50 finish, the decisive
diagnostic is the time-resolved cardinality and label-existence propagation by
formation.  If V50 cannot improve those quantities, the limitation is likely
deeper than graph selection alone and should not be hidden by making the scene
easier.
