# V153 safe graph Pareto value routing

## Why V152 is not simply relaxed

V152 asked a graph with the same number of transmissions to use no more
realized bytes than an independently executed static trajectory.  The M24
pilot showed why these are different resource notions: rank 4 used exactly
40 directed messages per page and improved every estimation criterion, yet
its full-posterior bytes were 1.962% higher.  Topology changes which posterior
is sent and also changes later mixture complexity, so equal message count
cannot imply equal trajectory bytes.

V153 does not rewrite that result.  V152 remains a failed strict
byte-neutral experiment.  V153 freezes a new two-cost question before any
X36 outcome is opened.

## Resource and safety model

For each page, the action is one complete directed KLA graph.  Every selected
edge carries the full Bernoulli Gaussian-mixture posterior; label omission,
moment projection and payload compression are disabled.

The hard feasible set enforces:

- current physical reachability;
- the registered row-stochastic KLA weights;
- exactly `2N - 2F` directed transmission opportunities per page (40 for M24
  and 60 for X36); and
- sensor- and formation-level rolling B3 strong connectivity.

Realized attempted bytes are a second cost, not assumed constant.  The
offline oracle observes the complete H=8 trajectory and retains a candidate
only when mean E-OSPA improves, worst-sensor E-OSPA does not regress, the
weakest formation does not regress, consensus OSPA does not regress, all hard
graph checks pass, and attempted bytes increase by no more than 5% relative
to the better static CW/CCW trajectory.  The 5% ceiling and 5% minimum mean
gain are frozen before X36 evaluation.

Among retained candidates, the oracle selects the lowest mean E-OSPA and
reports the full tracking--byte vector.  This is a Pareto headroom diagnostic,
not a deployable policy and not a claim that bytes are saved.

## Scale gate

The first unopened test is X36 seed 83 using the unchanged V152 graph
generator.  M24 seed 83 is development evidence that motivated this resource
separation and cannot establish stability by itself.  Full development
expansion is allowed only if X36 seed 83 contains a retained candidate with at
least 5% mean gain and no tail, formation or consensus regression.

For a scale-level result, at least four of the five pre-registered seeds
`[83, 89, 97, 101, 103]` must contain a retained dynamic action, aggregate
mean gain must be at least 5%, aggregate tail/formation/consensus gains must be
nonnegative, and aggregate attempted-byte increase must remain within 5%.
M24 and X36 pass independently; one scale cannot compensate for the other.

## Deployable selector after headroom

Only after both scales pass may a graph-level value model be trained.  Its
input is limited to current observable geometry, link state, posterior-set
summaries, per-sender payload-size metadata and recent selected/delivered
graphs.  It predicts a vector of tracking risk, tail risk, consensus risk and
payload cost for every complete safe codebook graph.  A calibrated lower
confidence rule selects a graph only when it dominates the static fallback
under the configured byte price or cap; otherwise the static action is used.

The learning target is graph value, not imitation of the retrospective oracle
rank.  This preserves a clear separation between the exact safety projector,
the data-driven value estimate and the user-visible communication trade-off.
