# V219 causal two-stage graph-value controller

## Research claim under test

The useful decision is not simply whether to remove communication edges.  A
receiver formation first decides whether a complete incoming posterior is
worth consuming on the current page.  Only after ordinary fusion can it know
whether a specific, supported complete label should be fused back.  A
deployable controller must preserve this information order while sharing one
policy across M24, X36 and larger formation graphs.

V219 tests the following claim: a formation-equivariant value model can reduce
posterior communication while protecting tracking and consensus when its
learned predictions are separated from exact topology, transport, label
support and byte-credit projections.

## Causal policy

At the start of a page, the precommunication head assigns each receiver
formation a twelve-coordinate H=3 value vector for withholding its incoming
cross-formation posterior payload for one page.  The only alternative at this
stage is the full-payload no-op.  Withholding must pass on its own; an expected
future repair is not allowed to certify it.

After an accepted withholding action and ordinary fusion, the post-fusion head
scores a bounded set of position-safe, current-transport-feasible and
credit-feasible complete-label KLA candidates.  Each score is an incremental
twelve-coordinate vector relative to keeping the withholding-only result.  A
no-repair action is always present.

The deterministic layer owns:

- current physical reachability and the rolling effective-KLA graph;
- one action per page, cooldown and no-op fallback;
- current request and response delivery feasibility;
- the 99% position-support guard for same-label hypotheses;
- exact attempted-byte credit, including synopsis, request and response;
- preservation of complete Bernoulli Gaussian mixtures.

The learned layer owns only finite-horizon value prediction.  Feasibility
scores, source risk and mixture overlap are inputs or guards, not tracking
certificates.

## Scale-equivariant representation

Each formation is a graph node with the V215 observable state and payload
features.  Each directed sender-to-receiver formation edge stores the selected
message fraction, fusion mass per receiver sensor and mean selected edge
weight.  A two-round shared message-passing encoder produces one embedding per
formation.  The first head reads the target receiver embedding.  The second
head reads the target receiver embedding, the candidate source-formation
embedding and the identifier-free V217 candidate features.

Sensor and formation identifiers are lookup metadata only.  Counts are
represented as fractions, per-receiver masses or logarithmic payload terms;
shared parameters and degree-normalized aggregation keep the input dimension
unchanged from M24 to X36 or X48.

The initial architecture comparison is deliberately small:

1. action-only ridge regression;
2. frozen two-round graph features followed by ridge regression;
3. a trainable two-round formation GNN.

Architecture selection uses leave-one-complete-training-trajectory-out error.
Calibration trajectories are reserved for vector lower bounds and evaluation
trajectories remain unopened until the policy, thresholds and fallback are
frozen.

## Compositional safety statement

For every protected absolute error or disagreement coordinate, let the
full-payload reference value be `R`, the withholding value be `W`, and the
post-fusion repair value be `K`, where lower is better.  If the first gate
accepts only when `W <= R`, and the second gate accepts only when `K <= W`,
then transitivity gives `K <= R`.  If no repair passes, the controller keeps
`W`; if withholding does not pass, it keeps `R`.  The same composition holds
for communication because the repair projection spends no more than the exact
cumulative credit created relative to the reference.

For learned decisions, the inequalities are applied to calibrated lower
confidence bounds of the corresponding gain vectors.  A trajectory-level
calibration rule and an explicit allocation of miscoverage across the two
stages are required before making a probabilistic claim.  This is not a proof
that the simulator represents a real sensor network, and base-route
calibration does not automatically cover recursively shifted controller
states.

## Experimental decision gates

The first method decision uses radial M24 and X36 training trajectories only.
Every trajectory contributes four V218 strata, and every state retains both
positive and negative continuous labels below the promotion gate.  The model
is frozen only after both scales contain saver, harmful-withholding,
repair-positive and no-repair roles.

The frozen controller is then evaluated, without retuning, in this order:

1. unseen radial M24 and X36 trajectories;
2. convoy and relay trajectories at both scales;
3. crossing, target-overlap, merge-split and other stress styles;
4. X48 scale extrapolation.

The main result requires paired reductions in E-OSPA and RMSE, non-regressive
consensus under the registered risk budget, and lower attempted bytes.  A
scenario-specific gain that fails to transfer to more formations remains a
development observation rather than a method result.
