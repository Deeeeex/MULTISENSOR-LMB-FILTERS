# V28 conflict-aware selective formation mixing

## Research decision

V27 rejects the hypothesis that faster global formation mixing is itself a
safe objective.  On its frozen primary state, every stronger-mixing action
reduced posterior-summary disagreement but increased cardinality error and
tracking loss.  V28 therefore treats network contraction and posterior
conflict as two separate terms rather than optimizing contraction alone.

For a label `ell`, Bernoulli KLA satisfies

`logit(r_bar_ell) = sum_i omega_i logit(r_i,ell) + log(eta_ell)`,

where `eta_ell <= 1` is the weighted spatial-overlap normalizer.  A larger
cross-formation weight can improve graph contraction while a small
`eta_ell` simultaneously suppresses label existence.  A useful routing
action must control both effects.

The corresponding dynamic interpretation is

`e_(t+1) <= tau(P_t) e_t + d_eta(P_t, posterior_t)`.

Here `tau(P_t)` represents how the stochastic routing matrix contracts
existing disagreement, while `d_eta` represents new conflict introduced by
incompatible label densities and local innovations.  V26 showed that graph
connectivity alone does not control `tau` at the relevant horizon; V27 showed
that reducing `tau` alone can enlarge the effective conflict disturbance.

## Action space

The registered one-lane reference cycle is always available.  V28 adds only
local, scale-compatible actions:

1. `damp-reference-f`: keep the graph but reduce the reference cross-edge
   trust entering formation `f` from 0.05 to 0.025;
2. `pair-redistribute-f-g`: swap two local residual inputs to create a
   reciprocal `f`--`g` exchange at weight 0.025, while reducing the two
   affected reference-cycle inputs to 0.025; total cross-formation residual
   trust is exactly unchanged;
3. `pair-add-low-f-g`: add the same reciprocal pair at weight 0.025 without
   damping the reference inputs;
4. `pair-add-reference-f-g`: add the pair at the reference residual weight
   0.05.

Adding a pair always replaces two local residual messages, so every action
has the same directed-message count as the reference.  Candidate gateway
endpoints maximize current-posterior Bernoulli/KLA compatibility subject to
physical reachability and source-permutation constraints.  The action count
is `1 + F + 3 F(F-1)/2`, quadratic rather than exponential in the formation
count; the same construction applies to D12, M24, X36, and X48.

## Frozen current-posterior projection

Every structural candidate is compared with the reference after exact
enumeration of current one-round link-delivery outcomes.  It is executable
only if all of the following hold:

- reference-weighted label-existence retention risk at most 0.01;
- every formation's mean expected-cardinality change at least -0.05;
- minimum retention ratio of labels supported by the reference at least
  0.80;
- no label expected above 0.5 under the reference falls below 0.5;
- expected one-step posterior disagreement increases by at most 1%;
- physical reachability, row-stochastic weights, equal message count,
  one-step sensor strong connectivity, and formation strong connectivity.

The retention score is asymmetric: it protects reference-supported
existence mass but does not assert that the reference is truth.  It closes a
specific safety gap left by an internal Bayes-risk score, which can become
small for a confidently empty posterior.  New-label gains and symmetric
posterior drift remain recorded diagnostics rather than being silently
discarded.

## Evidence gates

The first preflight reconstructs only the already-opened M24 seed-211,
`t=72` current posterior and executes no tracking continuation.  If no
nonreference action passes the frozen projection, V28 stops before another
outcome is opened.  If safe actions exist, the primary H=3 screen runs only
those actions plus the reference with common random numbers.

A strong action must then retain at least 2% mean E-OSPA gain, nonnegative
minimum-formation and worst-sensor gains, nonnegative H-window consensus
gain, no more than 5% attempted-byte increase, and rolling-B3 passage.  The
remaining opened M24 states are accessed only if the primary screen contains
a strong action.  GNN training still requires strong headroom on at least
three of four opened states.  X36, X48, and reserved validation seeds remain
sealed until those gates pass.

## Learning boundary

The later GNN does not replace the projector.  It predicts longer-horizon
value for formation-pair actions from current posterior, compatibility,
link, payload, and topology features.  The analytic existence-retention,
communication, physical, and connectivity constraints remain deterministic
at deployment.  This separation gives the learning component a meaningful
task—predicting the residual multi-step value among already safe actions—
without asking it to discover hard KLA or graph invariants from data.
