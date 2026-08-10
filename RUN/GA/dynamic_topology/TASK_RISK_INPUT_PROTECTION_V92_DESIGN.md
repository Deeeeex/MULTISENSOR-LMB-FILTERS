# V92 task-risk input protection

## Decision after V89--V90

V89 is rejected under both receiver semantics.  With legacy missing-label
renormalization it degrades M24 by 0.541%; with the FoV-aware censored receiver
it degrades M24 by 0.139%, including the worst sensor, worst formation and
consensus.  The failure is therefore not repaired by changing how absent
labels are interpreted.

The acquire--broadcast action is also structurally mismatched to its stated
purpose.  A gateway first receives novel information through a 0.05 residual
weight, then sends its entire recursively updated posterior to its formation
with weight 0.70.  In the idealized log-pool composition the original novel
source receives only about `0.70 * 0.05 = 0.035` indirect weight, whereas the
gateway's other information is strongly amplified.  KLA normalization,
prediction and update make this only an explanatory approximation, not an
exact recursive identity, but it shows why the action can improve agreement
without improving the estimate.

V92 returns to the safer causal mechanism that previously produced positive
M24 H=3 evidence: temporarily protect a formation from a low-weight
cross-formation input when the actual current fusion predicts that the input
will harm its posterior, then recover the input before rolling connectivity is
lost.  The first step is a current-only headroom scan, not a full tracker.

## Action and matched baseline

The baseline is the V91-certified physical-tree route.  Every receiver keeps
self weight 0.25, one local dominant input with weight 0.70 and one residual
input with weight 0.05.

One V92 probe selects one formation and removes every 0.05 cross-formation
residual input entering that formation.  Removed weight returns to receiver
self weight.  The high-weight local route is never changed, no replacement
sender is introduced, and the candidate saves messages relative to the static
baseline.  The existing input-bundle constructor rejects nonphysical actions
and verifies a two-step reference recovery reserve under rolling-B3.

## One-round task-risk proxy

For each route, V92 enumerates all independent delivery outcomes on the
selected incoming edges and invokes the same heavy-payload LMB fusion operator
used by the filter, including the FoV-aware censored missing-label semantics.
For Bernoulli label \(\ell\) with existence \(r_\ell\), position covariance
\(P_\ell\), and Euclidean OSPA cutoff \(c\), the outcome risk is

\[
R = \sum_\ell \left[
\tfrac{1}{2}\min(r_\ell,1-r_\ell) +
\tfrac{1}{2}r_\ell\min\!\left(
\frac{\operatorname{tr}(P_\ell)}{c^2},1\right)
\right].
\]

The first term is the minimum equal-cost Bernoulli presence-decision error.
The second is a capped conditional position-MSE proxy.  Delivery-weighted
receiver risks are aggregated as the network mean plus one half of the worst
receiver quartile.  The scan also reports the worst receiver and worst
formation separately.

This quantity is not E-OSPA and is not a recursive Bayes-risk theorem.  It is a
truth-free, task-oriented ranking proxy.  A candidate is called safe only if
it lowers the aggregate, worst-receiver and worst-formation proxy risks and
passes the existing reference-relative label-retention gates: no decision
threshold crossing, at least 80% supported-label retention, bounded existence
retention risk and bounded formation cardinality loss.

## Headroom decision

The scan uses every cached braided-handover state from t=40 to t=140 in steps
of four for both M24 and X36.  It evaluates the static route and every single
formation input-protection action without truth, future measurements or
tracking outcomes.

Implementation of a full-episode controller is opened only if both scales
have safe positive actions on at least 10% of cached times, median positive
task-risk gain of at least 0.25%, and a peak of at least 1%.  These are
implementation-headroom gates, not tracking claims.  The later full episode
still requires at least 5% full and focus E-OSPA gain with no worst-sensor or
worst-formation regression.

If this action family has no cross-scale headroom, the next action family may
replace the 0.05 residual sender while preserving the dominant local input.
The rejected V89 0.70 broadcast is not reopened.

## Data-driven boundary

A GNN is considered only after an analytic V92 controller produces stable
M24/X36 tracking gains.  It may learn the per-formation or per-edge task-risk
advantage from compact posterior, FoV and link features.  Physicality, message
budget, rolling connectivity, label retention and static fallback remain a
deterministic projection; they are not learned.
