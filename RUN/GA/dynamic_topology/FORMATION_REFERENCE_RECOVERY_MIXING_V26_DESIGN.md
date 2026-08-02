# Formation reference-recovery mixing audit v26

## Question

v25 shows that selected graphs satisfy rolling-B3 but still accumulate
consensus debt.  v26 asks whether topology-only mixing diagnostics can explain
that failure and serve as a sufficient hard constraint for the next method.
This is a deterministic posthoc audit of the saved H=5 screen, not a new
performance experiment.

## Why rolling-B3 may be insufficient

Rolling-B3 checks whether the union of the most recent three selected graphs
is strongly connected.  This is a reachability condition: information has a
possible path across the network.  It does not measure how much fusion weight
crosses that path or how rapidly a product of time-varying fusion matrices
reduces node-to-node differences.

For a row-stochastic fusion matrix, the Dobrushin coefficient is a standard
worst-case mixing diagnostic.  A coefficient below one indicates one-step or
multi-step contraction of row disagreement; a value of one gives no useful
short-horizon contraction certificate.  We additionally record the centered
spectral norm and maximum centered-row dispersion because the Dobrushin
coefficient can saturate on sparse graphs.

These quantities are topology-only diagnostics.  KLA receives a new local
Bayes update at every scan, so posterior disagreement also contains a
state-dependent disturbance from heterogeneous measurements and innovations.
The audit therefore tests whether graph mixing is sufficient, not whether it
is irrelevant.

## Frozen source and calculations

- source: the official v25 H=5 screen generated at commit `27b2c87`;
- arms: one all-reference trajectory and four frozen recovery candidates;
- action bank: all 256 mode vectors from the opened M24 state;
- exact analysis steps: `1, 3, 5, 7, 10, 20, 30`;
- after step 5, weight-only extrapolation appends the fixed reference matrix;
- sensor-level and formation-collapsed products are both evaluated;
- no filter trajectory is rerun and no new truth outcome is opened.

The formation-collapsed matrix records, for a receiver formation, the average
fusion weight assigned to every sender formation.  If all 256 actions have the
same collapsed matrix, then the current action representation only reallocates
gateways inside formations and cannot change coarse inter-formation mixing.

## Diagnostic rules

The current graph-only safety state is rejected as sufficient if all three
conditions hold:

1. sensor-level Dobrushin coefficients remain saturated at one throughout the
   realized H=5 window;
2. all 256 actions share the same formation-level matrix within `1e-12`;
3. at least one candidate has a centered product metric no worse than the
   reference but still has negative final-step consensus gain.

If rejected, the next action family must vary cross-formation fusion mass and
the value/risk state must include posterior or innovation heterogeneity.  A
multi-head GNN is still unauthorized until that redesigned action family shows
strong-safe headroom on multiple development states.

This audit is outcome-inspected diagnosis only.  Seeds 223/227, X36, and final
seeds remain unopened.

## Result

The official deterministic audit at generation commit `b22a34c` passed all
source and row-stochasticity checks and triggered all three pre-registered
rejection conditions.

- the sensor-level Dobrushin coefficient was exactly `1` for every arm through
  H=5; the first nonsaturated step appeared only at step 6 or 7;
- all 256 actions shared the same formation-level matrix within
  `1.11e-16`;
- each receiver formation retained average weight `0.9916667` locally and
  assigned only `0.0083333` to the next formation;
- candidates 3, 4, and 5 had H=5 centered spectral norms no worse than the
  reference but still had negative final-step consensus gains;
- candidates 4 and 5 were also counterexamples under maximum centered-row
  dispersion.

The current action representation therefore changes which sensor acts as a
gateway but leaves coarse inter-formation mixing mass fixed.  Rolling-B3 and
the tested topology-only product metrics cannot distinguish the realized
consensus outcomes.  This formally closes graph-only safety scoring for the
present action family.

The authorized redesign must expose cross-formation mixing mass as an action,
scale that mass with formation size, and condition predicted tracking and
consensus risk on posterior/innovation heterogeneity.  This audit still does
not authorize GNN training or any reserved-seed evaluation.
