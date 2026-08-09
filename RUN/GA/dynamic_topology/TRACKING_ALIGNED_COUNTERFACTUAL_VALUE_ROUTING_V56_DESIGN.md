# V56: tracking-aligned counterfactual value routing

## Research decision

The posterior-payload branch is not the right route to the original goal.
V54 and V55 optimize preservation of a chosen V46 fusion result, while the
research target is to improve downstream tracking.  A selector penalized for
departing from V46 has no reason to discover a departure that improves
E-OSPA or cardinality.  V56 returns to dynamic topology and changes the
learning target rather than adding another posterior proxy.

V56 predicts the finite-horizon tracking value of a small set of
formation-local routing interventions.  A deterministic projector remains
responsible for physical reachability, message budget, temporal connectivity,
and reference fallback.  The learned model ranks safe actions; it never emits
an executable graph directly.

## Evidence that motivates the action space

Two earlier branches contain useful, complementary evidence.

1. The value-gated adaptive-dominant controller showed large M24 headroom and
   positive X36 clean-scale transfer, but its one-step posterior objective
   failed on the harder formation-FoV matrix.  On the frozen formation-FoV
   pairs, M24 improved by 8.852% on average but only 4/5 seeds improved and the
   aggregate worst sensor regressed.  X36 regressed by 0.893% on average.  The
   action space was useful; the one-step value proxy was not reliable.
2. The V35 staggered-recovery controller produced positive X36 gains at all
   three opened windows: 1.782%, 7.192%, and 0.812%.  Consensus and attempted
   bytes improved in every window, but only the middle state crossed the
   strong-gain threshold.  The mechanism is therefore state dependent rather
   than uniformly beneficial.
3. The earlier v13 representation experiment used only five M24 states and 95
   pooled action rows.  Its leave-one-seed-out mean-gain correlation was
   negative and it selected the reference in every state.  This rejects the
   idea that the existing tiny pooled-feature table is enough for learning;
   V56 requires a genuinely larger mixed-scale graph dataset, not a larger
   network fitted to the same five states.

The combined lesson is not that dynamic topology lacks headroom.  It is that
an always-on analytic rule averages together high-value and low-value states.
The missing component is a scale-shared predictor of *when* and *where* an
intervention improves future tracking.

## Safe local intervention bank

For every formation, the runtime constructs a bounded set of local modes from
the current observable state:

- `reference`: retain the registered safe source and trust;
- `source-trust`: use the adaptive-dominant candidate source with trust in
  `{0.30, 0.50, 0.70}`;
- `protect`: temporarily withhold the current harmful cross-formation input;
- `recover`: restore a mature withheld input according to the V35 staggered
  recovery rule.

The source-trust modes reuse the strongest part of the earlier value-gated
controller.  Protect/recover reuse V35's temporal mechanism.  The action bank
grows linearly with formation count before joint projection; it does not
enumerate arbitrary sensor-level graphs.

Every local mode records its exact changed receiver rows, fusion weights,
attempted payload, current physical support, and temporal route history.  A
reference action is always present.

## Counterfactual teacher

Offline training states are generated from radial, convoy, relay,
merge-split, and curved-corridor scenes at D12, M24, and X36 scales.  Crossing
is retained as a stress family rather than mixed silently into the primary
matrix.

At each selected state, paired common-random-number rollouts evaluate the
reference and every formation-local mode over a short horizon.  Truth and
future measurements are used only to compute offline targets.  The deployed
policy receives neither.

The teacher stores a vector rather than one prematurely weighted score:

- network mean position E-OSPA improvement;
- absolute-cardinality-error improvement;
- affected-formation and worst-sensor E-OSPA improvement;
- inter-formation disagreement improvement;
- attempted and delivered byte saving; and
- any temporal-connectivity or execution failure.

The main value target is the paired finite-horizon reduction in
`E-OSPA + lambda_card * cardinality_error`.  Tail, consensus, communication,
and execution outcomes are separate constraint heads.  This directly repairs
the objective mismatch seen in V49--V55.

## Scale-shared graph value model

The deployable model is a small formation graph network with shared weights.
Each formation node receives only current observable summaries:

- normalized network and formation size;
- label-existence, covariance, association-confidence, and mixture-complexity
  summaries;
- innovation and measurement-support summaries;
- current visibility/opportunity and FoV-boundary summaries;
- current and recent source load, link reliability, payload, and route age;
- reference-relative one-round fusion diagnostics; and
- the candidate mode and trust value.

Physical formation pairs supply normalized distance, bidirectional link
reliability, common-label support, posterior conflict, and route-history edge
features.  One or two message-passing layers produce a context embedding for
each formation.  A shared mode head predicts mean value and lower-tail value
for every local intervention.  Absolute sensor indices, formation IDs, label
IDs, seed IDs, truth, and future quantities are excluded.

Training uses pairwise reference-relative ranking plus robust regression of
the value vector.  Whole-scene and whole-seed splits are required; random
state splits would leak nearly identical adjacent states.  D12 may increase
state diversity, but M24 and X36 examples must both appear in training so the
model is not asked to extrapolate network size from M24 alone.

## Conservative online decision

At a decision time:

1. Construct the reference and all currently available local modes.
2. Predict each mode's tracking value and lower confidence bound.
3. Discard modes whose tracking lower bound is nonpositive or whose predicted
   cardinality/tail/consensus bound violates its registered margin.
4. Combine the remaining formation-local modes with a bounded beam search.
5. Project every proposal through the exact physical, row-stochastic,
   message-budget, rolling-connectivity, payload, and staggered-recovery
   constraints.
6. Select the feasible proposal with the largest conservative tracking value;
   use the reference when no proposal clears the value margin.

The projector may delete or replace an unsafe proposal, but it may not turn a
negative-value mode into an accepted action.  Projected proposals are rescored
before execution.  The output is therefore safe by construction with respect
to the registered structural constraints, while tracking performance remains
an empirical, data-dependent claim.

## Theory target

The theoretical contribution has two parts.

First, feasibility is deterministic: every executed route satisfies physical
support, the fixed communication contract, row-stochastic fusion weights, and
the registered rolling information-flow constraint because it is returned by
the exact projector.

Second, let `q_f(a)` be the true finite-horizon value of a local mode and
`L_f(a)` its calibrated lower bound.  If all accepted lower bounds cover their
true values and the residual interaction between simultaneously selected
formation modes is bounded by `delta`, the selected feasible joint action has
nonnegative value up to the measured interaction term.  With uniform value
prediction error `epsilon`, the finite action-set regret is bounded by a term
proportional to the number of changed formations times `epsilon`, plus
`delta`.  Coverage and interaction residuals must be measured on held-out
scene-seed blocks; this is a conditional guarantee, not an unconditional
tracking theorem.

## Scene matrix

All primary scenes use the same six-sensor formation hardware, 120-degree FoV,
hard range limit, detection model, clutter model, and motion constraints.

| Family | Main difficulty | M24 | X36 | Role |
|:--|:--|:--:|:--:|:--|
| Radial | center handover and blockage | yes | yes | primary |
| Convoy | parallel motion and offset corridors | yes | yes | primary |
| Relay | long information chain | yes | yes | primary |
| Merge-split | changing physical topology | yes | yes | primary |
| Curved corridor | smooth ownership transfer | yes | yes | primary |
| Crossing | abrupt orthogonal conflict | yes | yes | stress |

This matrix separates a general routing effect from a result that works only
for the original surround-like geometry.

## Development sequence and stopping rules

1. Reconstruct a compact mixed-scale teacher set from existing paired M24 and
   X36 caches before generating new rollouts.
2. Measure oracle headroom of the combined local-mode bank.  Continue only if
   both M24 and X36 have at least 5% aggregate oracle gain and at least two
   thirds of states contain a positive tail-safe action.
3. Train a small shared linear/MLP sentinel and the graph model on identical
   splits.  The GNN is justified only if it improves whole-seed ranking regret
   and positive-action recall.
4. Freeze the feature schema, model, confidence calibration, beam width,
   projector, and fallback before paired full-episode evaluation.
5. Run development on one M24 and one X36 scene-seed pair, then expand to the
   five primary scene families with fresh seeds.

The target for the final paired matrix is at least 5% mean full-episode
tracking improvement at both M24 and X36, positive tracking change in at least
75% of scene-seed pairs, no aggregate cardinality or worst-sensor regression,
and no increase in attempted communication.  Communication saving is a
secondary Pareto benefit; it cannot rescue a failed tracking gate.

## Immediate implementation checkpoint

The next code milestone is deliberately small:

- add a common local-mode record shared by the adaptive-dominant and V35
  branches;
- export truth-free node, edge, and mode features from existing H=3 caches;
- join paired rollout targets only in the offline dataset builder; and
- run an oracle-headroom report on the existing M24/V37 evidence before any
  new long training batch.

No GNN training or new large experiment is justified until this combined
action-space headroom check passes.
