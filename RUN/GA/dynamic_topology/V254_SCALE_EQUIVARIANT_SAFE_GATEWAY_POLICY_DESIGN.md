# V254 scale-equivariant safe gateway policy

## Method decision

The minimum causal backbone already fixes the expensive part of topology
control: it preserves one local directed cycle per formation and a
bidirectional spanning tree between formations.  Its remaining failure is not
insufficient connectivity, but choosing the wrong physical sensor pair for a
formation-to-formation message when one formation is temporarily difficult to
localize.

V252 supports this decomposition.  At seed 1302 and `t=60--62`, changing only
the two incoming gateways of one receiving formation improves network E-OSPA,
RMSE and consistency while also reducing attempted bytes.  The final method
should therefore learn the value of physical gateway edges and retain an exact
topology projection, rather than classify a fixed catalogue of whole graphs.
The observation is still development evidence and does not authorize a V254
tracking claim.

## Problem formulation

At time `t`, let `G_t=(V,E_t)` be the current directed physical graph and let
`g(i)` identify the physical formation containing sensor `i`.  V242 supplies a
causal formation-level tree `T_t`.  Each undirected tree edge is represented by
two directed formation arcs.  For every directed arc `p -> q`, the gateway
policy chooses one physical edge `i -> j` such that

- `g(i)=p` and `g(j)=q`;
- `i -> j` is present in `E_t`;
- two messages entering the same formation do not reuse the same receiving
  sensor; and
- the local formation cycles and the formation tree itself are unchanged.

This separates the discrete decision into two levels: V242 decides which
formations must communicate, while V254 decides which sensors should carry
those messages.

## Observable graph representation

One shared graph encoder operates on any number of sensors and formations.
Its inputs are available before the communication action:

- node features: expected cardinality, cardinality variance, position
  covariance, association confidence and detection support from the local LMB
  posterior;
- directed-edge features: link reliability, normalized distance, sender to
  receiver information advantage, active-label overlap, state discrepancy and
  receiver need;
- causal history: whether the edge was recently selected and how often its
  requested gateway remained physically available; and
- structural roles: same-formation versus cross-formation edge and the
  direction of the current formation-tree arc.

Numeric sensor IDs, numeric formation IDs, target truth, future measurements,
future physical pages and alternative-action outcomes are excluded.  Shared
node/edge functions and symmetric aggregation make the score equivariant to a
permutation of physical sensors and formations.

Posterior moments are controller features only.  They neither replace the
transmitted LMB posterior nor alter the mixture-aware LMB-KLA fusion carried
out after an edge is selected.

V253 is the first linear screening baseline for this representation.  Its
47-, 167-, 327- and payload-aware 328-dimensional pooled summaries test whether
current observable posterior and edge state contain cross-seed value signal
before a graph model is trained.  Those pooled summaries include extrema and
dispersion and therefore cannot themselves be passed to the exact edge
projector: in general, the score of a complete assignment is not the sum of
scores of its selected edges.

V254 consequently introduces a separate projectable representation.  Every
currently feasible gateway edge contributes 20 normalized posterior/link
features, a causal recent-selection feature and its exact current full-LMB
payload contribution.  The first 21 contributions are divided by the number
of directed formation arcs, so summing an assignment recovers their means;
payload contributions are divided by the total estimated V242 attempted
bytes, so the candidate-minus-reference sum exactly recovers the estimated
attempted-byte ratio change.  Training, online scoring and matching therefore
share one additive objective.  Candidate-type indicators and numeric sensor or
formation identifiers are absent.  V253 can authorize this next screen, but
its coefficients are not reused as if they were edge values.

The linear scorer is reference-centered at assignment level rather than edge
level: the same standardized coefficient vector is applied to every feasible
edge, and the predicted advantage is the sum of candidate edge values minus
the sum of V242 edge values.  This keeps the intercept exactly zero and avoids
introducing a sensor-count-dependent bias when transferring from M24 to X36.

## Control-plane cost and deployable feature sets

The simulator exposes all local posteriors to the topology callback, but this
central memory interface is not free communication in a real network.  V254
therefore separates two feature contracts and charges the corresponding
telemetry before making any end-to-end communication claim.

The primary `compact-node-32` contract advertises one fixed 32-byte record per
sensor.  It carries the node-level quantities needed for link reliability,
distance, cardinality, uncertainty, association/detection support, source
quality, receiver need and current full-LMB payload size; recent route use is
maintained by the controller.  Six label-pair features (existence gain/gap,
precision gain/gap, state discrepancy and active-label overlap) are disabled.
The route command costs a 16-byte header plus eight bytes per directed gateway.
Thus control cost grows as O(N+F) and is known before the action.

The `rich-active-label-64` ablation allows all 22 features, but charges a
16-byte header and a 64-byte packed record for every active label at every
sensor, plus the same route command.  On the existing M24 seed-1302 t=60 H=3
snapshot, the mean/max active-label counts are 15.33/16.  Compact control costs
2,496 B over the three pages, 0.558% of the 447,624 B V242 posterior traffic;
rich control costs 72,000 B, or 16.085%.  The rich option can therefore erase
the current communication saving even if its posterior route is good.  This
is a single-window accounting diagnostic, not a tracking result, but it fixes
the method order: compact additive ridge first; compact GNN only if justified;
rich features remain a cost-adjusted ablation rather than the default policy.

V252 and V253 did not charge this controller telemetry, so their byte columns
remain teacher/representation evidence.  Every V254 complete-window and
complete-episode comparison must add `controlAttemptedBytes` to the posterior
ledger before reporting communication gain.

## Edge scoring and exact projection

The encoder produces a contextual value `s_theta(i -> j)` for every physically
available gateway edge.  In the linear sentinel, this value is a shared readout
of the additive V254 edge contribution.  A later GNN, if authorized, may replace
the encoder but must retain the same additive readout contract.  For each
receiving formation `q`, first retain the best sender for every pair of incoming
formation-tree slot and receiver sensor.  A rectangular maximum-weight matching
then assigns distinct receiver sensors to all incoming slots.  The projected
gateway assignment maximizes

`sum s_theta(i -> j) - lambda_switch * changed_arcs`

over the feasible set.  This local matching is exact for an additive edge
score because receiver-injectivity is the only coupling between incoming
formation arcs.  It replaces a fixed whole-graph candidate cap with edge
scoring plus polynomial-time projection, so increasing M24 to X36 or X48 does
not silently discard later formations.

Tie handling cannot use numeric sensor order without breaking equivariance.
If the optimal matching is not unique, the projector keeps the V242 reference
when it is one of the optima and otherwise abstains to the complete V242
assignment.  A non-reference assignment is therefore emitted only for a
unique optimum under the observable score and switching penalty.

The ordinary V242 assignment is always included as the reference action.  Let
`Delta_hat` be the predicted robust advantage of the projected assignment over
that reference.  V254 applies the learned assignment only when its calibrated
lower confidence score is positive; otherwise it abstains and uses V242.
The runtime path is therefore explicitly split into four functions: causal
edge representation, a shared edge readout, exact matching projection and a
thresholded policy wrapper.  A model failure is an invalid experiment rather
than a silent route change; an uncertain but valid prediction is an ordinary
V242 fallback.

## Properties supplied by construction

For every accepted action, independent of prediction accuracy:

1. **Physical feasibility.** Every selected cross-formation message is an edge
   of the current physical graph.
2. **Strong connectivity.** Each formation retains its directed local cycle,
   and the formation graph remains a bidirectional spanning tree.  Therefore
   every sensor can reach every other sensor.
3. **Fixed message count.** With `N` sensors and `F` formations, the projected
   graph contains exactly `N + 2(F-1)` directed messages per step.
4. **Relabeling equivariance.** Relabeling sensors or formations and applying
   the same relabeling to inputs produces the correspondingly relabeled output
   adjacency.
5. **Causal deployment.** The selector consumes only the current posterior,
   current physical graph and past routing history.

These are topology and communication-count guarantees.  They do not by
themselves guarantee lower payload bytes or better tracking; those claims must
come from paired outcomes.  Attempted payload bytes remain an explicit term in
the learned utility and in every evaluation gate.

## Learning target

Short paired continuations provide a robust assignment-level utility

`u = min(g_E, g_R, g_C, g_B, g_FE + 2, g_FR + 2)`,

where the gains cover network E-OSPA, RMSE, consistency, attempted bytes and
the two weakest-formation metrics.  The V242 reference maps to zero.  Training
uses differences between each action and the reference, so the model learns
whether a topology change is worthwhile rather than merely predicting the
absolute difficulty of the scene.

If V253 establishes cross-seed signal, the next model is an edge-additive ridge
sentinel trained on complete-assignment feature differences.  This is a
necessary bridge between pooled learnability and deployable projection.  Only
if the additive sentinel preserves safe selections yet leaves systematic
ranking error should a nonlinear encoder be compared using a structured
ranking loss over all actions from the same time window.  A one-sided
calibration residual from independent scene seeds supplies the abstention
margin.  With enough independent calibration seeds, this can be upgraded to a
block-conformal lower bound; the current three-seed V252 study is too small for
such a coverage claim.

## Evidence sequence

1. **M24 representation screen.** Use seeds 1302/1303 for model selection and
   seed 1304 once as the development holdout.  Continue only if the frozen
   selector improves all aggregate core metrics and attempted bytes without
   exceeding the 2% formation-tail tolerance.
2. **Untouched M24 episode.** Freeze all features, weights, thresholds and
   projection rules before opening seed 1305.  Report the fixed-tree baseline,
   V242 and the learned gateway policy together.
3. **No-retuning X36 transfer.** Apply the identical encoder, normalization,
   abstention rule and projector to
   `x36-formation-fov-temporal-coupled-formation-braid`.  A new X36 seed set
   must pass independently; M24 success cannot compensate for X36 failure.
4. **Non-radial scene transfer.** After scale transfer, evaluate at least
   crossing, merge-split and curved-corridor formation-FoV presets with the
   same 120-degree, range-limited sensor hardware.  These tests address scene
   geometry rather than provide extra tuning data.

Every full-episode table reports E-OSPA, OSPA, RMSE, inter-formation
consistency, actual attempted bytes, messages per step, weakest node,
weakest formation and the predeclared handover tail.  The main Lark document
keeps the best completed result even when it remains below the paper gate;
failed or incomplete candidates stay in repository evidence only.

## Stop conditions

- If V252 has little robust-positive action support, expand the feasible edge
  assignment space before changing the predictor.
- If V252 has support but V253 cannot transfer across seeds, diagnose
  representation and nonlinearity; do not claim that a larger GNN alone solves
  the problem.
- If the frozen M24 policy passes but X36 fails, inspect scale-normalization and
  the X36 physical/visibility regime without tuning on the X36 outcome.
- Only a policy that independently improves M24 and X36 tracking, consistency
  and communication proceeds to multistyle paper experiments.
