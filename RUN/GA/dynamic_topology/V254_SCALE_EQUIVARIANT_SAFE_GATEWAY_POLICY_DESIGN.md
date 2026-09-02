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

V253 is the linear screening baseline for this representation.  Its 47-,
167- and 327-dimensional pooled summaries test whether current observable
posterior and edge state contain cross-seed value signal before a graph model
is trained.

## Edge scoring and exact projection

The encoder produces a contextual value `s_theta(i -> j)` for every physically
available gateway edge.  For each receiving formation `q`, first retain the
best sender for every pair of incoming formation-tree slot and receiver
sensor.  A rectangular maximum-weight matching then assigns distinct receiver
sensors to all incoming slots.  The projected gateway assignment maximizes

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

The first nonlinear model, if V253 establishes cross-seed signal, should use a
structured ranking loss over all actions from the same time window.  A
one-sided calibration residual from independent scene seeds supplies the
abstention margin.  With enough independent calibration seeds, this can be
upgraded to a block-conformal lower bound; the current three-seed V252 study is
too small for such a coverage claim.

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
