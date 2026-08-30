# V190 hierarchical finite-horizon residual routing

## One-sentence contribution

V190 learns the finite-horizon value of complete-label multicast actions on a
variable-size formation graph, then projects a small action set onto the
existing rolling-connected carrier under a conserved communication-credit
budget and an explicit no-op option.

This framing keeps the paper contribution narrow.  The learned model decides
which residual information is worth sending; deterministic rules retain
responsibility for graph connectivity, atomic delivery and attempted-byte
accounting.

## Why the V188 action space is insufficient

The opened H=3 teachers establish three facts that a successor must explain:

1. Current-page value is not finite-horizon value.  The X36 formation-2
   action is slightly negative on its action page but improves E-OSPA, RMSE
   and consensus over the recursive window.
2. Geometric proximity is not an accuracy certificate.  A 150 m predicted
   position cutoff rejects the beneficial formation-1 action and accepts the
   mixed formation-3 action.
3. Formation allocation alone is not enough.  The F1+F3 set makes every
   aggregate X36 objective positive relative to static, yet one F2 label
   changes the binding formation RMSE gain only from -14.198% to -13.859%.

The last failure is structural.  Each X36 formation has 101--124 executable
common source-label candidates at t=72, but the V188 analytic proxy discards
all but one before any learned value model can act.  Training a formation
ranker on that collapsed action space cannot recover the missing labels.

The repeated-page teacher closes one ambiguity.  Applying the same F2
source-20 label `[19,16]` repair on all three H=3 pages improves E-OSPA over
V99 by only `0.099%`, while degrading RMSE and consensus by `0.369%` and
`1.164%`.  Its static-relative communication saving also falls to `5.039%`.
The binding failure is therefore not insufficient action frequency; it is
the chosen action and the missing finite-horizon abstention decision.

## Runtime action and transport semantics

A candidate action is

`a = (target formation, source sensor, semantic label, ingress sensor, route)`.

The source holds one complete mixture-aware Bernoulli density for the label.
The ingress sensor is physically reachable from the source.  The route is a
directed in-formation multicast tree rooted at the ingress sensor.  V190 does
not require the source to reach every receiver directly; this removes a
scale-limiting assumption of V188 and permits larger or elongated formations.

Delivery uses a two-phase rule.  The payload is transmitted and acknowledged
along the registered tree first.  Receivers apply the replacement only after
the coordinator has received every acknowledgement; otherwise every receiver
keeps the V99 posterior.  Attempted payload, acknowledgement and commit bytes
are charged even when the action aborts.  The first implementation may use
ideal delivery only as a charged teacher, but deployable evaluation must use
the sampled link outcomes and the atomic commit rule.

## Hierarchical decision model

### Stage 1: shared light synopsis

The existing 24-byte active-label record remains the coarse inventory.  It is
packed once per sensor and reused by all candidate construction and formation
features.  No full Gaussian-mixture component enters this stage.

### Stage 2: formation graph value

One node represents one formation.  Node features are the scale-normalized
17-feature V188 block: support debt, common-source opportunity, peer
agreement, receiver disagreement, candidate density, physical reach and
causal one-step history.  Edge features summarize current physical support,
delivery probability, carrier role, hop distance and residual byte cost.

A permutation-equivariant message-passing network outputs a finite-horizon
value embedding for every formation.  The same weights serve M24, X36 and
larger formation counts; neither absolute time, seed nor numeric formation ID
is a feature.

### Stage 3: diverse Top-K source-label bank

Within each formation, the coarse synopsis produces a union of causal
shortlists instead of one analytic winner.  Initial shortlist criteria are:

- minimum receiver Bayes-risk reduction;
- source evidence quality and precision gain;
- peer-supported rescue score;
- receiver disagreement;
- observation opportunity and label recency;
- low payload cost; and
- predicted motion disagreement.

The union is deduplicated by semantic source-label key and capped at K.  The
numeric label key is used only for routing and payload lookup, never as a
model feature.  A charged 64-byte rich motion synopsis is requested only for
the coarse Top-K.  A label-level ranker combines each rich candidate with the
formation embedding and predicts a value vector

`(E-OSPA gain, RMSE gain, consensus gain, weakest-receiver gain)`.

### Stage 4: budgeted action-set projection

The projector includes no-op as an ordinary candidate.  It selects at most
the registered number of formation-label actions while enforcing:

- available spendable credit after all synopsis and control costs;
- at most one label per target formation in the initial deployable version;
- a feasible atomic multicast route for every action;
- the frozen rolling sensor- and formation-connectivity contract; and
- a positive calibrated lower bound for each required tracking objective.

The teacher may temporarily allow two formations per page to identify
complementary actions.  Deployment remains capped at one until a paired
ablation shows that the larger set improves the weakest formation without
losing communication saving.

## What can be proved

### Communication conservation

Let S_t be the full-posterior reference attempted bytes, B_t the V99 base
attempted bytes after its own control traffic, R_t the total V190 synopsis,
route-control and payload spend, and rho the locked reserve fraction.  The
existing ledger exposes only `(1-rho) max(S_t-B_t, 0)` plus carried disposable
credit.  Because the projector cannot spend locked credit,

`sum_t (B_t + R_t) <= sum_t S_t - locked_saving_T`.

This property is independent of the learned scores and remains valid for an
arbitrarily bad model.

### Effective-graph connectivity

V190 adds residual payload routes but never deletes an edge retained by the
V99 carrier.  If the deterministic base projector certifies rolling strong
connectivity over window W, the union of base and residual effective edges is
also rolling strongly connected over W.  The learned model cannot override
this projection.

### Atomic repair

Under the two-phase commit contract, every receiver in a selected formation
either applies the same complete Bernoulli density or every receiver keeps its
base posterior.  Partial delivery therefore cannot create a mixed repaired
state inside a formation.  This statement concerns transport consistency,
not tracking accuracy.

### Calibrated abstention target

Tracking benefit is not deterministically guaranteed.  The intended
statistical claim is a trajectory-grouped risk bound for the complete policy,
not a per-action theorem.  A held-out calibration split chooses the no-op
threshold so that the upper confidence bound on the rate of admitted actions
with any negative required objective stays below a registered risk level.
This claim remains unauthorized until the exact calibration procedure and
independent trajectory count are frozen.

## Teacher targets and data splits

Every teacher starts from the same pre-action posterior and shares future
measurements, delivery uniforms and filter RNG with V99 and static.  Targets
remain separate rather than collapsed into one scalar:

- horizon mean E-OSPA gain;
- horizon mean position-RMSE gain;
- window and terminal consensus gain;
- weakest sensor and weakest formation gains;
- attempted and delivered bytes; and
- action abort or delivery state.

Whole scene-seed trajectories, not rows or time windows, define the splits.
The bounded order is:

1. radial M24/X36 action-space development;
2. convoy and relay development without threshold changes;
3. merge-split and curved-corridor heldout evaluation;
4. crossing stress evaluation; and
5. only then, larger formation counts and sensor counts.

The shallow baseline is a shared nonlinear ranker on the same candidate and
formation summaries.  The graph model is justified only if it improves
cross-scale ranking or weakest-formation decisions beyond that baseline.

## Decision gates

V190 is not promoted by a single bright X36 window.  The frozen radial gate
requires M24 and X36 independently to show, relative to the paired static
reference:

- at least 5% lower mean E-OSPA;
- at least 2% lower mean position RMSE;
- nonnegative weakest-sensor and weakest-formation changes;
- improved window consensus; and
- positive fully charged attempted-byte saving.

Below-gate teachers remain repository evidence.  The Lark current-best table
continues to show V187 until a new arm improves the joint record, even if that
arm has not yet passed the final multi-seed paper gate.

## Immediate implementation sequence

1. Treat repeated use of the current F2 top-one action as falsified.
2. Resolve and run a one-page H=3 teacher for the strongest distinct F2
   shortlist member, source 19 label `[13,12]`.
3. Run paired per-action H=3 teachers only on the smallest informative Top-K.
4. Fit the shallow hierarchical baseline and calibrate no-op.
5. Add the formation GNN only after a measured residual remains.
6. Freeze the method before opening new seeds and non-radial scenes.

No paper-facing tracking or generalization claim is authorized by this design
document alone.
