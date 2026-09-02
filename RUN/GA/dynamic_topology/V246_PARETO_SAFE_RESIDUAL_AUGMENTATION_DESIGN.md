# V246 Pareto-safe residual augmentation

## Why this is the next method

V242 supplies a causal, physically feasible, strongly connected formation
backbone with `N + 2(F-1)` directed messages.  Its weakness is also precise:
it removes every local residual input from the two-input V240 route and
returns that KLA mass to the receiver's self weight.  Always restoring all
of those inputs recovers V241, but also gives back the communication saving.

V246 therefore treats V242 as a guaranteed base graph and asks a marginal
question for each omitted input: does restoring this one ordinary KLA input
improve the current expected fused posterior enough to justify its bytes?
The policy does not infer value from graph centrality alone and does not
change the posterior payload or the corrected mixture-aware LMB-KLA rule.

## Candidate set and hard constraints

At time `t`, first construct the unchanged V240 route and V242 backbone.
The candidate set is exactly the local residual edges that V242 omitted from
V240.  Restoring a candidate also restores its registered V240 residual
weight by removing the same mass from receiver self weight.

This construction gives four invariants before any learned component:

1. Physical feasibility: every candidate already belongs to the current
   physical V240 route.
2. Strong connectivity: V242 is strongly connected and V246 only adds edges.
3. Fusion semantics: every selected row remains a convex KLA weight row and
   uses the same full, mixture-aware posterior payload.
4. Communication bound: at most one residual edge per formation and at most
   `ceil(F/2)` residual edges network-wide are selected.  Their estimated
   payload may consume at most 25% of the instantaneous byte gap between the
   V242 backbone and the full V240 route.

The last rule preserves at least 75% of V242's instantaneous saving relative
to the full two-input route even when posterior sizes differ across senders.

## Exact current-step value

For the current local posteriors and current link reliabilities, the existing
directed-routing evaluator enumerates every independent delivery outcome for
each receiver and executes the same heavy-payload, missing-weight-normalized,
mixture-aware LMB-KLA update as the controlled runtime.  It returns receiver
outcome distributions and expected posterior-summary disagreement.  The
evaluation is exact for this registered one-round model; it is not a
recursive tracking guarantee.

For a current route `G`, V246 records:

- `R(G)`: expected label-wise posterior task risk, including the receiver
  upper tail;
- `D(G)`: expected network posterior disagreement;
- `R_f(G)`: mean-tail posterior risk for every formation;
- `L(G',G)`: reference-relative label-existence loss when a candidate route
  `G'` is compared with `G`.

A residual edge is individually admissible only if, after adding it to V242,

`R(G+e) <= R(G)`, `D(G+e) <= D(G)`,

every formation satisfies `R_f(G+e) <= R_f(G)`.  The asymmetric
existence-retention guard additionally forbids a robust downward crossing:
a label with reference existence at least `0.60` may not fall below the
`0.50` reporting threshold.  This margin protects established support without
reintroducing the earlier zero-tolerance rule for every small receiver-level
change.  Among admissible edges, the policy ranks the smaller of the relative
task-risk and disagreement gains per added byte, then takes the best
formation-distinct proposal under both budgets.  Because two individually
safe receiver updates can interact in the network-disagreement term, the
joint proposal is evaluated once more.  If it fails any constraint, the
lowest-ranked edge is removed and the joint check is repeated until the
proposal passes or V242 is recovered.

## What can be proved

Let `G_0` be V242 and `G_*` the final route after joint validation and any
deterministic backoff.  The final check directly guarantees

`R(G_*) <= R(G_0)`, `D(G_*) <= D(G_0)`, and
`R_f(G_*) <= R_f(G_0)` for every formation.

No submodularity or independence assumption is used for this final guarantee.
Strong connectivity and physical feasibility are inherited from the candidate
construction, while message and estimated-byte bounds follow directly from
the two budgets.  These are current-step surrogate guarantees, not claims
about truth-level E-OSPA or future tracking.

## Stability and data-driven extension

The first implementation uses the exact evaluator as an online teacher on
every page.  Previously selected residual edges win only exact score ties and
are retained only while they remain admissible; unsafe edges are removed
immediately.  Thus stability comes from causal tie-breaking rather than a
forced multi-step hold.

For scale-up, a GNN may learn the teacher's edge ordering from label-support,
covariance, compatibility, visibility, link-persistence and byte features.
The learned score can propose a short list, but the deterministic V242
backbone, physical projection, communication budgets and exact Pareto guard
remain authoritative.  Thus learning reduces search cost rather than owning
connectivity or safety.

## Evidence sequence

1. Wait for V245 to determine whether task--topology coupling strengthens the
   full and minimum-backbone mechanisms.
2. On the opened coupled M24 seed, run V246 first as a shadow policy and report
   candidate counts, accepted-edge timing, value components, byte headroom and
   selection time.  Do not tune from truth metrics.
3. Freeze the quotas and guards, then run a paired coupled-M24 full episode
   against V242.  Promote it only if E-OSPA, RMSE and consistency improve while
   attempted bytes remain below the fixed-tree reference and no formation tail
   materially worsens.
4. Run the identical method on X36 before any GNN training.  Only after the
   analytic controller exposes a stable scale-normalized signal should its
   ranking be distilled.

V246 remains development evidence until unopened seeds and additional scene
styles confirm the same direction.
