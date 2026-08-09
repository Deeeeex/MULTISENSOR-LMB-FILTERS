# V70 receiver-domain normalized KLA routing opportunity

## First-principles defect

The current receiver computes each label existence in log-odds form as

`logit(r_fused,l) = log(eta_l) + sum_j w_j,l logit(r_j,l)`.

Replacing or withholding an incoming posterior therefore changes both the
weighted existence evidence and the spatial compatibility term `log(eta_l)`.
V65--V69 correctly evaluate that receiver counterfactual with the installed
fusion implementation.  The remaining scale defect is not in this local
counterfactual; it is in the denominator used to decide whether the local
action is material.

For one formation action, let `Delta M_g` be the receiver-supported existence
mass changed inside formation `g`.  V65 uses

`u_global(g) = Delta M_g / sum_h M_h`.

Appending unrelated formations leaves `Delta M_g` and the action unchanged,
but increases the denominator.  With `k` repeated modules, the score of the
same single local action falls as `1/k`.  V66 similarly divides finite-horizon
local decision influence by `N H`, and V68 divides one receiver-slot transport
gain by the whole-network reference mass.  A fixed threshold on these scores
cannot be a scale-consistent local routing rule.

## V70 decomposition

V70 separates an intensive local decision from an extensive network
allocation:

1. A formation-local quarantine utility subtracts supported useful-information
   loss from receiver-supported rescue, then divides the net mass only by the
   reference existence mass of the affected receiver formation.
2. A formation-local transport score converts each safe V68 slot gain back to
   existence mass, aggregates it over the affected formation, and divides by
   that same formation reference mass.
3. Supported downward crossings remain forbidden.  Quarantine must also keep
   useful-information loss no larger than rescued mass.
4. The local robust-margin exposure is normalized by the six affected
   receivers and the fixed three-step horizon, not by the total network size.
5. These scores only nominate actions.  A later deterministic projection must
   enforce the physical graph, fixed message budget, and rolling connectivity
   before any route is executed.

For affected receiver formation `g`, define

`M_g = sum_{i in g} sum_l r_ref(i,l)`

and let `Delta_g+` be receiver-supported rescue mass and `Delta_g-` be
cross-sender-supported loss mass under the current counterfactual.  The local
quarantine utility is

`u_q(g) = (Delta_g+ - Delta_g-) / M_g`.

Alternative transport uses the same form after aggregating safe receiver-slot
gain and harm inside `g`.  This common utility scale lets the policy compare
quarantine, alternative transport, and fallback without making network size
part of the local decision.

The 1% local net-utility threshold preserves the previous materiality scale but
changes its domain to match the action.  Requiring net utility, rather than
rescue alone, prevents a nominal 2% rescue with 2% supported loss from being
treated as beneficial.  The 0.10 local robust-margin threshold
is frozen from the five already opened V66 development rows: all four strong
radial events remain above it after local normalization, while the weak M24
convoy event remains below it.  No merge-split tracking outcome is read during
this design.

## Scale-consistency property

Consider repeating a local formation module while reserving backbone edges
outside the candidate action.  Each copy has the same local posteriors,
receiver reference mass, counterfactual rescue, and local decision exposure.
The V70 local score and action decision are therefore unchanged in every copy.
If the same action is applied to all copies, both total benefit and total
reference mass grow by the same factor, so the network-average effect is also
unchanged.  The focused unit test encodes the first part directly: appending
two unrelated formations halves the old global score from 1% to 0.5%, while
the V70 score remains 2% and the original local decision is preserved.

More formally, append any receiver-disjoint module whose posterior and links
do not enter the counterfactual for `g`.  Neither `Delta_g+`, `Delta_g-`, nor
`M_g` changes, hence `u_q(g)` is invariant.  For `k` identical copies, summing
all selected local mass changes gives `k(Delta+ - Delta-)`, while the network
reference mass becomes `kM`; their ratio is the original network-average
effect.  This is the replication property missing from a single-action score
normalized by the entire network.

The eventual global step is deliberately separate.  It should maximize the
sum of selected local mass utilities, `sum_g M_g u(g)`, subject to current
physical edges, the registered per-round message budget, mutual exclusion of
conflicting actions, and rolling three-step connectivity.  Local normalization
therefore does not relax a network constraint; it prevents unrelated network
mass from deciding whether a locally meaningful action may enter that
projection.

## Evidence boundary

V70 initially reuses only the frozen V69 M24/X36 current-state caches.  It may
report local action opportunities, but it cannot execute a route, read a
tracking outcome, train a model, or support a validation claim.  An executable
action requires a separately frozen global projection and paired tracking
protocol.
