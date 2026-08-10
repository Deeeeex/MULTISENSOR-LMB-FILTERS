# V71 receiver-domain transport projection design

## Decision

V70 shows that alternative transport, rather than further edge removal, is
the common M24/X36 opportunity. V71 turns those local nominations into one
actual current-round communication graph.

The key separation is:

- local receiver-domain net mass decides whether a formation has a material
  route opportunity without dilution by unrelated formations;
- a global deterministic projection decides which nominated formation
  bundles can coexist under the current graph constraints.

## Exact one-round composition

V68 evaluates a sender replacement in one receiver row while all other rows
remain at the reference. A label-wise KLA update at one receiver does not
read another receiver's fused result in the same round. Therefore individually
safe replacements compose exactly when no receiver appears in more than one
selected residual slot.

For each nominated receiver formation, V71 applies every positive safe V68
slot in that formation. Each slot replaces one incumbent cross-formation
sender with the best currently physical unused sender while retaining the
same 0.05 row weight. The action changes neither the number of messages in
that receiver row nor its multiset of fusion weights.

## Global projection

V71 enumerates subsets of nominated formations and maximizes the total
receiver-domain net mass. A subset is feasible only when:

1. every selected sender-receiver link is currently physical;
2. total and per-receiver message counts exactly match the reference;
3. every receiver retains the same positive fusion-weight multiset and row
   sum one;
4. every selected V68 slot has zero supported downward 0.5 crossing and
   supported harm no larger than transport gain;
5. the candidate followed by two reference rounds passes rolling-B3 sensor
   and formation strong connectivity.

The formation graph is allowed to change; requiring equality with the
reference formation graph would prohibit the route adaptation being studied.
Ties are resolved by larger total net mass, then fewer affected formations,
then lexicographically smaller physical formation identifiers. If no subset
is feasible, the projector returns the reference route.

## Frozen first anchors

The first source-only projection reads no tracking outcome:

| Scale | Time | V70 nominated transport formations |
|:--|--:|:--|
| M24 | 80 | 3 |
| X36 | 52 | 4, 5 |

Only if both anchors produce legal non-reference routes is paired tracking
authorized as the next experiment. Passing these opened anchors is method
development evidence, not a validation or generalization claim.
