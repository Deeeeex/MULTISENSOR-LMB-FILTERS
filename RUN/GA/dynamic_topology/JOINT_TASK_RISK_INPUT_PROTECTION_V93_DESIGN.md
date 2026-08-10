# V93 exact joint-formation input-protection oracle

## Question

V92 showed that protecting one formation at a time is usually safe relative
to the matched static physical-tree route, but its network-wide improvement
is too small. V93 asks whether this is merely scale dilution or whether the
entire input-protection action family lacks enough headroom.

## Matched baseline

Every candidate is compared at the same cached posterior state, physical
graph, link probabilities and random seed with the V91 current physical-tree
reference. No candidate-specific baseline or threshold is allowed.

## Exact action bank

For `F` formations, V93 evaluates all `2^F-1` nonempty subsets: 15 actions in
M24 and 63 actions in X36. An action suppresses only the registered 0.05
cross-formation residual inputs entering the selected formations, returns the
removed weight to receiver self-weight, and then follows two reference-route
recovery steps. The 0.70 dominant local input is never removed.

Each formation changes receiver rows disjoint from every other formation.
Therefore the per-receiver delivery/fusion distributions computed for the
single-formation actions can be composed exactly for the one-round task risk
and existence-retention guard. The selected best subset is also recomputed by
the complete fusion-outcome enumerator; any mismatch aborts the scan.

## Frozen gate

Both M24 and X36 must have at least 10% actionable snapshots, median positive
task-risk gain of at least 0.25%, and peak safe gain of at least 1.0%. Worst
receiver and worst formation risk may not regress, and the V92 label-retention,
decision-threshold and rolling-B3 constraints remain unchanged.

Passing this gate authorizes a full M24 tracking comparison against the same
static route; it is not itself an E-OSPA or recursive-tracking claim. Failure
closes the protection-only family and moves method design to replacement of
the harmful residual sender rather than further suspension tuning.
