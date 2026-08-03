# V40 conditional-preserving bundle suppression

## Decision from the v39 diagnostic

The strict v39 rule `R(candidate) <= R(reference)` is not retained as the
admission gate for entering a sparse state.  The first three radial M24
source-cache diagnostics contained zero feasible routes in the requested-safe
bank.  An all-single audit showed that this was partly a proposal-coverage
failure: seed 47 formation 1 was safe, saved two of 48 directed messages and
reduced the one-round disagreement by 0.058%, but its 1.258% protection score
was below the v38 2% activation threshold and it never entered the v39 bank.
Even after all safe singles are included, however, only one of the first three
cases contains a strict-feasible action.

This is consistent with earlier H=3 evidence.  The one-round disagreement
metric rewards immediate cross-formation homogenization, whereas the useful
v35/v38 mechanism temporarily suppresses an incompatible cross-formation
posterior to protect local label existence and later restores mixing.  V28
already found three actions whose one-round risk increased while realized H=3
consensus improved.  The metric remains useful for recovery ordering and as a
diagnostic; it is not a recursive tracking or consensus certificate.

These cache probes are development diagnostics only.  Their helper first
reconstructs a complete deterministic scenario and then exposes a sanitized
current context.  A formal truth-free preflight still requires a separately
frozen current-page manifest that never materializes truth or future inputs.

## Confounded counterfactual in v38

The registered reference receiver has weights

```text
self = 0.25, dominant = 0.70, residual = 0.05.
```

V38 removes a residual input by adding its 0.05 weight to self, yielding
`(0.30, 0.70)`.  The runtime missing-message rule is `renormalize`.  If the
same residual message is simply not delivered in the reference arm, the
effective remaining weights are instead

```text
(0.25, 0.70) / 0.95 = (0.2631579, 0.7368421).
```

The v38 comparison therefore changes both message availability and the
relative self-versus-dominant trust.  It is not a clean intervention on the
communication edge alone.

## V40 mechanism probe

V40 keeps the v38 reference topology, residual weight, physical constraint,
label-retention gates and rolling-B3 construction.  When a residual input is
removed, it proportionally renormalizes every remaining weight in that row.
For every delivery outcome in which the removed input is absent, candidate and
reference now have exactly the same inputs and effective weights.  The only
remaining intervention is whether the residual payload may arrive.

The first milestone evaluates the reference and every single-formation bundle
action.  It selects no runtime action and opens no tracking outcome.  The old
self-return action is scored beside each conditional action on the same
posterior and current link probabilities.

Before inspecting the conditional results, the four-cache radial development
gate is frozen as follows:

1. every changed receiver must pass exact missing-input weight equivalence;
2. all five previously requested-safe singles from seeds 41, 43 and 47 must
   have strictly lower one-round risk change than their v38 counterparts;
3. at least four of those five must remain safe and retain a protection score
   of at least 2%;
4. at least two of seeds 41, 43, 47 and 53 must contain a safe,
   positive-protection single with non-positive one-round risk change.

Failure stops V40 before any continuation or tracking experiment.  Passing
only establishes a cleaner and non-degenerate source-side mechanism.

## Theory direction after the mechanism gate

The intended paper-level certificate is windowed, reliability-aware KLA
contraction rather than dense-reference dominance at every step.  A
scale-aware horizon must cover at least the current formation-graph diameter;
the fixed B3 diagnostic is not claimed to be sufficient for X36.  The
Dobrushin/Hajnal coefficient of the expected effective mixing-product is the
primary candidate for a contraction statement.  A proper Bernoulli-RFS metric
can separately bound the local posterior perturbation at changed receivers.

Risk--communication scalarization may be used for practical selection, but it
will not be presented as the theoretical contribution unless its constants
are fixed independently of tracking outcomes.
