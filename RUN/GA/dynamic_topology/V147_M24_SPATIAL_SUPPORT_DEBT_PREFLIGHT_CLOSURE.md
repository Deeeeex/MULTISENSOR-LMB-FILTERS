# V147 M24 spatial support-debt preflight closure

## Decision

V147 stops before tracking. The frozen M24 current state contains no
nonreference whole-posterior row action that simultaneously has positive
mixture-aware label value, preserves protected labels, keeps the sensor graph
strongly connected and does not increase communication. Consequently no M24
tracking outcome and no X36 state are opened.

This is a structural negative result, not a failed long run. It closes the
fixed whole-posterior row-reassignment action space under the registered
weights and message budget. It does not negate V142's cross-scale mechanism
finding that spatially supported missing label states carry useful estimation
information.

## Frozen M24 bank

- Source commit: `3b68475`.
- Preset / seed / anchor: `m24-formation-fov / 1601 / 95`.
- Reference route: `fixed-counter-clockwise`.
- Candidate sender rows evaluated: `48` dominant promotions and `48`
  residual replacements.
- Positive-net rows: `0` dominant and `1` residual.
- Rows passing label protection: `0` dominant and `1` residual.
- Feasible nonreference complete-graph actions: `0 / 7`.
- Tracking outcomes opened: `0`.
- X36 bank or outcomes opened: `0`.

The sole label-safe residual candidate changes receiver 20 from sender 3 to
sender 1. Its current virtual net value is `+0.220084` and it would reduce the
current payload ledger by `37,128` bytes. It nevertheless splits the
sensor-level graph into five strongly connected components. Keeping it would
violate the very effective-information-flow condition that motivated the
dynamic-topology project.

Promoting an alternative source into the 0.70 role is not a hidden solution:
all 48 screened dominant candidates have negative predicted net value, and
every one causes at least one protected downward decision crossing. The best
dominant net value is still `-3.4329`.

## Method implication

The constraints expose a real authority--safety conflict:

1. residual-weight replacement is label-safe but usually too weak and, in
   the only positive M24 row, destroys sensor connectivity;
2. dominant-weight replacement has enough authority but moves every label in
   the complete posterior together, damaging incumbent-supported labels; and
3. changing coverage count or pulse duration cannot create a feasible row
   that is absent from the current action set.

The next method must therefore change the decision object. The supported
direction is a budgeted receiver--sender--label policy: preserve a fixed
strongly connected posterior backbone, advertise compact current label
synopses, route only high-value full-GM label objects on residual capacity,
and use a deterministic byte/connectivity projection around a learned or
analytic value estimator. This is not the discarded full/light-equivalence
story: the synopsis is control metadata, while every selected estimation
payload remains a complete mixture-aware label density.

Before any GNN is trained, the new edge--label action family must expose at
least 5% paired M24 and X36 oracle headroom under the same no-regression and
communication gates. Failed candidates and this closure remain
repository-only.
