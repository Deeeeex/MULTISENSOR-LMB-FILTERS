# V157 positive-value reference-label oracle design

## Why V156 is insufficient

V156 shows a non-monotone trade-off. `K=1` gives the strongest mean X36
improvement, while larger uniform capacities progressively repair the worst
formation-time tail but over-correct other cells. No fixed `K` passes. The next
question is therefore label value, not label capacity:

> If a selector knew the true marginal tracking value of each complete-label
> edit, could a small variable-size set retain the V105 aggregate gain and
> remove every local tail failure?

## Frozen mechanism oracle

- Reuse the paired `x36-formation-fov`, seed `211`, `t=72`, `H=8` boundary,
  V105 carrier/protection schedule, V126 reference capture, and the same 36
  registered intervention cells.
- At each cell, compare the actual evolving working posterior with the paired
  static reference posterior.
- Enumerate complete GM Bernoulli label edits, including explicit tombstones.
- Greedily evaluate the exact current-step E-OSPA change of each remaining
  single-label edit using target truth. Apply the best edit only when its
  marginal E-OSPA reduction is strictly positive, then recompute marginal
  values. Stop when no positive edit remains or four labels have been applied.
- The selected capacity may be `0, 1, 2, 3, or 4` independently for every
  receiver-time cell.

Truth is used only to define the mechanism upper bound. V157 is not deployable,
does not validate generalization, and cannot be reported as the proposed
method.

## Communication accounting and gate

The reference capture contains complete labels of at most `1368` bytes under
the repository's full-mixture estimator. Charging one 32-byte header plus four
maximum-size labels at every one of the 36 cells gives a conservative transport
upper bound of `198144` bytes. The actual selector can only use fewer bytes.

V157 passes only if it satisfies all existing X36 gates after charging that
upper bound:

- mean and mature-window E-OSPA gain at least `+5%`;
- nonnegative worst-sensor, minimum-formation, every formation-time, F6-peer,
  window-consensus, and terminal-consensus gains;
- nonnegative adjusted communication saving and the registered graph checks.

## Decision after the outcome

- If V157 passes, label-value prediction has demonstrated headroom. The next
  step is to construct an observable teacher dataset and train a compact
  receiver-label scorer, while a deterministic projector enforces byte and
  time-expanded support constraints.
- If V157 fails, the sparse reference-label route is closed; increasing model
  capacity or adding a GNN would not repair the missing mechanism headroom.

As with V156, a below-gate result remains repository-only and is not added to
the main Lark document.

## Outcome and next method gate

V157 passed the registered X36 development gate.  Relative to the paired
static full-payload baseline, mean E-OSPA improved by `+6.522%`, the mature
window improved by at least `+6.145%`, every formation-time cell was
nonnegative, and window/terminal consensus improved by `+10.163%/+14.668%`.
After charging the frozen worst-case `198144`-byte reference-label payload,
the adjusted attempted-byte saving remained `+5.184%`.

Together with the failed fixed-capacity V156 sweep, this isolates the useful
mechanism more sharply: the X36 tail is not repaired by sending more labels
uniformly; it is repaired by applying only labels with positive receiver-time
value.  This is mechanism evidence, not a deployable result, because both the
static reference label and its truth-valued marginal score are privileged.

The next gate is therefore **causal source feasibility**, before any GNN is
trained.  We must determine whether the beneficial edits are tombstones or
complete labels that can be synthesized from runtime-available local/received
posteriors within the relevant deadline.  A scorer cannot make an unavailable
posterior appear.  Only if a causal source oracle retains the V157 gain will
we construct observable training features and learn label value; otherwise the
static-reference label-memory interpretation is rejected.
