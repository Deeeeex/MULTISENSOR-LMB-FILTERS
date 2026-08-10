# V85 formation-level handover bundle

## Problem corrected

V84 finds genuine current information-owner transitions at both M24 and X36,
but its executable action replaces only one `0.05` sender at one receiver.  At
the primary anchors this produces only `+0.0566%` and `+0.0004%` H=3 mean
tracking improvement.  The local KLA counterfactual is valid; the action is
too sparse in the time-expanded network.

## Action unit

V85 retains the V84 edge conditions but applies materiality at the level that
will actually be executed.  For every residual receiver row, alternatives
must come from a different formation than the incumbent and have positive
current sender novelty.  The receiver-first mixture-aware KLA counterfactual
then keeps the best safe positive replacement for that row.  Distinct rows are
single-round separable, so all qualifying rows in one receiver formation can
be composed without approximating their individual KLA updates.

The formation bundle must jointly satisfy:

- at least two changed receiver rows;
- at least `1%` aggregate receiver-formation KLA net improvement;
- at least `0.5%` aggregate reliability-weighted sender novelty;
- no supported downward extraction crossing and aggregate harm no greater
  than aggregate gain;
- current physicality, exact directed-message parity, per-row message and
  weight-multiset parity, and rolling-B3 for action--reference--reference.

The small `0.01%` edge-novelty prefilter only avoids evaluating candidates
with effectively zero new support.  It is not the action gate; the unchanged
`0.5%` threshold is applied after receiver rows are aggregated.

## First source gate

Only the already opened primary V84 anchors are scored first: M24 t=104 and
X36 t=112, seed 41.  V85 may proceed to paired tracking only if both scales
produce an executable multi-receiver bundle under the same thresholds.  A
failure returns to action design; thresholds are not tuned against tracking.

Truth, future measurements, route execution, tracking outcomes, secondary
anchors, and model training remain closed during this source calculation.
