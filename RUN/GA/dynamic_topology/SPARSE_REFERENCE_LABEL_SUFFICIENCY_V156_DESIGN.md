# V156 sparse reference-label sufficiency design

## Decision question

V126 showed that replacing the complete V105 fused posterior with the paired
static posterior at 36 privileged node-time cells removes the X36 tail failures
while retaining the aggregate gain.  V156 asks a narrower question before any
deployable mechanism is designed:

> Does that recovery require a second complete posterior, or can it be carried
> by a small number of complete Bernoulli-label states?

This is a representational upper-bound experiment.  It does not yet specify how
an online policy discovers, stores, or routes the useful labels.

## Frozen paired boundary

- Scenario: `x36-formation-fov`, seed `211`, start `t=72`, horizon `H=8`.
- Working arm: the frozen V105 protection-only schedule, with its physical
  carrier, fusion weights, measurements, delivery uniforms, and RNG unchanged.
- Reference state: the paired static full-payload fused-posterior capture used
  by V126.
- Intervention cells: exactly the 36 formation-page cells registered by V126;
  no new cell is selected after looking at V156 outcomes.
- A label edit replaces the complete GM Bernoulli state for one label.  If the
  reference posterior does not contain a working label, the edit is an explicit
  tombstone for that label.  Moment-matched surrogates are not used.

## Fixed label ranking

At an intervention cell, labels are ranked from the current working posterior
and the privileged paired reference posterior.  The ordering is fixed as:

1. disagreement in MAP-set membership;
2. disagreement about crossing the filter existence threshold;
3. larger absolute existence-probability difference;
4. larger first-component position difference;
5. lexicographic label order as a deterministic tie break.

No target truth or future measurement is used by the ranking.  The paired
reference posterior is nevertheless counterfactual information, so every V156
arm remains an oracle and development-only.

## Capacity arms and causal execution

Run maximum edit capacities `K = {1, 2, 4, 8}`.  At each registered cell, the
selector edits at most `K` labels in the *actual evolving candidate posterior*;
therefore an earlier sparse correction is allowed to propagate and is not
silently overwritten by a precomputed V105 hybrid.  V105 and full V126 are the
two endpoints.

The preflight found that 20/36 cells have at most two MAP-label edits, but the
median material gap is six labels and the worst is seventeen.  Consequently,
`K=1/2` test genuine sparsity, `K=4` tests a still-compact correction, and `K=8`
is a capacity ceiling rather than a claim that the state difference is sparse.

## Gate and interpretation

A useful sparse capacity must, on the frozen paired X36 development case:

- retain at least `+5%` mean and mature-window E-OSPA gain over static routing;
- keep worst-sensor, minimum-formation, minimum formation-time, F6-peer, and
  consensus gains nonnegative;
- use fewer reference-label bytes than the communication headroom saved by
  V105 under the existing full-mixture payload estimator.

If `K<=4` passes, the next method hypothesis is a time-expanded, per-label
observation-support memory with deadline-aware forwarding.  If only `K=8`
passes, the mechanism is not yet compact enough and must not be promoted.  If
no sparse arm passes, the label-memory route is closed and the V126 improvement
remains a full-state rollback artifact.

Below-gate numeric outcomes stay in repository experiment records and are not
added to the main Lark document.
