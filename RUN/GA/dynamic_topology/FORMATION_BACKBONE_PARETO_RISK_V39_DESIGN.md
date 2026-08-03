# V39 reference-Pareto risk gate

## Why v38 needs another method version

The first frozen M24 source cache exposes a conflict that the v38 action
rule does not resolve.  At `m24-formation-fov / seed 41 / t=55`, v38
requests formation 3 because suppressing its two low-weight residual inputs
raises that formation's reference-relative expected cardinality by 7.02%.
The action passes the label-retention and rolling-B3 safety checks, but its
exact one-round network-disagreement score is 3.775% worse than the fixed
reference while saving only 2 of 48 attempted messages.

This is not a scene-generation failure.  The local cardinality-protection
score and the network-disagreement score measure different consequences of
the same action.  V38 uses the former to start a suspension and only uses a
relative disagreement-improvement test when releasing an existing
suspension.  It can therefore start an action that is locally protective but
globally adverse according to its own network surrogate.

The observation is source-only.  No tracking estimate or target truth was
opened, and it is not evidence that v39 improves tracking.

## V39 rule

V39 retains the v38 causal proposal family, protection thresholds, exact
existence-retention checks, physical-edge constraint and rolling-B3 reserve.
Instead of checking only the final v38 action, it forms a finite route bank
from the registered reference, the safe v38 incumbent, every requested safe
single-bundle action, and every safe staggered-release action that v38 has
already evaluated.  Identical adjacency-and-weight routes are deduplicated
exactly while their proposal provenance is merged.

V39 then solves the following bank-relative constrained selection problem:

```text
maximize    directed message saving
subject to  all v38 hard safety checks pass
            candidate one-round network risk <= reference network risk
tie break   lower network risk, fewer switches from the previous selected
            topology, then stable proposal order
```

The risk inequality is literal; no numerical tolerance can admit a route
above the reference.  Because the registered reference is always feasible,
the selected route is guaranteed to satisfy the current-step surrogate
constraint, and it maximizes communication saving within the evaluated
proposal bank.  This is a bank-relative Pareto projection, not an exhaustive
optimizer over every directed graph.

The comparison uses the same current posterior and current link-drop
probabilities as the v38 one-round score.  It does not use target truth,
future measurements, future link pages or realized link uniforms.  The
controller distinguishes three states: no sparse proposal, a selected sparse
proposal, and risk rejection of every sparse proposal.  Only the last state
counts as a reference fallback.

For the first cache, all available sparse proposals fail the reference-risk
constraint, so v39 rejects the v38 formation-3 action and returns the
48-message reference route.  The selected risk advantage is therefore
exactly zero instead of -3.775%.  This establishes the intended projection
behavior only; communication and tracking claims remain sealed.

## Exact incremental scoring

The original controller recomputed every receiver's delivery-outcome
distribution and every receiver-pair disagreement for every bundle
candidate.  A bundle action changes only the receiver rows whose incoming
residual edge or fusion weight changes.  V39 reuses the registered-reference
distribution and pair-risk entries for every unchanged receiver pair, and
recomputes all entries touching a changed receiver.

This is an exact same-context factorization, not an approximation:

- the changed receiver set is derived from exact adjacency-row or
  fusion-weight-row differences;
- unchanged receiver distributions are copied from the same-step reference;
- every pair containing at least one changed receiver is recomputed;
- the complete pair matrix is aggregated with the unchanged registered rule.

The reuse envelope binds the fusion-semantic posterior projection, model
fields actually consumed by fusion, the current link-reliability page,
fusion-semantic trigger fields, time, scoring configuration, reference route,
receiver distributions and pair matrix.  It deliberately excludes truth,
future link uniforms, function handles and unrelated posterior fields.

Synthetic full-versus-incremental tests require exact (`isequaln`) receiver
distributions, outcome counts, pair-risk matrices and aggregate risk.  The
property matrix covers no change, adjacency-only, weight-only and joint
changes, sub-`1e-15` weight changes, fixed and probabilistic delivery,
`mean/tail` aggregation, `renormalize/self` missing-message behavior and
fixed-seed randomized row replacements.  Stale posterior, link, time or
trigger reuse and malformed or mixed cache envelopes must fail closed.

The cache digest is an integrity envelope, not producer authentication.  The
incremental path therefore accepts only same-process reference details as a
runtime optimization.  Any persisted preflight artifact must be audited by a
complete recomputation before it supports a formal claim.

On the first real cache, the formation-3 candidate score is exactly equal
between full and incremental evaluation (`1.2504647281062391`).  Two current
machine diagnostics measured 11.0--14.8 seconds for full scoring and
1.44--1.99 seconds for incremental scoring (about 7.4--7.6x).  The complete
v39 controller still took 23.9--28.4 seconds because it must evaluate the
reference and the full proposal bank.  These wall times are diagnostic only
and are excluded from deterministic fingerprints.

## Evidence sequence

The existing 15-case fixed-reference source batch continues on its original
frozen worktree.  V39 does not alter that executable source or its cache
identity.

Before any tracking score can be opened:

1. all 15 reference caches and their preflight manifest must close;
2. v39 must complete a truth-free conditional-continuation preflight on all
   15 cases;
3. the preflight must full-recompute every formal proposal, verify exact
   equality with the incremental score, and freeze selected, attempted and
   delivered topology, fusion weights, full-posterior state hashes,
   communication accounting, causal attestations and the complete v39
   candidate-bank/projection trace;
4. independent AB/BA executions must reproduce each arm trace exactly;
5. a separate data-only permit must bind the two arms, scoring code, truth
   window and seed-block analysis rule.

The later paired estimand is the focus-window effect conditional on a shared
reference-generated predecision state.  It is not a claim about deploying
v39 from `t=1`, held-out generalization, X36 scale transfer or paper-level
validation.
