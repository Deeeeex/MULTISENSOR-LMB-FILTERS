# V202 mode-gated residual fusion design

## Research objective

The method must improve E-OSPA, position RMSE, network consensus, and
attempted communication together on both M24 and X36.  Dynamic routing alone
already saves bytes and improves set-level accuracy, but it can accumulate
localized posterior errors.  Sending a complete formation posterior whenever
such an error is suspected is too coarse: useful and conflicting labels are
restored together, and recursively useful actions can interfere.

The first-principles correction is to keep the graph decision and the payload
decision separate.  A deterministic base carrier protects the effective KLA
information-flow graph.  Communication saved by that carrier becomes a
conserved residual budget.  The residual controller spends that budget only
on the smallest posterior object capable of addressing the observed failure.

## Non-negotiable invariants

1. **Protected information flow.**  The residual controller may add payloads
   but may not remove an edge retained by the rolling-connected base carrier.
2. **Communication conservation.**  Synopsis, request, response, route, and
   commit bytes are charged before an action is admitted.  A locked fraction
   of the base saving remains unspendable, so cumulative attempted bytes stay
   below the static full-payload reference independently of learned scores.
3. **Mixture-aware payload.**  A selected label carries its complete
   Bernoulli Gaussian-mixture density.  Moment summaries are used only for
   candidate scoring, never as a substitute for the fused spatial density.
4. **Atomic formation action.**  One source-label payload is either available
   to every receiver in the selected formation or applied to none.  Residual
   label KLA preserves receiver-specific evidence, so the resulting densities
   need not be identical.
5. **Explicit abstention.**  No-op is a first-class action.  A positive
   current risk proxy alone cannot force communication.
6. **Permutation and scale invariance.**  Absolute time, seed, formation ID,
   sensor ID, and numeric label values are routing keys only, not policy
   features.

## Action modes

| Mode | Observable failure | Payload and update | Current evidence |
|:--|:--|:--|:--|
| No-op | The base carrier remains adequate or value is uncertain | No residual payload | Required fallback |
| Observation handover | Receivers retain an active but weak, stale, or poorly observed label while a reachable source has strong current evidence | One complete source label, formation multicast, residual label KLA | X36 F2 is recursively useful; corrected F3 teacher is running |
| Precision refresh | Receiver and source describe a compatible label, but receiver uncertainty is much larger | One complete source label, formation multicast, residual label KLA | X36 F6 is recursively useful over H=3 |
| Set/cardinality restore | The error is diffuse across the extracted label set and cannot be attributed to one label | One budgeted complete-formation restore | M24 V197 mechanism is positive |

These modes are not four unrelated policies.  They share one base carrier,
one communication ledger, and one constrained projector.  The mode controls
candidate construction and payload granularity; the projector controls
whether the action can be executed safely and economically.

## Causal decision sequence

### 1. Build a shared light synopsis

Every sensor packs current local and fused metadata for all active labels
(`r >= 0.01`) once per page.  The synopsis exposes existence, evidence and
association support, observation opportunity, position moment and trace, and
complete-payload size.  It contains neither Gaussian-mixture components nor
target truth.

### 2. Construct all executable candidates

For every formation, enumerate reachable source-label actions and the
formation-level set/cardinality restore action.  Low receiver existence is
not a universal rejection rule: it is precisely the state in which an
observation-handover action may be valuable.  Physical reach, peer support,
atomic delivery, and positive communication credit remain hard constraints.

### 3. Classify the failure mode

The initial classifier should be an interpretable, shared shallow model or
monotone score, not a GNN.  It consumes symmetric formation aggregates and
bounded label features:

- receiver/source existence and log-odds gap;
- bounded log position-trace ratio rather than the unbounded raw trace
  difference;
- covariance-normalized spatial compatibility;
- source evidence and observation opportunity;
- receiver observation deficit and temporal support dwell;
- minimum and median Bernoulli Bayes-risk reduction;
- top-versus-runner-up risk dominance;
- peer agreement and fraction of formation receivers supporting the label;
- set-cardinality dispersion and unsupported-set-entry risk; and
- payload cost normalized by spendable communication credit.

Numeric identities and future outcomes are excluded.  Mode probabilities are
estimated first; finite-horizon action value is ranked only within a mode.
This prevents a high precision-refresh score from being compared directly to
an observation-handover score with incompatible semantics.

### 4. Estimate finite-horizon value and project

Teacher targets remain a vector rather than one hand-tuned scalar: horizon
E-OSPA gain, RMSE gain, consensus gain, weakest-formation change, cardinality
readout change, and charged attempted bytes.  The projector admits at most the
registered number of actions, keeps a no-op option, and requires calibrated
nonnegative lower bounds for every mandatory tracking objective.  Static
full-payload routing remains the conservative fallback when the base carrier
itself cannot be certified.

## Why a single maximum-risk rule is insufficient

At X36 `t=72`, the verified useful F2 candidate has minimum causal risk
reduction `0.3144`, whereas an unresolved F6 precision-refresh candidate has
`0.4987`.  Both have high source existence, evidence, and observation
opportunity.  A global maximum-risk rule therefore chooses F6 before F2.
Conversely, F3's MAP-sensitive handover label has receiver existence near
`0.32`, so a universal `r >= 0.5` gate removes it before ranking.  One scalar
rule cannot express both cases without opened-scene threshold tuning.

The bounded F6 proxy first showed source 19, label `[7,5]` with `+5.056%`
task-risk and `+59.596%` position-risk proxy gain under residual KLA.  The
paired H=3 recursive teacher then confirmed that the action is useful rather
than a proxy artifact: relative to static, E-OSPA, RMSE, and consensus improve
by `3.579%`, `10.648%`, and `7.433%`, while attempted bytes remain `5.754%`
lower.  Relative to V99, RMSE improves by `11.239%`.

The implemented bounded mode transform separates the two opened candidates
without reading their identities: F2 produces handover/precision evidence
`0.1051 / 0.0040`, while F6 produces `0.0145 / 0.2686`.  These values classify
the opportunity type only; neither is an admission threshold or predicted
tracking return.

## Experiment sequence

1. Close the X36 mechanism space with the corrected F2+F3 H=8 teacher.
2. Test the budget-feasible F2 then F6 sequence before fitting a selector;
   this preserves the one-action-per-page deployable cap while exposing
   cross-mode recursion.
3. Build causal teacher rows from complete scene-seed trajectories; keep
   training, calibration, and validation trajectories disjoint.
4. Fit the shared shallow mode/value baseline and calibrate no-op.
5. Freeze the complete policy, then evaluate radial M24 and X36 against the
   paired static reference, V99 no-repair base, V197 full restore, and V187
   balanced development best.
6. Without retuning, run convoy and relay, followed by held-out merge-split
   and curved-corridor scenes.  Crossing remains a stress test.  A formation
   GNN is justified only if the shallow policy leaves a repeatable cross-scale
   interaction residual.

The primary development gate remains at least `5%` E-OSPA improvement and
`2%` RMSE improvement on each scale, nonnegative worst-sensor and
worst-formation changes, improved consensus, and positive fully charged
attempted-byte saving.  RMSE is reported together with E-OSPA and cardinality
readout so that dropping difficult tracks cannot masquerade as improvement.

## Evidence boundary

This file defines the next deployable-method hypothesis.  It does not claim
that the mode classifier, online selector, F3 action, or F6 action has passed
recursive, cross-seed, cross-scale, or cross-scene validation.  V187 remains
the current balanced development best until a fully charged paired arm
improves the joint record.
