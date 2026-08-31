# V208 formation-graph action-value controller

## Decision

The next controller will not learn a scalar edge score or imitate the old
one-hop label selector.  It will learn finite-horizon values for a small set
of semantically different actions on a variable-size formation graph, while
hard support and communication constraints remain deterministic.

This decision follows from the current mechanism results.  Dynamic posterior
withholding can save communication, but useful interventions fall into two
qualitatively different classes:

1. a supported label can become spatially inaccurate and can be repaired by
   one complete-label residual KLA update; and
2. a diffuse set or route state can require a full-formation release, but a
   zero-support label by itself does not prove that release is useful.  V207
   shows that such a gap may predate the intervention and that a late full
   release can worsen both tracking and communication.

The useful action is therefore state- and mode-dependent.  A single learned
edge weight, a maximum-risk threshold, or a receiver-wise classifier cannot
represent the observed action family.

## Online action space

For every formation and page, the controller considers only four action
types:

| Action | When it is meaningful | Payload |
|:--|:--|:--|
| No-op | No intervention has positive conservative value | None |
| Withhold | The base dynamic route can omit a harmful or redundant posterior | Base route decision |
| Full-formation release | A withheld formation has positive predicted diffuse set/route restoration value | Ordinary full posterior already used by the baseline |
| Supported-label repair | Receiver support exists but a source has useful precision or observation information | One complete Bernoulli GM label |

The label repair retains mixture structure through the repository's
componentwise powered-GM KLA approximation.  It is not allowed to stand in
for exact arbitrary-mixture KLA, and it is not used when the receiving
formation has zero support for the label.

## Variable-size graph representation

At time \(t\), one node represents one formation.  All features are computed
from current or past information and are normalized so the same parameters
can be used for M24, X36, and later X48.

**Node features** summarize active-label count, existence mass, uncertainty,
observation-supported set-entry risk, formation-wide support gaps, gap dwell,
whether a gap predates the current intervention, recent withholding/release
history, and remaining communication credit.

**Directed edge features** summarize current physical reachability, delivery
probability, geometric distance normalized by range, FoV overlap, posterior
agreement, cross-formation label support, and the bytes required by each
candidate payload.

**Candidate features** reuse the truth-free one-hop label features already
implemented in `computeObservableOneHopLabelActionFeatures`, augmented with
action type, formation coverage, bounded covariance log-ratio, support
fraction, support-gap dwell, and payload cost.  Absolute sensor IDs, formation
IDs, label keys, time indices, truth, future measurements, and future link
draws are excluded.

Two shared message-passing layers aggregate neighboring formation evidence.
An action head combines the receiver embedding, source embedding, graph-level
summary, and candidate features.  Shared parameters and permutation-invariant
sum/mean/max aggregation make the score independent of formation numbering
and allow the graph size to change without retraining the architecture.

## Learning target and decision rule

The model predicts a vector rather than a single reward:

\[
(\Delta E,\;\Delta R,\;\Delta C,\;\Delta B),
\]

where the entries are finite-horizon improvements in E-OSPA, RMSE, consensus
error, and attempted bytes relative to the same-state no-op continuation.
It also predicts whether the action causes a material sensor- or
formation-level regression.  Initialization ensembles provide a conservative
lower bound for every output.

Selection is not based on a brittle weighted average.  Candidates must first
have positive lower-confidence aggregate tracking and consensus value, fit the
remaining byte credit, and pass a learned tail-risk screen.  Among admissible
candidates, the controller maximizes the minimum normalized improvement across
the four objectives.  Tail metrics remain penalties and reported diagnostics,
not an all-formations-must-be-strictly-positive veto; the latter is retained
only as a strong-evidence label for experiments.

## Deterministic safety projection

Learning ranks useful choices but cannot override the following rules:

1. retain the base route's required connectivity and physical-reachability
   constraints;
2. never exceed the current attempted-byte credit or the one-action-per-page
   cap;
3. never apply label KLA to a formation-wide zero-support label;
4. treat a persistent support gap as an action feature, not as an automatic
   release: V207 shows that broad external support does not establish receiver
   relevance or positive release value;
5. apply cooldown and hysteresis so release and withholding cannot oscillate;
6. use no-op whenever all conservative values are non-positive or the state is
   outside the training support.

Support-gap dwell is causal context: the first page records a possible
transient and later pages establish persistence.  Persistence alone never
overrides the value and tail-risk screens.

## Data and validation plan

Training examples are counterfactual action values, not labels copied from one
opened schedule.

1. Generate dense immediate targets for all four action types on development
   trajectories from radial, convoy, and relay scenes at M24 and X36.
2. Add short recursive targets on high-ambiguity states and states actually
   visited by the current policy, so the model learns delayed benefits and
   self-induced distribution shift.
3. Split by complete scene-seed trajectory.  No row, time, or receiver from a
   held-out trajectory may enter training or calibration.
4. Freeze features, model, thresholds, and safety projection before opening
   the held-out M24/X36 seeds.
5. Only after both scales transfer to convoy and relay without retuning, open
   crossing, merge-split, curved-corridor, and finally X48 as stress and scale
   extrapolation tests.

The first implementation reuses the repository's pure-Octave graph stack:
`computeTrackingAlignedFormationGraphFeatures` supplies variable-size
formation features, while the two-round pooling pattern in
`scoreLabelSetMessagePassingPolicyModel` supplies the equivariant message
passing and the existing Adam code supplies training.  Only the action block
and four-output heads are new.  This avoids a new runtime dependency and keeps
training and deployment numerically aligned.

## Promotion criteria

The main paper claim requires paired, held-out improvement over static routing
at both M24 and X36 in mean E-OSPA, RMSE, consensus, and attempted bytes.  The
current strict weakest-formation gate is reported as an additional strength
indicator, not the sole definition of usefulness.  Until held-out evaluation
is complete, teacher runs are recorded as mechanism-level bests and learned
controllers as policy-level bests in separate tables.
