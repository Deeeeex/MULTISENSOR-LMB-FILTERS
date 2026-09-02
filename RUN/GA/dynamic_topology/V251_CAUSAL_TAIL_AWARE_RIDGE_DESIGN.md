# V251 causal tail-aware ridge design

## Method question

V250 proves that changing only the sensor-level embedding of the six
cross-formation messages can improve the localization tail while preserving
the 30-message backbone.  Its offline teacher, however, identifies the
reference-worst formation from realized H=3 RMSE.  That identity is a label,
not a deployable input.

The first causal alternative is to infer the worst formation from current
posterior Bayes risk, covariance or within-formation disagreement.  This is
not assumed valid: a formation can be confidently biased, internally coherent
and still have large matched-position RMSE.  V251 therefore audits those
proxies explicitly and learns candidate value from cross-formation gateway
features instead of feeding a truth-defined formation index to the model.

## Causal representation

Each V250 candidate contains the same directed formation tree and message
count.  V251 maps its six directed gateways to the existing truth-free routing
features and pools them without sensor or formation identifiers.  The 47
features comprise:

- the assignment mean of 20 current edge features: reliability, normalized
  distance, sender/receiver expected cardinality and cardinality variance,
  spatial variance, association confidence, detection mass, positive and
  negative existence/precision gaps, spatial discrepancy, label overlap,
  source quality advantage and receiver need;
- the mean change in those 20 quantities on gateway rows that differ from the
  V242 reference;
- the fraction of changed arcs, current overlap with the two-page causal route
  history and its change from the reference; and
- action-type indicators for reference, coordinated, receiver-local and
  single-arc candidates.

The 11-feature link-only subset keeps reliability, distance, route history,
change fraction and action type.  The 47-feature posterior-rich subset is the
actual ridge baseline.  Both are sensor-permutation invariant and
formation-label invariant.  Neither includes numeric identities, truth,
future geometry, future measurements, link uniforms or alternative-arm
outcomes.

## Teacher and ridge objectives

Truth supplies eight offline outcome heads for every completed H=3 arm:
network E-OSPA, RMSE and consistency gains; attempted-byte saving; minimum
formation E-OSPA and RMSE gains; and the E-OSPA/RMSE gains of the
reference-worst formation.  These heads are diagnostics, not deployment
features.

The primary linear baseline is pairwise ridge ranking.  For each anchor, the
tail-aware teacher candidate is paired against every alternative; the feature
difference is labelled as preferred and its reverse as nonpreferred.  This
removes the three-positive-versus-sixty-negative imbalance of direct
multi-head regression.  The capacity screen uses a small fixed numerical
regularizer `lambda=1e-6`; it is not a tuned deployment hyperparameter.

## Frozen interpretation gates

1. In-sample pairwise capacity must recover the teacher at all three anchors
   and the realized selected sequence must pass the existing V250 aggregate
   core, byte and 2% formation-regression gate.
2. Leave-one-anchor-out is evaluated without changing features or lambda.  It
   passes only when at least two held-out selections are tail-aligned and the
   aggregate realized sequence passes the same V250 gate.
3. Capacity failure means the causal representation must change before more
   H=3 compute.  Capacity success with leave-one-anchor-out failure means the
   representation can encode the teacher but three anchors are insufficient;
   independent M24 training anchors are then required before an online policy.
4. A GNN is not authorized by either result.  It may be compared only after a
   ridge policy is trained on expanded M24 data and evaluated on untouched
   trajectories, followed by no-retuning X36 transfer.

## Evidence boundary

V251 reuses the 63 completed arms from one M24 development seed.  In-sample
fit is a representation-capacity test, while leave-one-anchor-out contains
only three groups from the same trajectory.  Neither is a full-episode method,
an X36 result, a generalization result or a paper-level claim.  The main
current-best table must remain unchanged.
