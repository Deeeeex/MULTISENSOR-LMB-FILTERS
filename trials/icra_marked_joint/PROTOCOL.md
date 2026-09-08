# Marked local-innovation fusion: fixed development experiment

The complete absolute-ceiling main experiment, conservative control, graded
positive-recency round, and missing-label diagnostic are available before
this experiment. The graded rule failed its continuation check. Directly
omitting missing-label censors did not show stable fixed-input OSPA gains.
Neither route is advanced on the basis of favorable individual sequences.

An earlier experiment already tested joint local innovations without score
likelihoods and failed on the complete development cohort. Since the later
marked observation update substantially reduced false-target errors for all
fusion backends, this round tests the *interaction* between that shared marked
update and accumulation of current local innovations. It does not hide or
supersede the earlier failure, and does not claim that pooling current
likelihoods separately from a common prior is a new idea.

## Fixed methods

Use the unchanged `fuseJointEvidence.m` and `innovationLmbPacket.m` from the
earlier round. For each local Bernoulli, recompute in every local update

    delta_j = logit(r_j_after_local_update) - logit(r_j_predicted).

Let b be the unchanged eligible-source base weights, q the unchanged age
weights, and eta the spatial normalizer. If at least two eligible positive
weight sources represent the aligned label, the primary M-JE uses

    z_J = sum_j b_j (logit(r_j) - delta_j) + sum_j delta_j + log(eta).

The fixed secondary M-JE-R replaces b by q in the inherited part. A missing
eligible label is not assigned an invented innovation: with any censor or only
one eligible represented source, M-JE falls back to the unchanged No-age rule
and M-JE-R to the unchanged ER rule. Untouched-source exclusion and the
existing observable-absence rules remain in force. Spatial fusion uses the
same inputs and ordinary spatial weights.

The local update of *both* methods is the previously checked marked update,
using the fixed per-detection likelihood ratio. The primary is `marked_joint_evidence`
(M-JE), selected before any marked joint trajectory; the only secondary is
`marked_joint_evidence_recency`. The primary separates new evidence from
inherited evidence without adding a recency assumption to the inherited term.
The secondary cannot replace it based on this round's outcomes.

All detection likelihood fits, priors, births, association, dynamics, FoV,
pruning, MAP extraction and radio settings stay as in the shared port. No new
model, threshold, parameter fit or search is introduced. Each source sends
one current innovation scalar: 216 B/Bernoulli plus a 32 B header. ER preflight
uses its native 208 B payload. The scalar innovation is recomputed locally
before sending; received metadata cannot become an additional independent
source in the same fusion event.

The scalar common-prior, conditionally independent-evidence identity is a
fixture, not an exact Bayes claim for approximate LMB association updates or
correlated detectors. This construction may overcount correlated fresh errors,
and its nonlinear recursion can alter association and pruning. A positive
result must be attributed to the tested combination, not to a novel general
prior/likelihood-pooling principle.

## Execution and decision rule

Run all nine original development sequences, both radio conditions, both
new methods. Reuse the complete M-No-age, M-ER, M-CR, M-ECR-C and M-ECR-S
reference trajectories; independently rescore their outputs on common truth
instances. Compare marked versus earlier unmarked joint methods as a secondary
backend interaction diagnostic on these same nine sequences, using the
preserved earlier outputs rather than rerunning or selectively choosing them.

Before the full stage, rerun ER on development 0000 to check all trajectory,
metric and packet fields for exact parity. Innovation-specific records have a
different meaning from support-ceiling records and are checked analytically.
Each local update records predicted and updated probabilities, the resulting
innovation, source, label, time and current opportunity, enabling independent
verification of the innovation calculation and resets.

Continue to the complete 25 former reserved sequences only if the fixed M-JE
primary has lower sequence-macro OSPA than M-No-age and M-ER in both conditions,
and no higher OSPA than M-CR in both. The full nine-sequence stage is required
even if early results are unfavorable. On failure, retain all results and stop
expansion of this candidate. This continuation rule precedes preflight.

OSPA remains position-only p=2 with 12 m cutoff; report all sequence values,
paired 10,000-sequence bootstrap percentile intervals, missed/false GOSPA
squared costs, common-truth localization and native packet accounting. The
25-sequence follow-up is `seen_transfer`: all these real trajectories have
already informed development, and no fresh validation claim is made.

Source and previous-results hashes are recorded before the first preflight
trajectory. The successful preflight evidence and complete-stage driver are
fixed before full execution. Every sequence requires a real successful MATLAB
exit, the completion marker, and exactly four result files. Original source,
inputs, failed candidates, and the manuscript are preserved.
