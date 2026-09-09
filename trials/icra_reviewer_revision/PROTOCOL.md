# Reviewer revision protocol, 2026-09-09

Registered before any new tracking result in this revision. Base commit:
`9479f0d39958056464ab23ec07da94244abb5711`. The reviewed PDF and complete
review are identified in `SOURCE_REVIEW_CN.md`. Existing results remain intact.

## Frozen primary and interpretation

Keep the primary GCE algorithm, all gates, curvature tolerance, age rule,
birth/update/association/extraction settings, detector and score calibration
unchanged. Hash its current sources before new evaluation. No unfavorable
comparison is grounds for modifying GCE or selecting a replacement primary.
The contribution to test is selective admission of approximate current
Gaussian Bernoulli update ratios. Prior/likelihood separation and Bernoulli
normalization are established structure; the PSD test is an integrability
safeguard, not a statistical consistency or independence guarantee.

## R1: data independent of fusion development

Audit all previously accessed sequences, including the nine DMSTrack `val`
sequences, seven early train probes, and the remaining 25 train sequences.
The nine plus 25 reported sequences do not exhaust development exposure.
Do not relabel any of these as unseen. Check the official V2V4Real validation
split, which is distinct from DMSTrack's test-backed `val` release; acquire
public detections or perform inference with the unchanged released detector
if inputs are available. Establish source sequence identities, timestamps,
overlap, detector checkpoint and detector-training relationship before
tracking. Inspect labels only for input conversion and scoring, never for
algorithm or threshold selection. Freeze the exact new sequence list and
input hashes before producing any new tracking outcomes. Reuse the existing
full-nine calibration and positive prior without fitting on new sequences.

Run every available new sequence, both existing link conditions, with the
shared marked local frontend: No-age KLA, Scalar, Guarded Scalar, GCE without
curvature, GCE, and the development-selected Fixed Ratio. Add existing
principal matched-input baselines if their adapters pass parity. Report all
outcomes. A different detector, detector validation exposure, related route,
missing poses or unavailable new inputs must be disclosed separately;
none may be hidden by the label "independent test". If acquisition fails,
retain the limitation as unresolved while completing other valid revisions.

## R2: complete recursive 2 x 2 correction/curvature comparison

Add Guarded Scalar (GS): compute current qualification, beta, score/miss
gates, source curvature rejection and aggregate fallback exactly as GCE on
GS's own local inputs. Keep the original spatial pool p0, covariance and
log I0; set logit r = sum beta*z + sum kept_kappa*delta + log I0. Feed this
posterior into GS's own next prediction, association, births and extraction.
GS needs the ratio packet for its guard, so charge the same full 352-byte
Bernoulli packet as GCE. Do not claim equal cost to native 232-byte Scalar.

Run GS on all nine development and 25 seen-transfer sequences in both link
conditions. Reuse the frozen complete Scalar, no-curvature and GCE outputs
after provenance and independent rescoring checks. Do not replace the
existing fixed-input analysis with the recursive experiment: report their
different estimands explicitly. Include GCE-GS, GS-Scalar and the interaction
contrast of the two correction/curvature factors, with sequence-paired CIs.

## R3: fixed-strength ratio and controlled correlation

Fixed Ratio retains GCE's history beta, current-opportunity and represented-
label requirements, source curvature guard and aggregate fallback. Replace
both positive and negative score/miss support by lambda for current eligible
sources, giving raw_kappa = (1-beta)*lambda. Select lambda from {0.25,0.5,1}
using the lowest arithmetic mean of the two sequence-macro OSPA values on
the nine existing development sequences; exact ties prefer smaller lambda.
This baseline selection is allowed only on old development results. Record
all candidates before evaluating new data; do not alter the grid or primary.
Run the selected arm on the 25 seen-transfer and any genuinely new sequences.

Add a separate reproducible linear-Gaussian control with two measurements,
known cross-source correlation rho in {0,0.25,0.5,0.75,0.9}, frozen noise and
prior, and 10,000 samples per rho (seed 20260909). Compare pooled posterior,
unit-admission GCE and the oracle using the known joint covariance. Report
position RMSE, mean normalized squared error and nominal 95% ellipsoid
coverage. This demonstrates the limit of a PSD guard under correlated errors;
it does not certify consistency of the real tracker or replace real data.

## R4: motion frame and detection probability

Retrieve actual ego poses and map existing detections into a fixed planar
frame, or apply the equivalent known inter-frame rigid transform to every
predicted state and covariance. Preserve the same measurement/truth crop in
the current ego frame and the same noise, birth and fusion settings. Check
the coordinate adapter on stationary/moving point and covariance fixtures;
cross-check released relative source transforms against absolute poses.
Run No-age, Scalar, GS and GCE on all nine plus 25 sequences for which exact
poses can be matched, both conditions. No pose estimated from tracking truth.
Do not use relative inter-vehicle transforms as inter-frame ego motion.

Independently vary modeled pD in {0.7,0.8,0.9,0.95}, retaining the same
measurements, calibration, domain and clutter. Run the four arms on the
nine development sequences, both conditions; reuse the original pD=0.9
arms after parity. These are model-sensitivity checks, not retuning of GCE.

## Verification, reporting and manuscript integration

Require unit fixtures for GS normalization/guard parity, fixed strength,
no-current-opportunity and aggregate fallback; then exact full-recursion
parity for unchanged GCE and Scalar on original development 0000 in both
conditions (excluding runtime/new metadata). Record code hashes before
preflight, immutable output directories, subprocess exit codes and terminal
completion lines. Independently reconstruct estimates/OSPA from each saved
complete trajectory, retain every sequence and compute 10,000 paired
sequence-bootstrap resamples (seed 8301). Frames/nodes are not replicates.

Amend the main comparison and ablation tables using completed verified
cohorts, keeping negative results and the existing fixed-input uncertainty.
Report raw and modeled fragmented/control byte overhead relative to No-age
as well as savings relative to full GCE. Do not infer wireless latency,
capacity or energy from packet-length-independent delivery emulation.
Rewrite abstract/contributions/method interpretation/discussion to match
the results. Preserve exactly seven full body pages and one Ack+Ref page;
compile, check cross-references/citations, and visually inspect all pages.
Maintain a reviewer ledger with added experiments, evidence paths, text
changes and unresolved limitations, without claiming submission readiness.
