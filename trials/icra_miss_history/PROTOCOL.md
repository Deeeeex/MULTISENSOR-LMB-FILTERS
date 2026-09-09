# Persistent missed-evidence admission

Start from `421f47aa`, after the archived 19-segment V2X diagnosis. The
previous goal turn produced a verified diagnosis and is classified as
progress. The performance-improvement goal remains open.

The failure case motivates a per-source, per-label history rule. This is
different from the failed current-scan global association-rate estimators.
Only the extra negative current-ratio exponent changes. Keep positive
admission, local pD=0.9, detections, calibration, prediction, matching,
curvature rejection, aggregate fallback, density normalization, extraction,
radio draws and the 352-byte Gaussian component packet unchanged.

## Frozen candidates

Let a_t be the executed current detection-association mass and h be the
previous soft consecutive-miss length for the same local source label.
Use h=0 after a missing local frame. A frame outside nominal sensing support
resets h to zero. Otherwise update h_new=(1-a_t)(h+1). This source-private
state is computed before communication and does not depend on delivery.

For nominal p in (0,1), choose the fixed prior Beta(p/(1-p), 1). This has
mean p and one pseudo missed observation; its concentration is a declared
design choice, not learned calibration. After h complete misses the next
predictive detection probability is p_eff=alpha/(alpha+1+h). Define
rho=log(1-p_eff)/log(1-p), with exactly rho=1 when h=0. The main candidate
`marked_gaussian_evidence_miss_history` multiplies the original negative
support (1-a_t)p/(2-p) by rho. This has an exact Beta-Bernoulli interpretation
for an uninterrupted run of complete misses. Fractional association masses
and reset on renewed support define a heuristic extension, not a Bayesian
estimate of true visibility. Both scalar and spatial parts of the admitted
negative ratio use this exponent through the unchanged GCE implementation.

The mechanism control `marked_gaussian_evidence_miss_half` always multiplies
negative support by 0.5 on a sensing opportunity. It checks whether temporal
conditioning helps beyond a uniform decrease. Neither constant is selected
from outcomes. The original GCE is rerun in the two preflight cases for exact
parity. Every candidate maintains its own full recursive trajectory.

Beta models for unknown detection probability already exist, including
[Vo et al., Multi-Bernoulli Filtering with Unknown Clutter Intensity and
Sensor Field-of-View](https://ba-ngu.vo-au.com/vo/VVHM_CISS11.pdf) and
[G. Li, Multi-object Tracking in Unknown Detection Probability with the
PMBM Filter](https://arxiv.org/abs/1907.01599). This experiment does not claim
that model as new, or implement their augmented-state filters.

## Evaluation and stopping

Before candidate outcomes, freeze both preflight cases: original development
0000 for baseline parity, and exposed v2xt_0001 for the diagnosed collapse.
The second case is a mechanism check and cannot select a method or parameter.
Freeze the remaining eight original V2V development segments and all five
earlier V2X segments as the screen. Together with preflight 0000, that covers
all nine original V2V development segments. Keep both link conditions.

Advance the history candidate only if its mean of reliable/intermittent
sequence-macro OSPA is strictly lower than original GCE in both the nine-
segment V2V group and five-segment V2X group. Report both links separately,
all per-segment changes, miss/false/localization components, output counts,
wire bytes, and the fixed-half control. No result-dependent subset exclusion.
If this gate fails, stop this exact family rather than tune its prior.

If it passes, evaluate all remaining 34 V2V segments and all 14 additional
V2X segments with the same implementation. All 19 V2X segments and all 43
V2V segments are now exposed; none is called a new independent test. A later
prospective evaluation requires a separately frozen, genuinely unused input
roster. Do not promote a method to the paper from the screen alone.

## Verification

Check complete-miss discount, first-miss equality, renewed-support reset,
fractional masses, missing-frame reset, no-opportunity reset, source-private
state, empty inputs, actual packet fields, and baseline recursive parity.
Independently reconstruct the history from executed local updates, the
positive marks from raw association weights, all Gaussian and existence
equations, extraction, observation crop, radio draws and physical bytes.
Keep all native exits, source/input hashes, original failures and repairs.
