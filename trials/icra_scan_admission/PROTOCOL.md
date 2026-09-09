# Current-scan support for negative evidence

The first three admission families and their external V2X-Real validation
results have been inspected. The selected spatial projection changed mean
development OSPA by only -0.00210 m and did not improve the external
two-condition mean. The external five segments are now development data
for this question. Preserve their original frozen evaluation unchanged.

The post-outcome detector diagnostic uses the same 12 m labeling cutoff:
pooled in-support detection recall is 0.9110 on all 43 V2V4Real segments
and 0.5313 on five V2X-Real segments. All trackers still use nominal pD=0.9.
This motivates examining the support assigned to an additional miss ratio.
It does not establish a visibility model or identify the cause of every
tracking error. Local filtering, detections, calibration, domain, radio
draws and Gaussian ratio curvature rules remain fixed.

For each source and current scan, let a_j be the current association mass,
r_j^- the predicted existence and o_j indicate current sensing opportunity.
Define p_hat = min(pD, sum(w_j a_j o_j) / sum(w_j o_j)), with p_hat=pD
when the denominator is zero. Compare w_j=r_j^- and w_j=(r_j^-)^2.
Replace only negative innovation support by
(1-a_j) p_hat / (2-p_hat) on current opportunities; retain zero support
otherwise. Positive support and the source-level/aggregate Gaussian guard
are unchanged. Use the existing negative-support packet field, with no
new transmitted field, fitted constant, smoothing parameter or future data.
The association statistic is observable but depends on the local model;
do not identify it with true per-target detection probability.

First reconstruct the complete original nine V2V4Real development inputs
and all five now-inspected V2X-Real inputs. Check original set-extraction
parity, report p_hat distributions and compare fixed-input alternate
outputs including miss/false/localization costs. Advance a candidate only
if its two-condition mean OSPA strictly improves on the original in both
datasets at these fixed inputs. Select the lowest nine-sequence mean,
with ties favoring exponent one. If neither qualifies, stop this family.
The screen does not establish full-recursion accuracy.

If a candidate qualifies, run its full recursion on all nine original
development sequences and the five now-inspected external segments. Keep
both radio conditions and all outcomes. Require an improvement in the
two-condition mean on both before extending to the remaining V2V4Real
segments or a new external cohort. A new external check must use recordings
not used here, with its input roster and algorithm frozen before scores.
The already-inspected five-segment cohort is never labeled as held out for
this family.
