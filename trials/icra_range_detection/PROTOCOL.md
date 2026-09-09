# Shared range-calibrated detection model: complete recursive comparison

Start from diagnostic archive `e8258ca7`. The single range-recall model in
`trials/icra_visibility_diagnostic/RANGE_RECALL_CALIBRATION.json` passed its
previously declared recording-held-out calibration check. No new fusion
trajectory has been generated for this model.

## One common observation model

Use the frozen logistic probability `sigmoid(a + b * distance/40)` inside the
original source observation support. Outside that support, detection
probability remains exactly zero. For each V2V development sequence, use the
existing fit that excluded its entire recording. For V2X, transfer the full
nine-sequence V2V fit unchanged. Use the fitted probability as recorded,
including values greater than 0.9 at close range; no clipping to the previous
nominal probability, curve tuning, history adaptation or V2X refit.

Apply this probability through the shared sensor-quality interface to every
local update and every original negative-support computation. GCE, No-age
KLA and Guarded Scalar each run their own complete trajectory under exactly
the same detection-probability model. The fitted-constant control uses the
same excluded-recording/full-fit calibration's constant probability for all
three methods. Original nominal-0.9 results remain separate references.

Measurement covariance, detections, objectness calibration, local filtering
algorithm, current-ego CV model, births, label association, GCE curvature and
aggregate guards, source opportunity definitions, Gaussian normalization,
output extraction and scoring remain the original implementations. This is
a sensor-model experiment; no pose correction or annotation edit is made.
No extra packet fields are required: GCE and Guarded Scalar use their existing
352-byte component payload, No-age its existing 216-byte component payload,
with the same headers, padded wire accounting and fixed radio draws.

Log every predicted component's source, original label, pre-update mean,
source distance and executed detection probability, including components
later pruned. Audit the original local update, scalar and full Gaussian
equations under this model. Preserve original versions and exact nominal
baseline parity; a modified measurement model must not be mistaken for a
new fusion-method advantage.

## Frozen execution and decision

The three base arms are `marked_gaussian_evidence`, `marked_lineage` and
`marked_gaussian_evidence_guarded_scalar`. Their `_range` and `_constant`
versions are the six new arms.

1. Preflight on full V2V 0000 and the separately declared v2xt_0001 mechanism
   case, both links, all three nominal arms and all six new arms: 36 outputs.
   Verify nominal recursion parity, common model routing, analytic local
   updates, probabilities, full Gaussian moments, extraction and bytes.
2. Run the remaining eight V2V development sequences and all five existing
   V2X validation sequences, both links, all six new arms: 156 outputs. Reuse
   preflight 0000 to form all nine V2V development sequences. Keep the separate
   v2xt_0001 mechanism case out of method selection. Re-score existing nominal
   references, verifying native inputs and hashes.
3. For each of the two screen cohorts separately, compute the sequence macro
   OSPA averaged over both link conditions. Advance range GCE only if it is
   strictly better than all four references: nominal GCE, constant-model GCE,
   range-model No-age KLA, and range-model Guarded Scalar. Also report each
   link separately, all segment results, all three constant controls, GOSPA
   localization/miss/false terms, and padded/raw/delivered communication.
4. If any cohort/reference gate fails, close this exact model experiment.
   Do not tune its curve, probability clipping, training folds, or a different
   per-scene model to rescue the result. If all gates pass, continue to the
   remaining complete V2V/V2X cohorts with matched controls before considering
   a manuscript change. All existing data are exposed development evidence;
   a new independent generalization claim still needs new recordings.

Freeze stage source manifests and calibration hashes before execution. Do
not replace a completed trajectory or omit an adverse sequence.
