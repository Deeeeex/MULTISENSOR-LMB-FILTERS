# Range recall check before any range-based fusion method

The box-ray cue did not identify the repaired missed target. Its diagnosed
negative source is about 39 m away, while the main V2X false-output source is
about 6 m away. Raw clouds also show a substantial alignment discrepancy in
the latter sample. This motivates checking the existing nominal detection
probability, not changing poses or removing annotations.

Before fitting any range model, fix the following diagnostic:

- Use all nine V2V development sequences, existing prepared measurements and
  truth, and the exact current 40 m source support. No V2X fit, tracking scores,
  history-candidate parameters, or candidate output labels enter the fit.
- A positive target-source frame is the existing squared-distance assignment
  within 2 m. Also report recall within 12 m descriptively, but do not fit or
  choose another model using that cutoff.
- Compare the nominal probability 0.9, a sequence-balanced fitted constant,
  and one monotone logistic model `sigmoid(a + b * distance/40)`, `b <= 0`.
  Use the existing calibration convention of regularization `0.001*b^2`.
  Do not sweep the penalty, distance transform, or label cutoff.
- For each excluded recording, exclude all its constituent sequences. Fit on
  the remaining recordings, giving each training sequence equal total weight.
  Report each excluded sequence and the nine-sequence macro log loss and
  Brier score. A smaller held-out macro log loss than the fitted constant is
  required even to consider this single range model for a later method screen.
- Fit one full-nine model only for an eventual unchanged transfer to V2X.
  Report probabilities at 5, 10, 20, 30 and 39 m. No fusion implementation or
  recursive performance claim is part of this diagnostic. All data remain
  exposed development data, with recording dependence reported.

This empirical recall averages localization error, occlusion, detector
misses, and annotation support. It is not an estimate of true per-target
visibility. If the range model fails this calibration check, stop this exact
model without searching more flexible curves on these outcomes.
