# V272 screen interpretation

## Result

The frozen `fa16508` V272 continuation does not pass the joint M24 event
screen. Relative to the paired V242 continuation over `t=57--73`:

- network E-OSPA changes by `-0.895%` and inter-formation consistency by
  `-1.255%`;
- every formation has non-positive E-OSPA gain, with F3 at `-2.357%`;
- network conditional set RMSE changes by `+3.984%`, but mean absolute
  cardinality error increases from `9.941` to `10.142`;
- F4 conditional RMSE changes by `+0.598%`, while F4 E-OSPA changes by
  `-0.941%` and event cardinality error increases from `10.677` to `10.885`;
- attempted payload bytes in the window increase by `1.814%` because sender
  payload sizes differ, although the spliced full-trace saving over the fixed
  static route remains `9.908%`.

## Interpretation

The lower conditional RMSE is not sufficient evidence of improved
localization. In a set-valued tracker, losing difficult targets can reduce
RMSE over the remaining matches while worsening the actual set estimate. The
simultaneous increase in cardinality error and E-OSPA is consistent with this
selection effect.

The structural V272 result therefore establishes only that reliability-bounded
gateway rotation improves graph mixing. It does not establish that more mixing
is beneficial under heterogeneous FoV. Faster propagation can also spread
negative or incompatible existence evidence, so a topology objective based on
contraction alone is misaligned with the LMB estimation loss.

## Decision boundary

Do not tune the V272 reliability floor, history depth or event window using
this opened outcome. Do not run the complete M24 or X36 tracking episodes for
the frozen V272 policy.

Before another tracking arm, test a posterior-only diagnostic on the frozen
V259 snapshots: determine whether a causal LMB compatibility quantity predicts
which reliability-admissible gateway substitutions preserve label existence
while retaining the structural mixing gain. If it has no stable signal across
the registered anchors, close contraction-only dynamic routing as a deployable
method family. If it does, the next policy must use that quantity as a safety
constraint, charge its control metadata, and remain inside the same
`N+2(F-1)` posterior-message architecture.

## Evidence boundary

This is an interpretation of one opened M24 continuation and is development
evidence only. It is not a validation, cross-scale tracking result or paper
claim.
