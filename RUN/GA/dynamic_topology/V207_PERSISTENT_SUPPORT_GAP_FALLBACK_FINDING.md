# V207 persistent support-gap fallback finding

## Result

V207 keeps the V206 action sequence and adds one ordinary full-posterior
release for F5 at t=79.  The release is triggered in this teacher on the
second consecutive page where label `[19,16]` has zero fused existence in
all six F5 receivers while many external local posteriors retain high
existence.

The release executes.  F5 is in the V99 withheld set at t=79, and the added
payload raises attempted bytes on that page from `3,582,952` to `3,651,312`
(`+68,360 B`).  It does not improve the result.

| Metric | Static reference | V206 | V207 | V207 vs V206 |
|:--|--:|--:|--:|--:|
| Mean E-OSPA | 84.037151 | 77.404566 | 77.447588 | +0.043022 |
| Mean RMSE | 59.967347 | 50.908755 | 50.909839 | +0.001084 |
| Window consensus error | 61.383329 | 52.325514 | 52.442306 | +0.116792 |
| Terminal consensus error | 57.542048 | 43.272880 | 44.207216 | +0.934336 |
| Attempted bytes | 28,578,864 | 27,654,584 | 27,722,944 | +68,360 |
| F5 E-OSPA gain | 0 | +10.112% | +9.791% | -0.322 pp |
| F5 RMSE gain | 0 | -1.074% | -1.127% | -0.053 pp |

At the only changed page, t=79, F5 mean E-OSPA worsens from `68.008628` to
`70.073653`, and mean RMSE worsens from `10.577130` to `10.629180`.  V207
therefore fails the strict formation gate and is dominated by V206 in every
aggregate objective relevant to the method.

## Why the support-gap diagnosis was insufficient

The `[19,16]` absence is not created by the V206 route.  In the frozen
pre-intervention reference cache at t=72, the same label already has zero
existence in every F5 receiver.  It also remains zero in all F5 local and
fused posteriors throughout the V206 t=72:79 window, while 24--25 external
local posteriors usually carry it.  Broad external existence is therefore
not enough to establish that the label is relevant to the receiving
formation or that its absence causes the candidate's error.

The ordinary full-posterior release also cannot be interpreted as a generic
support restoration.  Once the receiving label has zero support, standard
geometric averaging does not guarantee that merely restoring an edge will
recreate it; meanwhile the full payload changes every other label and can
introduce harmful information.  The V207 result demonstrates exactly this
failure mode.

## Method decision

The rule

`persistent zero support + broad external support -> forced full release`

is rejected.  A zero-support label remains a hard inadmissibility condition
for residual label KLA, but it is only a contextual feature for deciding
between full release and no-op.  A release requires predicted finite-horizon
value, including spatial relevance, observation opportunity, set/cardinality
state, history relative to the pre-intervention state, tail risk, and bytes.

The next controller should therefore prevent harmful support loss before it
becomes irreversible when the current state predicts value, rather than use
an unconditional after-the-fact fallback.  V206 remains the mechanism-level
current best and V187 remains the policy-level current best.

## Evidence boundary

V207 is one opened X36 seed-211, t=72, H=8 teacher intervention.  Formation,
page, and release type are opened inputs.  It falsifies this specific
deterministic fallback on the diagnosed window; it is not cross-seed or
cross-scene evidence and does not validate a learned controller.
