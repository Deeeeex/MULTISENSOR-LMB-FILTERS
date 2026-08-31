# V205 mixed-operator complete-sequence finding

## Result

The X36 seed-211, t=72, H=8 teacher applied the five supported-label
residual-KLA actions confirmed by V204 and one final protected F5 support
restoration.  All six actions executed and were charged.

| Metric | Static reference | V204 supported-label sequence | V205 mixed sequence |
|:--|--:|--:|--:|
| Mean E-OSPA | 84.037151 | 77.084367 | 76.919427 |
| Mean RMSE | 59.967347 | 50.927938 | 50.923884 |
| Mean cardinality error | 6.652778 | 5.628472 | 5.607639 |
| Window consensus error | 61.383329 | 52.107033 | 52.378862 |
| Terminal consensus error | 57.542048 | 43.234637 | 45.409265 |
| Attempted bytes | 28,578,864 | 27,683,256 | 27,703,864 |

Relative to static, V205 improves mean E-OSPA by `8.470%`, mean RMSE by
`15.081%`, window consensus by `14.669%`, terminal consensus by `21.085%`,
and attempted bytes by `3.062%`.  Every formation improves in E-OSPA.  Five
formations improve in RMSE; F5 remains the only negative tail at `-1.812%`.

## What the F5 hard restoration changes

V205 differs from V204 only at t=79, where source 17 label `[19,16]` is
inserted into all six F5 receivers whose current existence support is zero.
The action improves F5 E-OSPA from `70.236694` to `69.247053` and F5 RMSE
from `12.552062` to `12.527740`.  It also reduces mean cardinality error by
`0.020833`, but costs `20,608 B` and worsens window and terminal consensus by
`0.271828` and `2.174628` absolute points relative to V204.

The standard LMB KLA existence formula contains the product of every
positive-weight input existence probability.  Consequently, when one input
has exactly zero label support and the other inputs are nondegenerate, KLA
cannot resurrect the label.  V205 confirms that a protected insertion is the
correct operator family for this case, but also shows that this particular
late insertion is not the remaining gate-closing action.

## Method decision

The residual F5 error begins before the t=79 support restoration.  V99 alone
changes F5 RMSE from the static `12.304800` to `12.619932`; the late hard
restoration recovers only part of that loss.  The next controller therefore
needs to choose among three formation-level actions rather than only choosing
a repair label:

1. keep the V99 posterior-withholding decision;
2. release the formation back to the ordinary full-posterior route; or
3. spend residual credit on a sparse complete-label update, using KLA for
   supported labels and protected insertion only for absent labels.

V191 already showed that releasing F5 on the first H=3 page changes its RMSE
tail from negative to `+0.547%`.  V206 is therefore a single structural
combination test: release F5 from withholding at t=72 and retain the positive
V204 KLA sequence for F1/F2/F3/F6.  The t=79 hard insertion is removed.

## Evidence boundary

V205 is one opened X36 teacher window with ideal repair delivery charged but
not sampled.  Formation, source, label, time and operator choices are teacher
inputs.  It establishes an update-operator boundary and localizes the
remaining error to the base routing decision; it is not an online policy,
cross-seed result or validation claim.  V187 remains the balanced development
best until the combined release-and-repair action is evaluated.
