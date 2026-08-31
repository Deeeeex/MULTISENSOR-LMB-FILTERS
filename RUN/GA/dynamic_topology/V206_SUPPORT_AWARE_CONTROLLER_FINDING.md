# V206 support-aware controller finding

## Result

V206 combines the three actions isolated by the preceding mechanism studies:

1. keep a V99 formation-level posterior-withholding decision;
2. release a formation back to its ordinary full-posterior route; and
3. spend residual communication credit on one complete-label KLA update.

On the opened X36 seed-211, t=72, H=8 window, F5 is released from the
first-page withholding decision.  Five supported-label actions are then
applied at t=72, 73, 76, 77, and 78 for F2, F6, F3, F1, and F3.  Every
label action uses the complete Bernoulli GM payload and a 0.5-weight
mixture-aware residual KLA update.  All five actions execute and are charged.

| Metric | Static full-posterior reference | V204 label-KLA sequence | V206 release + label KLA |
|:--|--:|--:|--:|
| Mean E-OSPA | 84.037151 | 77.084367 | 77.404566 |
| Mean RMSE | 59.967347 | 50.927938 | 50.908755 |
| Mean cardinality error | 6.652778 | 5.628472 | 5.677083 |
| Window consensus error | 61.383329 | 52.107033 | 52.325514 |
| Terminal consensus error | 57.542048 | 43.234637 | 43.272880 |
| Attempted bytes | 28,578,864 | 27,683,256 | 27,654,584 |

Relative to static, V206 improves mean E-OSPA by `7.892%`, mean RMSE by
`15.106%`, window consensus by `14.756%`, terminal consensus by `24.798%`,
and attempted communication by `3.234%` (`924,280 B`).  Its weakest-sensor
E-OSPA and RMSE gains are both positive (`21.833%` and `10.248%`).  All six
formations improve in E-OSPA:

`[0.883, 12.634, 9.546, 8.970, 10.112, 4.868]%`.

Five formations also improve in RMSE:

`[2.352, 55.486, 5.764, 1.789, -1.074, 41.225]%`.

The sole remaining strict-gate failure is therefore F5 formation-level RMSE.

## What the full-posterior release establishes

Releasing F5 on the first page improves its RMSE tail from `-2.009%` in V204
to `-1.074%` in V206.  It does not merely trade communication for tracking:
V206 uses `28,672 B` fewer attempted bytes than V204 after downstream payload
sizes are accounted for.  The action family is thus recursively compatible
with the five supported-label repairs and with positive communication credit.

The temporal trace also explains why the gate does not close.  Relative to
static, the F5 mean per-page RMSE difference under V206 is `0.000`, `-0.046`,
`-0.262`, `-1.280`, `-1.755`, `+0.416`, `+0.936`, and `+3.048` for
t=72:79.  The first-page release protects t=73:76, but subsequent withholding
recreates the tail at t=77:79.  A one-time early release is therefore not a
complete controller for a persistent support defect.

## Observable late-tail diagnosis

The t=79 V206 local posterior was replayed through the same truth-free V99
state constructor.  F5 has zero observation-supported set-entry risk at that
page, so lowering or tuning the existing set-entry threshold cannot authorize
the needed action.  At the same time, label `[19,16]` has exactly zero fused
existence at every F5 receiver, while 24 external local posteriors carry it
with existence above 0.97.  This is an observable persistent support gap,
not an ordinary set-entry event and not a supported-label precision refresh.

The next bounded experiment therefore adds one late full-posterior release
when the already withheld formation still has a formation-wide zero-support
gap backed by broad, high-existence external support.  The release is a
deterministic safety fallback; label/action value ranking remains the part
that can later be learned.  No set-risk threshold sweep is authorized.

## Fusion and evidence boundary

The ordinary runtime and every residual label action use
`buildMixtureAwareKlaReferenceConfig`, which activates the repository's
componentwise powered-GM KLA approximation.  The experiment does not use the
legacy `gaLmbTrackMerging` path that moment-matches each sensor posterior to a
single Gaussian.

V206 remains one opened, ideal-delivery, fully charged teacher window.
Formation, source, label, time, and operator choices are teacher inputs.
It establishes a near-complete action-space ceiling and a causal missing-mode
diagnosis; it is not an online policy, cross-seed result, or validation claim.
V187 remains the current policy-level balanced development best, while V206
is the current mechanism-level best.
