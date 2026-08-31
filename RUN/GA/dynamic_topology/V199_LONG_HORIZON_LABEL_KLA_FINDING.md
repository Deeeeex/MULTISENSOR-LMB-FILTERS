# V199 long-horizon label-KLA finding

## Question

V198 showed that releasing the complete F2 formation posterior at the only
temporally supported repair opportunity can reduce the X36 RMSE deficit, but
does not reverse it. V199 asks whether the problem is the decision to repair
F2, or the granularity of the information restored.

The frozen comparison uses `x36-formation-fov`, seed `211`, opened time `72`,
and an eight-step recursive horizon (`t=72:79`). At `t=72` only, the teacher
transmits source sensor 19's complete GM representation for label `[13;12]`
and applies a residual label-wise KLA update with source weight `0.5` to the six
F2 receivers. The action is delivered ideally for mechanism isolation but its
full payload is charged. All arms share the same cached posterior,
measurements, link uniforms, filter RNG, horizon, carrier graph, and
communication constraints.

## Frozen result

| Arm | Mean E-OSPA | E-OSPA gain | Mean RMSE | RMSE gain | Consensus gain | Byte saving |
|:--|--:|--:|--:|--:|--:|--:|--:|
| Static full-payload reference | 84.037151 | 0 | 59.967347 | 0 | 0 | 0 |
| V99 omission-only controller | 79.451115 | +5.457% | 62.172152 | -3.677% | +8.803% | +5.423% |
| V198 full-F2 release | 79.535696 | +5.357% | 60.832025 | -1.442% | +7.809% | +5.341% |
| V199 one-label residual KLA | 78.274634 | +6.857% | 58.204481 | +2.940% | +11.348% | +4.870% |

Relative to V99, the single label-wise repair improves mean E-OSPA by
`+1.481%`, mean RMSE by `+6.382%`, and consensus by `+2.790%`, at an additional
`157,896 B` of attempted communication across the recursive horizon. The
repair itself costs `60,232 B` at `t=72`; changed downstream posterior sizes
explain why the realized horizon-level increment is larger.

## Local effect

The formation-level gains of V199 relative to the static reference are:

| Formation | F1 | F2 | F3 | F4 | F5 | F6 |
|:--|--:|--:|--:|--:|--:|--:|
| E-OSPA gain | -0.931% | +12.634% | +7.711% | +8.970% | +12.496% | -0.021% |
| RMSE gain | +1.847% | +55.486% | -29.799% | +1.789% | -2.561% | -0.749% |

Compared with V198, the F2 label-wise action changes the F2 RMSE result from
`-2.720%` to `+55.486%`; the other five formations are unchanged. This isolates
action granularity as the cause: restoring a complete formation posterior
injects irrelevant or conflicting state information, whereas restoring one
complete, high-value label corrects the localized spatial defect without
overwriting the rest of F2's posterior.

## Method implication

The controller cannot be a binary choice between omission and full posterior
release. The evidence now supports three semantically distinct actions:

1. **Omit / no-op** when the reduced topology remains locally safe.
2. **Full formation restore** for global extraction, cardinality, or
   unsupported set-entry failures, as seen in the M24 F4 mechanism screen.
3. **Complete source-label residual KLA** for localized spatial-precision
   failures, as isolated here for X36 F2.

Temporal support determines whether a repair token should be spent; it does
not determine which payload granularity is appropriate. The next analytic
selector should therefore score action type as well as formation value, using
only causal observables such as risk reduction, precision gain, source
quality, peer agreement, motion compatibility, and temporal support. Truth is
reserved for offline teacher attribution and evaluation.

## Evidence boundary and next decision

V199 is a single opened-window teacher experiment, not a deployable policy or
cross-scene result. It fails the development tail gate because F3 remains the
dominant RMSE failure (`-29.799%`), while F1/F6 also have small E-OSPA losses.
It therefore does **not** replace V187 as the current balanced best. V187
remains the main-document snapshot with E-OSPA `74.678760`, RMSE `53.540189`,
window/terminal consensus gains of `+9.834% / +13.959%`, and byte saving of
`+0.160%` on the same X36 opened horizon.

Do not tune the F2 source, label, or KLA weight next. The next bounded task is
to use the already captured H=8 posterior snapshots to locate the time,
receivers, and labels responsible for F3's RMSE deficit. Only after that
attribution should a third teacher action be run or the three-action selector
be implemented.
