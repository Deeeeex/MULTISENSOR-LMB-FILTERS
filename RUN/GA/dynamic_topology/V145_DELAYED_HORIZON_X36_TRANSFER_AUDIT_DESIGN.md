# V145 delayed-horizon X36 transfer audit

## Status and purpose

This is a development audit frozen before opening any V145 X36 result.  It
does not revise the original V145 gate: V145 remains a failed immediate-
intervention candidate because its M24 intervention gain was `+3.060%` rather
than `+5%`.  The new question is narrower and temporally different: does its
safe delayed benefit transfer from M24 to X36?

The question is justified by the completed M24 trajectory rather than by one
isolated metric.  V145 obtained `+6.221%` full-window and `+6.938%`
mature-window E-OSPA gains, `+1.564%` minimum-sensor and `+3.335%`
minimum-formation gains, exact reentry, and `0.117%` attempted-byte saving.
V149 then showed that replacing hundreds of label roles barely changes the
final output because the inherited readout layer dominates them.  Testing the
scale transfer of the simpler temporal gate is therefore the next decisive
experiment.

## Frozen execution

- Method: unchanged `protection-load-gated-role-v145-v1`.
- Preset / seed: `x36-formation-fov` / `1601`.
- Action, anchor, route, fusion weights, delivery realization, W/R schedule,
  output readout and communication accounting: resolved by the existing V145
  frozen protocol with no retuning.
- Every attempted edge carries exactly one complete mixture-aware LMB
  posterior; cross-formation edges remain R-only and auxiliary bytes remain
  zero.

## Development decision rule

The delayed-horizon hypothesis survives only if X36 simultaneously has:

1. at least `+5%` full-window and mature-window E-OSPA gains;
2. nonnegative intervention gain;
3. minimum sensor and formation gains no lower than `-0.01%`;
4. exact registered reentry;
5. zero auxiliary payload and no attempted-byte increase.

A pass is headroom evidence, not paper evidence, because this hypothesis was
formed after observing the original M24 result.  It authorizes freezing the
same temporal method and evaluating fresh, paired M24/X36 scene--seed blocks.
Only that held-out matrix may enter the main progress document.  A failure
closes delayed whole-posterior W/R scale transfer and returns the method line
to a downstream-outcome-aligned action teacher rather than another cadence or
label-score sweep.
