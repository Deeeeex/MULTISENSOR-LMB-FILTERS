# V179 rollout policy-iteration finding

## Paired X36 seed-211 result

| Arm | Mean E-OSPA | E gain | Mean RMSE | RMSE gain | Minimum formation RMSE gain | Window consensus gain | Attempted-byte saving |
|:--|--:|--:|--:|--:|--:|--:|--:|
| Static full posterior | 84.037151 | -- | 59.967347 | -- | -- | -- | -- |
| V162 analytic Top-4 | 76.322368 | +9.180% | 55.219073 | +7.918% | -29.799% | +11.509% | +2.458% |
| V169 static-trained sequential learner | 74.653711 | +11.166% | 53.589941 | +10.635% | -3.552% | +9.804% | -0.071% |
| V179 rollout-aware one-action learner | 74.573180 | +11.262% | 53.566866 | +10.673% | -2.351% | +9.823% | -0.027% |

V179 preserves the strong global tracking and consensus gains while reducing
the F5 formation-RMSE regression by 1.20 percentage points relative to V169.
Its communication excess over the static baseline falls from 20,320 bytes to
7,608 bytes.  It nevertheless fails the registered gate because the F5
formation RMSE remains below the static baseline and attempted-byte saving is
still slightly negative.

## Recursive mechanism

| Time | Static F5 RMSE | V169 F5 RMSE | V179 F5 RMSE |
|--:|--:|--:|--:|
| 78 | 8.5075 | 10.0512 | 10.0413 |
| 79 | 7.5289 | 10.9991 | 9.8266 |

At `t=78`, V179 transfers label `[7,6]` from source 15 to receivers 25--29
and label `[31,23]` from source 34 to receiver 30.  These actions change the
state distribution that the `t=79` selector receives.  At `t=79`, four
receivers request the jointly useful label `[19,16]` from source 16, while
receivers 27 and 29 abstain because their new states fall outside the frozen
model support.  The resulting improvement over V169 confirms that rollout
aggregation is useful, but the remaining distribution shift requires another
on-policy aggregation round rather than threshold tuning on the opened cells.

## Decision

1. Keep the dynamic physical-tree backbone and the single complete-label
   repair action.
2. Capture the V179 pre-side-channel states and add its `t=78` cells to the
   training pool; keep its `t=79` cells as the next opened development readout.
3. Refit the same compact model family using the existing V166 F3 calibration
   boundary.  Do not alter the probability threshold using V179 `t=79`.
4. Defer source-side synopsis sparsification until formation RMSE is nonnegative;
   the remaining byte deficit is only 7,608 bytes.

## Evidence boundary

This is an opened same-seed X36 development comparison.  V179 inference uses
only current posterior, topology and FoV features, but its training data and
repair schedule were developed after opening seed 211.  The result is not
validation or generalization evidence and remains repository-only until the
registered recursive gate passes.
