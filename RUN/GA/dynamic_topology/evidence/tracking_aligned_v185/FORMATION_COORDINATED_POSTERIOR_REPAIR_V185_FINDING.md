# V185 recursive formation-coordination finding

## Paired X36 result

| Arm | Mean E-OSPA | Mean RMSE | F5 RMSE gain | Consensus gain | Attempted bytes | Byte saving |
|:--|--:|--:|--:|--:|--:|--:|
| Static full payload | 84.037151 | 59.967347 | -- | -- | 28,578,864 | -- |
| V179 independent F5 repair | 74.573180 | 53.566866 | -2.351% | +9.823% | 28,586,472 | -0.027% |
| V185 coordinated F5 repair | 74.813076 | 53.619306 | -0.628% | +9.804% | 28,593,032 | -0.050% |

V185 preserves large aggregate gains and reduces the weakest formation RMSE
deficit by 1.723 percentage points relative to V179.  It nevertheless fails
the strict gate: F5 RMSE remains negative and attempted communication exceeds
both the static reference and V179.  The formal candidate runtime is 786.68 s,
slightly below V179's 801.55 s; longer observed wall-clock time included
scenario/action-bank setup and interrupted pre-runs and is not method runtime.

## Recursive action shift

V179 uses mostly `[7,6] <- 15` at t=78, then `[19,16] <- 16` at four F5
receivers at t=79.  V185 instead applies the common action `[31,24] <- 32` to
all six receivers at t=78.  That changes the next visited state, so the t=79
formation rule selects `[19,13] <- 15`, not the `[31,24] <- 31` action that was
safe and strongly RMSE-positive in the V180 pre-action state.

The result therefore does not refute formation coordination.  It shows that
coordinating both repair times creates an avoidable rollout-distribution
shift: the first coordinated action removes the state for which the second
action was designed.

## Next frozen decision

The next arm should be temporally hybrid rather than another scalar-model
sweep:

1. reproduce the V179 independent selector at t=78;
2. enable the V185 formation-coordinated selector only at t=79, using the V181
   model and V185 policy;
3. intersect the six receivers' physical-neighbor sets before advertisements,
   because a source outside that intersection can never form a common exact
   `(label, source)` action;
4. charge synopsis, request and response bytes only on the retained physical
   links and keep the one-edit-per-receiver limit.

This design preserves the known V180 t=79 state, targets the observed F5
localization deficit, and removes communication/computation spent on
provably ineligible sources.

## Evidence boundary

V185 is an opened X36 seed-211 development result.  It fails the registered
gate and is repository-only; it must not be promoted to the Lark main document
or manuscript result tables.
