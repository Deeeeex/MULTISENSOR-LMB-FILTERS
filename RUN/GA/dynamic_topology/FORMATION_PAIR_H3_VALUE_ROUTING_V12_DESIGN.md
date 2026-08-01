# Conservative formation-pair H=3 value routing v12

## Opened-development decision

The pre-registered v11 single-formation bank did not pass its headroom gate.
Across the five opened M24 states, four states had a strictly tail-safe
positive action and the mean best tail-safe gain was +3.594%, but only one
state reached the required +5% strong-gain threshold.  The gate required two.
The v11 result therefore authorizes neither model fitting nor a validation
claim.

The failure is localized rather than null.  Several high-mean actions were
rejected only because their first-step change propagated a very small loss to
another formation.  A deployable joint projector will select several local
actions simultaneously, so a singleton-only teacher also leaves its leading
interaction residual unmeasured.

## Pre-registered v12 extension

The v12 screen keeps every completed v11 singleton arm and adds every
unordered pair of formations at the fixed conservative dynamic trust weight
0.30.  Pair membership does not depend on truth, future measurements, v11
return rankings, or seed identity.  For F formations, the supplemental bank
contains

1 + F(F - 1) / 2

actions including the reference.  M24 therefore adds six pair actions and
X36 would add fifteen.  This remains quadratic in network organization and
is far smaller than enumerating all joint mode assignments.

Only the receiver rows belonging to the two selected formations change for
the first step.  The next two steps use the same registered fixed reference.
The screen uses common measurements, link uniforms, initial posterior, and
filter RNG.  Truth and future measurements are available only to the offline
H=3 score.

## Frozen headroom gate

The augmented summary merges the completed v11 singleton records with the
new v12 pair records only after verifying identical reference outcomes and
all runtime safety checks.  The v11 thresholds are retained without
relaxation:

- per-formation gain tolerance: 0.000%;
- at least 3 of 5 states have a positive tail-safe action;
- at least 2 of 5 states have a tail-safe gain of at least 5%;
- mean best tail-safe gain is at least 2%;
- every executed arm passes exact first-action replay, selected rolling-B3,
  payload-emergency, repair, infeasibility, and truth-use gates.

Passing this opened-development gate would authorize feature/target model
development only.  It would not support a learned-policy claim, an M24
validation claim, or an X36 validation claim.  Failure requires replacing the
candidate topology family rather than changing these thresholds.

## Interaction diagnostic

For each pair at trust 0.30, the report stores the residual between its H=3
mean gain and the sum of the matching two singleton gains.  This residual is
the first empirical term needed by the conditional regret statement in v11.
It also tests whether a purely additive formation-value model is adequate or
requires an explicit pair correction.
