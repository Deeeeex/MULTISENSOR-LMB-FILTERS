# V98 X36 within-window risk-migration probe

V97 already selects every safe positive-net formation at X36 t=72, yet its
matched static gain is only 2.412%.  The next decision is whether the safe set
changes after the first and second recursive fusion updates.

The probe reruns only the matched static and V97 arms at the already-opened
X36 t=72 state.  At t=72, 73 and 74 it captures the candidate's current
pre-fusion posterior and recomputes the V97 rescue, useful-loss, downward-
crossing and selected-set quantities using current geometry and past selected
graphs.  Target truth and future measurements are not inputs to these
features.

If the selected set changes or a formation excluded at t=72 becomes
addressable, an online per-step selector has a concrete mechanism to test.  If
neither happens, the fixed-set failure cannot be attributed to stale temporal
selection and the next method must expand the action type rather than its
schedule.  This probe does not authorize a new tracking or validation claim.

## Result

The mechanism is present.  The positive-net set changes from formations
`[1 2 4 5]` at t=72 to `[1 2 3 4 5]` at t=73 and remains there in the
fixed-V97 diagnostic trajectory at t=74.  Formation 3 becomes both safe and
positive-net only after the first recursive update; formation 6 becomes
addressable at t=73 but remains non-positive-net.  The selected observable net
benefit rises from 0.00643 to 0.01059 and then 0.01820.

This rejects a frozen three-step set as a faithful implementation of the
current-state rule.  The next method must recompute the selector after every
local update and apply the resulting set to that same step's payload plan.
The t=74 set must not be hard-coded from this probe because changing the t=73
action changes the posterior used at t=74.
