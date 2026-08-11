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
