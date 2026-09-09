# Cross-vehicle association and temporal consistency

Base commit: `dc6a4c14`. The user selected this direction after the admission
study. The protected GCE implementation and every completed result remain
unchanged. New replay code and results belong to this directory.

## Diagnostic scope and data exposure

First inspect the original nine V2V4Real development segments and all five
already evaluated V2X-Real validation segments, in both link conditions.
Use original GCE and Guarded Scalar trajectories. All these data are exposed
development/diagnostic data for the new direction. The remaining V2V4Real
release is also exposed. A new method must be frozen before evaluating any
additional public recording, with its acquisition identity documented.

Join local Gaussian moments, local existence probabilities and original
source labels to each recorded cross-vehicle fusion group. Distinguish
locked equal-label pairs from newly assigned different-label pairs. Verify
the existing assignment objective independently, allowing only exact
objective ties. Do not infer an association from the fused mean alone.

For diagnostic identities, extract each local MAP cardinality with the
existing rule and apply the existing spatial crop. Assign those estimates
one-to-one to annotation centers at 2 m and, separately, 12 m, maximizing
valid cardinality before minimizing squared distance. A pair is scored
correct/wrong only when both endpoints have a valid diagnostic assignment.
Count available common-identity opportunities, missed common-identity pairs,
pair-partner changes and adjacent-frame identity switches of final tracks.
Report denominators and unscored pairs. Annotation identities are used only
by this offline diagnostic, never by matching or tracking code.

## Bounded development

Use the diagnostic to decide whether to test an ambiguity/motion gate or
temporal pair consistency. Freeze a small, explicit candidate set and an
exit condition before any candidate trajectory is evaluated. Each candidate
must execute prediction, local update, association, fusion and feedback on
its own state. Keep the detector, calibration, birth/local-update model,
radio masks and GCE evidence equation fixed for an association comparison.
Any coordinate-model control is separate and explicitly paired.

History may contain only information already available at the receiver.
No future detections, annotation IDs, truth-derived motion or communication
through undelivered packets may affect a decision. A temporal cost must
account for changing coordinates, or use an invariant same-frame quantity.
Count additional payload/state/runtime if introduced.

Select on the original nine segments using the arithmetic mean of the two
sequence-macro OSPA values. Advance only if both link conditions improve
over unchanged GCE and the diagnostic association error does not increase.
Publish every registered candidate and any exit without advancement. Only
after a candidate advances should it be assessed on the full exposed
release and additional recordings frozen before their tracking scores.
