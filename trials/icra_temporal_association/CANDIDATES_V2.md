# Persistent identity conflicts with the original kinematic assignment

V1 lowered the scored wrong-pair rate but worsened OSPA in both conditions.
The largest regression occurred without a single executed known-label
rejection. Preserve V1 and its complete results. The two candidates below
restore the original free-label Gaussian KL cost and change only the
handling of a persistently inconsistent shared identity.

## Common conflict state

Use the unchanged current-observation message, majority-support criterion,
same-frame distances and three-frame nominal thresholds from V1. Enter a
receiver-local conflict state for a known shared label only with qualified
current evidence, at least two qualified received samples in the three-frame
window, and normalized accumulated discrepancy greater than 1. Clear that
state when the same requirements provide normalized discrepancy at most 1.
Missing or unqualified observations do not clear it. Remove inactive state
when the label is absent from both current inputs. State depends only on
local information and packets actually received.

If no currently shared label is in conflict, call the original matching
function with its exact arithmetic. Otherwise unlock those known labels,
forbid their old conflicting correspondence, and use the original 4D
symmetric Gaussian KL cost for every other free pair. Retain the original
50-per-endpoint unmatched cost, equivalently normalized 0.5 with KL/100.
Current detection geometry never replaces that global kinematic cost.

## Candidate R: reopen and abstain

After reassignment, omit any unmatched remote component whose original
label would still collide with a local label. If a conflicting local label
has no replacement match, the remote source abstains for that label through
the existing source-weight transfer. This is the V1 conflict handling with
the original global assignment and persistent conflict state.

## Candidate S: retain a distinct observation branch

Use exactly the same conflict state and assignment as R. For a colliding,
unmatched remote component, allow a separate branch only when the current
three-frame evidence still satisfies the entry criterion. Its original
birth-location identifier must be below 1,000,000. Give the branch the
original birth time and the deterministic label location
1,000,000,000 + 1,000,000*receiver + original_birth_location.

Create or reuse that branch only if its label is absent from all current
local labels and all other retained remote labels. Otherwise use R's
abstention. Generated branch labels are never split again. This allows at
most one receiver-specific branch per original label, with no ground-truth
ID, unbounded alias counter or fresh label on every frame.

The newly separated remote branch is initially a remote-only posterior;
the local source abstains for that branch rather than manufacturing a
censored absence. The conflicting original local label follows R's rule.
Subsequent prediction, local update, association and fusion are unchanged.
Log every branch label, original source key and source-specific abstention.
Labels fit in the existing packet fields, so both candidates retain V1's
416-byte per-component codec; any increase in component count is charged.

## Frozen assessment

Test R and S with the same GCE density equation. First run original 0001
and 0008 with unchanged GCE and both candidates, validating exact baseline
parity and all new decisions. 0001 exercises real identity conflicts; 0008
checks that removing the global metric replacement preserves the original
trajectory in a segment without original persistent known-label conflicts.
Then complete the remaining seven original development segments.

Advance only a candidate improving both link-condition sequence-macro OSPA
means with a nonincreasing pooled 2 m wrong-pair rate; choose the lowest
two-condition mean among eligible candidates, with ties preferring R.
All nine segments and all failures remain in the report. Compare an advancing
candidate to all V1 outcomes, the complete exposed release and the frozen
additional test cohort before claiming improvement. Keep the same matching
frontend on a conservative fusion control in the final assessment.
