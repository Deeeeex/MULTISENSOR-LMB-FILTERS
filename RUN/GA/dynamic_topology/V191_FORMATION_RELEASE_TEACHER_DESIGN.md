# V191: selective full-posterior release teacher

## Question

M24 V99 improves the full H=3 mean E-OSPA, RMSE, consensus and communication
objectives, but its F4 position RMSE is dominated by one t=104 cardinality
spike.  A forced positive-label edit makes that spike worse.  V191 asks the
narrower counterfactual: if F4 is removed from V99's withheld-payload set only
at t=104, does the ordinary full posterior prevent the extraction failure while
retaining the useful part of V99?

## Intervention

The V99 policy still recomputes its receiver-formation set from the current
posterior.  After that decision and before message construction, V191 removes
one teacher-named formation from the selected set on one registered page.
Removing the formation restores the existing full-posterior message path; it
does not add a side channel, alter topology, change fusion weights or inject an
external posterior.  The existing payload estimator therefore charges the
actual restored full message automatically.

The first screen is frozen as follows:

| Field | Value |
|:--|:--|
| Scale / anchor / horizon | M24 / seed 211, t=104 / H=3 |
| V99 selection before release | F1 + F3 + F4 at the first page |
| Teacher release | F4 at t=104 only |
| Later pages | ordinary online V99 recomputation, no forced release |
| Positive-label repair | disabled |
| Paired state | same opened cache, measurements, link uniforms and filter RNG |

## Decision rule

- If the F4 RMSE gap closes while mean E-OSPA and RMSE stay positive and byte
  saving remains positive, selective release becomes a required action in the
  controller.  The next task is a causal risk gate for choosing it.
- If F4 remains strongly negative, the failure is not caused by first-page
  payload withholding alone; inspect receiver-selective or label-negative
  evidence without broadening the positive-label search.
- V191 is a teacher intervention.  A positive result is action-space headroom,
  not a deployable method or validation claim.
