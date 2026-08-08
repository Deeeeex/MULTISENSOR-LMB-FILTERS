# Retention-gated B4 pulse routing V51

## Why this is the next method branch

The X36 convoy result rejects V49's graph-only premise: V49 improves its
registered contraction score and delivered B4 connectivity, yet worsens
tracking, cardinality and inter-formation consensus. V50 tests the smallest
repair by using the current posterior to choose a more useful formation cycle.
In the complete seed-1009 run, V50 selected no nonreference cycle in any of the
40 windows because none passed its posterior-value gate. It therefore exactly
reproduced V46 on every tracking metric. Sender replacement is not the active
source of headroom in this scene.

Earlier M24 continuation experiments identify a stronger estimator-side
mechanism. Temporarily withholding incompatible cross-formation inputs, then
restoring them in stages, improved mean tracking in four separate windows by
`1.40%`, `4.30%`, `7.76%` and `10.39%`, while also improving cardinality and
window consensus. Those experiments were short-window probes rather than a
full-horizon, cross-scale method. V51 turns the mechanism into a scalable
online B4 controller instead of repeating their state-specific protocol.

## Core decision

V51 keeps full LMB posterior exchange and fusion unchanged. It controls only
which cross-formation residual messages are admitted on the first phase of a
four-step B4 window.

At each pulse:

1. Build the current V46 B4 route without reading the posterior.
2. For each receiving formation, aggregate the current label-wise existence
   deficit of the selected residual inputs, weighted by the current link
   reliability. The score uses only the current observable posteriors; it does
   not use truth, future measurements or realized delivery outcomes.
3. If admitting the pulse is predicted to reduce supported existence mass,
   defer that formation's incoming cross message for this pulse. The dominant
   same-formation route remains active, and its removed residual weight returns
   to the receiver's self weight.
4. A formation cannot defer two consecutive pulse opportunities. Thus every
   formation receives a cross-formation update at least once in eight steps.
   When several formations need recovery, lower-risk formations reconnect
   first while the remaining high-risk formations retain protection.
5. If the posterior signal is weak or malformed, execute the unmodified V46
   pulse. The controller never invents an edge outside the current physical
   graph and never increases the scheduled message count.

The resulting action is not arbitrary graph pruning. It is a bounded delay of
an estimator-adverse message under an explicit temporal information-flow
guarantee.

## Why the action space scales

The protection layer makes one binary defer/reconnect decision per formation,
followed by a deterministic temporal projection. It therefore grows linearly
with formation count. No formation-cycle enumeration, `2^F` subset enumeration
or sensor-level graph search is required. Removing the rejected V50 ranking
layer also avoids repeatedly constructing label features for unused routes.

For M24 and X36 the controller handles four and six formation decisions per
pulse respectively. Decisions are made once every four filter steps rather
than at every local update.

## Relationship to the older M24 mechanism

V35 established that posterior retention debt contains useful information and
that staggered recovery is better than an abrupt return to the reference
route. V51 retains those two ideas but changes the deployment surface:

- it runs from the beginning of a complete scenario instead of intervening at
  preselected three-step windows;
- it runs on the current repaired V46 B4 route rather than an intervention-only
  cached route;
- it replaces expensive exact outcome enumeration with direct label-set
  transfer features at the online selector;
- it enforces an eight-step maximum information delay, making the scale rule
  independent of the number of formations.

## Method decision from the V50 run

V50 used the V46 fallback in `40/40` X36 convoy windows and consequently
matched V46 exactly: full-horizon E-OSPA `126.370`, focus E-OSPA `123.494` and
mean absolute cardinality error `14.534`. This is a clean neutral result, not a
small positive effect. V51 therefore does not retain V50's candidate ranking.
It measures absolute existence-retention debt on the V46 cross input and may
defer that input directly. The `2%` debt threshold comes from the earlier M24
retention mechanism rather than being fitted to the X36 tracking outcome.

## Compact experiment sequence

1. Run one X36 convoy candidate arm against the already saved V46 baseline.
   Primary metrics are full/focus E-OSPA, mean cardinality error, worst-sensor
   E-OSPA and inter-formation OSPA. Report message and payload changes.
2. If the direction is positive, freeze the thresholds and run X36
   merge-split and curved-corridor scenes.
3. Use fresh seeds on convoy plus the two new styles, then apply the unchanged
   controller to the paired M24 scenes.

The paper-level claim requires positive tracking and cardinality changes at
both scales and across multiple scene styles. A graph score or a single
short-window improvement is not sufficient.
