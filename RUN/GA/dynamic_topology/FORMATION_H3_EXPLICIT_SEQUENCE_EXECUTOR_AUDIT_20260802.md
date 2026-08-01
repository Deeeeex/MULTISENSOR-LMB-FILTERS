# H=3 explicit formation-action sequence executor audit

## Change

`runFormationModeH3OpenedReturnScreen` now accepts an explicit `N x 3`
`actionSequenceIndices` matrix.  Each nonreference index replays the graph and
fusion weights constructed once from the opened current posterior.  Index 1
recomputes the registered fixed reference at that step.  The existing
`actionIndices` and `interventionDurationSteps` interfaces are translated into
the same sequence representation, so prior one-step and fixed-duration
protocols retain their semantics.

Every explicit sequence must contain exactly one all-reference arm, use only
integer in-bank actions within the reference payload, and contain no duplicate
rows.  Each nonreference graph is checked against the executed graph.  Truth
use, repair, emergency fallback, infeasibility, physical reachability, and
selected rolling-B3 gates remain unchanged.

## Numerical regression

The old and explicit interfaces were run independently from the frozen
`m24-formation-fov / seed 211 / t=104` cache on these two arms:

| Sequence | Mean E-OSPA | Worst sensor | Consensus | Attempted bytes | Delivered bytes |
|:--|--:|--:|--:|--:|--:|
| reference -> reference -> reference | 54.424440592211 | 118.704010351395 | 50.282821401361 | 4,713,192 | 4,583,448 |
| formation 4, trust 0.50 -> reference -> reference | 54.143118342731 | 118.704010351395 | 49.905088566485 | 4,695,768 | 4,566,696 |

Both interfaces reproduced every displayed value exactly.  The explicit arm
also recorded sequence indices `[12,1,1]` and passed its exact-execution and
runtime assertions.

## Checks

- `test_formation_mode_intervention_bank`: pass
- `test_formation_h3_event_conditioned_protocol`: pass
- legacy two-arm numerical replay: pass
- explicit two-arm numerical replay: pass

This audit validates executor compatibility only.  It contains no new method
or performance claim and opens no new state or seed.
