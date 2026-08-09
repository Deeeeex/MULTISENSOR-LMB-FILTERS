# V56 tracking-aligned duration attribution result

## Decision

Holding the best V56 formation-local source/trust action for longer is safe
and directionally useful, but it does not create enough tracking headroom.
V57 must therefore expand the spatial action coverage to a heterogeneous
multi-formation mode vector.  Duration and recovery remain control variables,
not the primary source of gain.

## Frozen comparison

- preset / seed / time: `m24-formation-fov-convoy / 1201 / 60`;
- horizon: three steps;
- compared arms: reference, formation-1 trust 0.50, formation-1 trust 0.70;
- duration: one, two, or three consecutive intervention steps;
- reference cache: the same t=60 cache used by the V56 atomic headroom screen;
- one-step result commit: `f02c21f`;
- duration-capable runtime commit: `2f8cac4`.

| Duration | Best action | Mean tracking gain | Consensus gain | Attempted-byte saving | Strict constraints |
|:--|:--|--:|--:|--:|:--:|
| 1 | formation 1, trust 0.70 | +1.313% | +0.537% | +0.683% | pass |
| 2 | formation 1, trust 0.50 | +1.410% | +0.697% | +1.738% | pass |
| 3 | formation 1, trust 0.50 | +1.499% | +0.835% | +2.886% | pass |

Extending the action from one to three steps adds only 0.186 percentage
points of network-mean tracking gain.  The best three-step result remains
3.501 percentage points below the pre-registered 5% M24 headroom threshold.
Worst-sensor and minimum-formation gains remain nonnegative, so the result is
not a safety failure; it is an action-coverage failure.

## Consequence for V57

V57 should compose one mode per formation and evaluate the complete joint
route, instead of accepting a joint action only when every isolated local
action passed independently.  The joint candidate layer should:

1. combine heterogeneous reference/source-trust/protection modes across
   multiple formations;
2. score retention, payload, physical support, and rolling information flow
   on the realized complete graph;
3. use a bounded structured search rather than exhaustively running every
   mode vector through the tracker;
4. retain state-dependent hold and recovery to prevent the consensus debt
   observed in earlier uniform coordinated actions;
5. clear the unchanged M24 5% headroom gate before any X36 outcome is opened.

The duration experiment is development attribution only.  It does not support
generalization or validation claims.
