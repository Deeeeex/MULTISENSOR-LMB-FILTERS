# V98 finding: the useful payload-control set is time varying

## Question

V97 applies the formation set selected at the opened state for all three
fusion steps.  Its X36 t=72 gain over the matched static baseline is only
2.412%, below the registered 5% mean-gain gate.  V98 asks whether the weak
result is consistent with that fixed set becoming stale inside the window.

## Evidence

The diagnostic uses the candidate arm's current pre-fusion posterior, current
geometry and past selected graphs only.  It does not use target truth, future
measurements or future outcomes.

| Time | Addressable formations | Positive-net formations | Selected net benefit |
|---:|:---|:---|---:|
| 72 | `[1 2 4 5]` | `[1 2 4 5]` | 0.00643 |
| 73 | `[1 2 3 4 5 6]` | `[1 2 3 4 5]` | 0.01059 |
| 74 | `[1 2 3 4 5]` | `[1 2 3 4 5]` | 0.01820 |

Formation 3 is unsafe at t=72 because it has one cross-supported downward
decision crossing.  At t=73 that crossing disappears and its observable
rescue exceeds its useful loss, so it becomes a valid action.  Formation 6
also becomes addressable at t=73, but its useful loss still exceeds its rescue
and it is correctly excluded.

## Decision

The frozen V97 set is not temporally faithful: it omits a newly safe,
positive-net action at t=73.  This is a concrete reason to test causal online
re-selection, rather than tuning a coverage percentage or hard-coding an X36
formation ID.

V99 should recompute the same threshold-free positive-net rule after each
local update and before that step's fusion.  Each later decision must be based
on the posterior produced by the preceding V99 action; the V98 t=74 diagnostic
set is evidence of migration, not a schedule to replay.  V98 itself remains
development evidence and makes no tracking-performance claim.
