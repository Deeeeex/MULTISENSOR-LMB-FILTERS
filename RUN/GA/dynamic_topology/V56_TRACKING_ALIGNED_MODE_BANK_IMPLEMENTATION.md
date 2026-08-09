# V56 tracking-aligned local mode bank

## Implemented action semantics

The V56 local bank no longer represents every intervention as a trust value.
It exposes three explicit action kinds for each formation:

- `reference-or-recover`: execute the registered route; when prior history
  contains a protected formation, this same graph performs recovery;
- `source-trust`: replace one formation's source/trust realization with one
  of the existing 0.30, 0.50, or 0.70 modes; and
- `protect`: withhold that formation's current cross-formation input for one
  step, but only when the current retention and rolling-connectivity
  projection accepts it.

Using the reference as recovery avoids emitting two actions with identical
current graphs.  The graph model receives route-history overlap, so it can
distinguish an ordinary reference step from restoration after protection.

## Shared safety and payload contract

The source/trust and protect actions are required to share the exact same
registered reference adjacency and fusion weights.  Every payload is charged
from the current sender posterior sizes under one common ledger.  The runner
executes only actions that are both within the reference payload and accepted
by the current-state posterior/retention projection.  Truth and future
measurements remain target-only quantities.

## Initial compatibility result

The merged bank was constructed on the historical M24 formation-FoV seed 211
state at time 60:

| Quantity | Result |
|:--|--:|
| Formations | 4 |
| Total local actions | 17 |
| Modes per formation | 5 |
| Safe protect modes | 3 / 4 |
| Node feature shape | 4 x 24 |
| Formation-edge feature shape | 4 x 4 x 35 |
| Formation-mode feature shape | 4 x 5 x 20 |

The previously evaluated source/trust actions alone provide only 1.119%
strict constrained oracle gain on this state.  This is not a V56 result: the
new protect outcomes are still absent.  It shows why the next gate must run
the merged bank and measure its H-step tracking headroom before any GNN is
trained.

## Next gate

Generate current-version reference caches and run the merged local bank on
one M24 and one X36 development case.  Aggregate oracle promotion requires
at least 5% constrained mean gain at both scales and a positive constrained
action in at least two thirds of opened states.  A failed scale must trigger
action-space redesign, not a larger predictor.
