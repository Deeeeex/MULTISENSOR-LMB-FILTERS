# V88 time-expanded path-selective routing design

## Decision inherited from V87

V87 established that a gateway's second-hop value is receiver-specific.
The M24 primary state has four positive paths and one harmful path, whereas
all five X36 paths are positive.  V88 therefore changes the action unit from
an entire gateway formation to an individual source--gateway--receiver path.

## Frozen primary action

The temporal sequence remains `acquire -> broadcast -> reference`.

1. Acquire replaces the gateway's registered `0.05` residual sender with
   the V84 source for one KLA round.
2. Broadcast promotes the acquired gateway into the existing `0.70` slot
   only for receivers that pass the V87 path gate.  The displaced dominant
   sender remains at `0.05`; an excluded receiver keeps its reference row.
3. The final step returns every row to the current physical reference.

The masks are frozen before paired tracking is opened:

- M24 t=104: gateway 11 broadcasts to `[7, 9, 10, 12]`; receiver 8 remains
  unchanged.
- X36 t=112: gateway 23 broadcasts to `[19, 20, 21, 22, 24]`.

Every receiver retains the same positive-weight multiset and directed
message count as the reference.  Both selected sequences must pass rolling
B3 at sensor and formation levels.

## Primary paired gate

Reference and candidate share the same predecision posterior, measurements,
delivery draws and filter RNG over H=3.  A scale passes only with at least
`5%` mean tracking gain, nonnegative worst-sensor, minimum-formation, window
consensus and terminal-consensus gains, no more than `1%` attempted-byte
regression, exact message parity and rolling-B3.

This paired test is opened development evidence.  A positive result would
authorize construction of a causal full-episode policy that recomputes path
values online; it would not by itself establish temporal or scene-style
generalization.
