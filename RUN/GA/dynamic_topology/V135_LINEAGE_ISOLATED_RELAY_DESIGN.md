# V135 lineage-isolated relay: method decision

## Problem isolated by V134

V134 showed real short-horizon headroom when selected cross-formation inputs
were withheld from a local estimator, but the altered posterior was then sent
to other nodes.  Those nodes could converge internally while converging to a
common biased posterior.  Consequently, structural KLA recovery and better
network consensus did not imply better tracking.

## Causal state split

Every sensor maintains two LMB states from the same local measurements:

- **Working state W:** produces the sensor's tracking output and may abstain
  from consuming a selected cross-formation input.
- **Relay state R:** always applies the frozen reference route, realized packet
  delivery and nominal KLA weights.  R is the only state transmitted.

There is exactly one ordinary full posterior per reference message
opportunity.  W is never transmitted, and V135 adds no auxiliary payload or
alternative-arm state.  A protected W therefore cannot contaminate a future
sender.  On the first unprotected page, W rejoins the causal R state.

## First screen and claim boundary

The first screen asks only whether state-flow isolation preserves at least 5%
intervention E-OSPA headroom while eliminating full-window, mature-window,
sensor and formation regressions on both M24 and X36.  Attempted bytes must be
identical to the full-posterior reference.  This phase makes no communication
saving claim; payload compression is authorized only after this mechanism
passes both scales.

M24 uses the frozen V134 rank-1/rank-3 pair action because it had the strongest
useful aggregate headroom without the severe long-tail regression of the
all-formation V134 action.  X36 uses the all-ranked-formation staggered action
as a direct scale stress test.  Results that fail the joint gate remain in the
repository only and are not added to the main Lark document.
