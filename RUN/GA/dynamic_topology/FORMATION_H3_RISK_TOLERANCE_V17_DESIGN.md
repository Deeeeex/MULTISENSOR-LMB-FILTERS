# Formation H=3 risk-tolerance diagnostic v17

## Question

The fixed M24 v13 bank contains many actions that improve network-mean
tracking, but the strict oracle rejects most of them because at least one
formation, sensor, consensus, or communication target is negative.  v14-v16
show that changing event timing, restoring the current joint projector, and
holding a known action for three steps do not recover strong strict headroom.

This audit asks whether the remaining bottleneck is primarily the exact
samplewise zero-regression rule or the value available in the 19-action bank.
It does not rerun the filter, change actions, or define a new deployment gate.

## Frozen opened-data scan

- preset: `m24-formation-fov`;
- already-opened states: seeds `[211, 223, 227]`, times `[60, 72]`;
- source: the frozen v13 singleton-plus-pair H=3 target matrices;
- objective: maximize network-mean tracking gain, with reference always
  available at zero;
- tolerance grid: `[0, 0.10, 0.25, 0.50, 1.00, 2.00]` percentage points;
- unchanged runtime requirements: exact first action, no truth use, no repair,
  no emergency/infeasibility, and all registered B3 checks passed.

Two descriptive selection rules are scanned:

1. `estimation-aux-tolerance-bytes-strict`: minimum-formation, worst-sensor,
   and consensus gains may be no lower than `-tau`, while attempted and
   delivered bytes must remain nonnegative;
2. `all-aux-tolerance`: all five auxiliary targets may be no lower than
   `-tau`.

At `tau=0`, both rules must reproduce the registered v13 strict gains exactly:
`[1.590662, 0.024472, 0, 0.516168, 0, 0.751288]%`.  Failure to reproduce this
row invalidates the audit.

## Interpretation rule

The existing M24 headroom gate—at least two positive states, at least one
`>=3%` state, and at least `+2%` mean gain—is reported only as a common ruler.
No tolerance row can pass a method or authorize a claim because the outcomes
are already known.

- If a small `tau` of at most `0.5%`, with bytes still strict, releases strong
  and mean headroom, a new protocol may study window-level/CVaR risk budgets
  with causal prediction, calibration, and reference fallback.
- If headroom appears only after communication regression is allowed, the
  current actions express a tracking-versus-payload trade-off rather than the
  required joint improvement.
- If even `tau=2%` lacks strong mean headroom, the current action bank is the
  primary bottleneck and must be expanded before value-model work.

This is privileged mechanism diagnosis only.  Seeds outside the six opened
states, X36, and all final seeds remain unopened.

## Result

The `tau=0` rows exactly reproduce the registered v13 strict gains.  Neither
diagnostic rule changes the oracle through `tau=0.5%`: the mean remains
`+0.480%` and no state reaches `3%`.  With communication still strict, even
`tau=2%` yields only `+0.928%` mean and zero strong states.  Allowing all five
auxiliary targets to fall by as much as `2%` yields `+1.857%` mean, still with
zero strong states and communication regression in three of six states.

A post-scan descriptive ceiling that ignores all auxiliary targets gives
`+2.407%` mean and one strong state.  That strong action gains `+5.988%` in
network-mean tracking but loses `-11.486%` in consensus.  The current bank
therefore contains local tracking value, but not a jointly safe action with
the required magnitude.

This rejects risk-threshold relaxation as the next method.  The actionable
hypothesis is now a finite-horizon sequence that can combine complementary
nonreference actions and repay formation/consensus risk before the end of the
window.  Such a sequence must be evaluated under the original terminal
six-target gate and exact per-step topology runtime checks; the descriptive
tolerances will not be carried into validation.
