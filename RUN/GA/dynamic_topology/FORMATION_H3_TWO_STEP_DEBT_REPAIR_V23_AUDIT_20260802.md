# Formation H=3 two-step debt-repair v23 audit

## Verdict

v23 passed its protocol, provenance, reproduction, execution, and safety
instrumentation checks but failed the strong-headroom gate.  It establishes
that a second action can repay part of an earlier debt and that action effects
are sequence dependent.  It does not establish a trainable safe oracle.

## Contract checks

- generation commit: `768e77fdaf30921f697e7e979d0012117b30fa3e`;
- opened state: `m24-formation-fov / seed 211 / t=72`;
- frozen beam: six role-specific first-step prefixes crossed with 18
  structured second-step repairs; the third step remained reference;
- all six prefix-plus-reference controls reproduced v22 within `5e-6`
  percentage points;
- all 108 sequences passed the physical, payload, exact-execution,
  rolling-B3, truth-use, repair, emergency, and infeasibility checks;
- strict feasibility required all six targets to be nonnegative;
- strong headroom additionally required at least `3%` mean tracking gain.

## Main outcomes

| Role | First step | Second step | Mean | Min. formation | Worst sensor | Consensus | Attempted | Delivered | Strict |
|:--|:--|:--|--:|--:|--:|--:|--:|--:|:--:|
| Weak safe oracle | `[1,4,1,1]` | reference | +0.024% | 0 | +0.100% | +0.006% | +0.998% | +0.163% | yes |
| Closest strong boundary | `[1,1,2,2]` | `[1,4,1,1]` | +2.908% | 0 | -0.002% | +0.105% | -0.554% | -0.580% | no |
| Minimum high-return consensus debt | `[1,4,3,1]` | `[1,1,4,4]` | +5.371% | 0 | +0.099% | -4.529% | +0.273% | -0.595% | no |
| High-return, low delivered debt | `[1,4,3,1]` | `[1,1,2,4]` | +6.019% | 0 | +0.099% | -5.527% | +0.739% | -0.109% | no |
| Best mean | `[1,4,3,1]` | `[1,4,1,4]` | +7.956% | 0 | +0.122% | -6.491% | +0.157% | -0.717% | no |

The strict count was `2/108`, including the reference.  The strict
nonreference count was `1/107`, and the strong-safe count was `0/108`.

## Research finding

The second action is not an additive correction.  For a reference or weak
safe prefix, formation 4 can create positive consensus credit at a byte cost.
After the `[1,1,3,4]` high-return prefix, the same formation-4 action instead
increases consensus debt.  After `[1,4,3,1]`, a stronger formation-4 action
repays roughly half the original consensus debt while preserving tracking and
attempted-byte gains.  The sign and magnitude therefore depend on the graph,
the first action, and the resulting posterior state.

This is evidence for a sequence-conditioned graph value function with
separate target heads, not evidence that such a model is already trainable.
The current scalar posterior proxy had two true positives, no false
positives, 105 false negatives, and `0.019` action agreement.  It is too
conservative and cannot rank the safe boundary.

## Next decision

Run one final terminal-repair mechanism probe.  Reuse the saved v21 terminal
screen rather than repeating it, and retain its closest sequence as an exact
reproduction control.  Expand only four v23 two-step prefixes: the closest
strong-boundary prefix and three high-return prefixes spanning minimum
consensus debt, minimum delivered-byte debt, and maximum tracking headroom.
Cross each with the same structured 18-vector repair grid at the third step.

If this terminal grid finds no strictly feasible sequence with at least `3%`
mean gain, stop expanding the present three-step mode family and redesign the
action representation or horizon before any GNN training.  Seeds 223/227,
X36, and final seeds remain unopened.

## Evidence boundary

All findings are local to the opened seed-211/time-72 development state.
They authorize only the specified terminal mechanism probe, not validation,
generalization, or paper-level performance claims.
