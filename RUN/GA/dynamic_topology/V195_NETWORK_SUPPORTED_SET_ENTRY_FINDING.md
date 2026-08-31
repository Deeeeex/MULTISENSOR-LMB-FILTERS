# V195 single-frame network support finding

## Question

V194 releases a proposed formation omission when that omission makes an
unsupported label enter a receiver's marginal-MAP set.  Its receiver-only
definition is over-conservative after the first recursive page because a
single missed detection can make an established label appear unsupported.
V195 asks whether current positive association at an active cross-formation
reference sender is sufficient to remove those false alarms.

## Causal replay result

The replay uses the exact observable states visited by the paired V194 arm.
It changes only the support definition and uses no target truth, future
measurement, or tracking outcome.

| Scale | First-page release retained | Later receiver-only releases | Later network-supported releases |
|:--|:--:|--:|--:|
| M24 | yes (`F4`) | 4 | 2 |
| X36 | yes (`F2`, `F5`) | 3 | 3 |

On M24, sender support removes the unnecessary `F1/F2` releases at `t=105`,
but both return at `t=106`.  On X36, it approximately halves the unsupported
entry counts and risks, yet leaves every release decision unchanged.

## Decision

Single-frame network support is informative but not a sufficient scheduler.
It should not be promoted to a recursive candidate because it does not solve
the X36 failure mode.  The next test should require consecutive absence over a
short causal label-support window: a full-posterior release is allowed only
when neither the receiver nor a currently contributing cross-formation sender
has positively associated the entering label during that window.  This turns
V194's one-frame absence test into an explicit dwell certificate and directly
targets transient missed detections.

