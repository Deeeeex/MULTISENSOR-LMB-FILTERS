# V187 formation-coordinator aggregation finding

## Paired X36 result

V187 is the current best **balanced development candidate** on the opened
X36 seed-211, t=72, H=8 window.  It preserves the V179 trajectory through
t=78 and replaces only the t=79 formation-5 repair with one
coordinator-selected common complete-label transfer.

| Arm | Mean E-OSPA | Mean RMSE | Window / terminal consensus gain | Minimum formation RMSE gain | Attempted bytes | Byte saving |
|:--|--:|--:|--:|--:|--:|--:|
| Static full payload | 84.037151 | 59.967347 | -- | -- | 28,578,864 | -- |
| V179 independent F5 repair | 74.573180 | 53.566866 | +9.823% / +13.868% | -2.351% | 28,586,472 | -0.027% |
| V187 coordinator aggregation | 74.678760 | 53.540189 | +9.834% / +13.959% | -1.050% | 28,533,176 | +0.160% |

Relative to V179, V187 gives up 0.105580 E-OSPA points while improving mean
RMSE by 0.026677, the weakest-formation RMSE gate by 1.301 percentage points,
and attempted communication by 53,296 bytes.  Relative to the matched static
reference it simultaneously improves mean E-OSPA by 11.136%, mean RMSE by
10.718%, window consensus by 9.834%, terminal consensus by 13.959%, and
attempted bytes by 0.160%.

## Decision

V187 does **not** pass the registered gate because the weakest formation RMSE
remains negative at -1.050%.  It therefore remains development evidence, not
validation or a paper result.  It nevertheless replaces V179 as the current
best balanced candidate because it is the first member of this action family
to improve all four network-level objectives relative to static: E-OSPA,
RMSE, consensus, and attempted communication.  V179 remains the lowest
mean-E-OSPA candidate.

The result validates coordinator aggregation as an execution layer, not the
current trigger schedule as a deployable policy.  A single t=79 common action
recovers only part of the formation-5 RMSE debt accumulated earlier in the
rollout.  Coordinating both t=78 and t=79 was already rejected by V185 because
the first common action shifts the next state and invalidates the second
opened-state choice.  Further fixed-time action substitution is therefore the
wrong next step.

## Next method boundary

The next method must separate three components:

1. retain a deterministic strongly connected full-posterior backbone for
   global KLA information flow;
2. use an online, formation-index- and time-index-free risk trigger to open
   residual receiver--sender--label repair capacity before the local RMSE debt
   becomes terminal;
3. select actions on the states actually visited after earlier repairs, with
   the static backbone as a conservative fallback and every synopsis, request,
   and complete-label response charged.

The first learner should remain an interpretable scale-normalized ranker or
shallow model.  A GNN is justified only after this edge--label action family
shows paired headroom on both M24 and X36 and leaves a repeatable interaction
residual.  Validation must then use unseen seeds and the frozen convoy, relay,
merge--split, and curved-corridor geometries; the crossing scene remains a
stress test.

## Evidence boundary

The selection rule, t=79 trigger, and preflight were developed from opened
seed-211 X36 states.  No cross-seed, M24, or cross-geometry generalization is
claimed.  The formal result is stored under
`formation_coordinator_aggregation/x36_t72_h8/` and reuses the exact frozen
static reference outcome from V179.
