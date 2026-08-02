# V29 temporal cross-edge suspension audit

## Decision

**The frozen v29 primary gate fails with `0/15` strong candidates.** The best action reaches `1.8145%` mean tracking gain, `0.1855` percentage points below the preregistered `2.0%` threshold. Do not open another M24 state, train a GNN, or run X36/X48 under v29.

- Contract: `formation-temporal-cross-edge-suspension-v29-audit-v1`
- Source generation commit: `7e6e61bcc6efc8d85de5dc4c6e3f152dac06d481`
- Source cache SHA-256: `60dfbf2615181cde046af15f42bba37c415ea0034cb7ce53685b79042bfaf762`
- Preset / seed / time: `m24-formation-fov / 211 / 72`
- Return times: `[72 73 74]`
- Best action: `suspend-f2-f3-f4`
- New tracking state opened by audit: `0`

## What the best action establishes

| Suspended formations | Mean gain | Worst-sensor gain | Formation gains | Window consensus | Final consensus | Attempted-byte saving | Mean card. error |
|:--|--:|--:|:--|--:|--:|--:|--:|
| `[2 3 4]` | +1.8145% | +6.6642% | `[0 2.9988 2.5658 1.5955]` | +5.7924% | -7.9945% | +1.2046% | 2.708333 -> 2.597222 |

The best subset passes posterior safety, rolling-B3, worst-sensor, minimum-formation, window-consensus, and byte-saving gates. It fails only the frozen mean-gain threshold. This is stronger mechanism evidence than v28, but it is still a failed gate rather than a near-pass that can be rounded upward.

## Temporal diagnosis

| Time | Reference tracking | Best tracking | Tracking gain | Reference consensus | Best consensus | Consensus gain |
|--:|--:|--:|--:|--:|--:|--:|
| 72 | 58.481288 | 53.680709 | +8.209% | 43.391245 | 33.141132 | +23.623% |
| 73 | 62.128070 | 62.074444 | +0.086% | 40.150107 | 40.191338 | -0.103% |
| 74 | 61.426530 | 62.977767 | -2.525% | 38.948541 | 42.062296 | -7.995% |

Suspension produces a large first-step improvement, but two plain reference-recovery steps do not preserve it: tracking advantage decays and terminal consensus becomes worse than reference. Therefore simply extending suspension or treating reference recovery as automatically safe is not justified.

## Frozen v30 direction

The next upper-bound probe should test a **two-phase isolate-and-reconnect controller** on this same opened t=72 state. Phase A uses the v29 existence-safe suspension bank. Phase B must recompute a causal recovery route from the then-current posterior and restore cross-formation information through a compatible gateway or bounded recovery pulse, rather than blindly returning to the fixed reference graph. The recovery bank and all safety gates must be frozen before outcomes are run. A GNN remains unauthorized until a source-only controller clears the same-state strong gate and then survives additional opened states.

## Evidence boundary

This audit reuses only the frozen v29 primary M24 screen. The outcomes may motivate a preregistered two-phase controller on the same opened state, but cannot relax the failed v29 gate, select a deployed action with truth, open another state, train a GNN, or support validation or generalization claims.
