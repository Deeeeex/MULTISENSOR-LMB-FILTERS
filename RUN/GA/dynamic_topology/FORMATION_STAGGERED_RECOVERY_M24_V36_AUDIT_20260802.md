# V36 multi-state M24 replication audit

## Decision

**The preregistered replication gate passes.** Two of three new states are strict-strong, all three improve mean tracking, worst-sensor tracking, window consensus, terminal consensus, and attempted bytes without a formation regression. The weak t=60 state misses only the 2% mean-gain threshold.

- Contract: `formation-staggered-recovery-m24-v36-audit-v1`
- Source generation commit: `1fe5090e4180621f6a5843f262fe84a1be7ba700`
- Source screen SHA-256: `af314a31799db440a245026246f7690cc6bf1ce028e141bdbcb0f840910abae9`
- Source preflight commit / SHA-256: `3fc4a0e280dea00bfc934e5a0fb8989f6887acb0 / 33aa6acece0b9c05144ba071304938d267eada280f38d335574e07f25a29d903`
- Preset / seed / anchors: `m24-formation-fov / 211 / [60 104 124]`

## Per-state result

| Anchor | Runtime suspension | Mean tracking | Worst sensor | Formation gains | Window consensus | Terminal consensus | Byte saving | Mean card. error | Strong |
|--:|:--|--:|--:|:--|--:|--:|--:|:--|:--:|
| 60 | `[2 4] -> [2 4] -> []` | +1.4017% | +11.9980% | `[0 2.3462 0 2.9353]` | +6.0074% | +1.3877% | +2.3108% | 3.250000 -> 3.138889 | 0 |
| 104 | `[1 2 4] -> [1 2] -> 4` | +10.3944% | +36.4080% | `[18.779 11.185 0 8.9231]` | +24.3372% | +8.8198% | +0.3468% | 2.208333 -> 1.569444 | 1 |
| 124 | `[2 3 4] -> [2 3] -> 4` | +7.7641% | +31.6245% | `[0 10.052 15.168 5.5993]` | +16.4216% | +2.6183% | +0.4051% | 3.430556 -> 2.736111 | 1 |

## Aggregate result

- Strict-strong states: `2 / 3`
- Median mean tracking gain: `+7.7641%`
- Minimum state mean tracking gain: `+1.4017%`
- Positive terminal-consensus states: `3 / 3`
- t=60 misses only the 2% mean gate: `1`

## Interpretation

Together with the separate t=72 primary result, the analytic controller now has positive local development evidence at four temporally separated M24 windows. V36 itself evaluates only the three newly opened windows and passes its frozen 2-of-3 gate. This supports testing whether the same source-side mechanism exists at X36; it is not yet cross-scale evidence.

The terminal step increases bytes at each new state, but earlier suspension yields a positive H=3 total saving. This confirms that the useful object is the full temporal route, not a claim that every individual step saves communication.

## Authorization boundary

- X36 source-only protocol: **authorized**.
- X36 outcome, GNN training, X48, reserved seeds, and validation: **not authorized**.

## Evidence boundary

This audit freezes three paired H=3 outcomes from already-opened M24 seed-211 states. The three actions and runtime traces were source-frozen before the outcome screen. The result establishes temporal development stability for the analytic controller and authorizes only an X36 source-only protocol. It does not establish cross-scale generalization and does not authorize X36 outcomes, GNN training, X48, reserved seeds, or validation claims.
