# V30 causal retention-debt audit

## Decision

**The frozen v30 primary gate fails with `0/1` strong candidates.** The controller passes every registered gate except terminal consensus, which changes by `-1.5947%`. Do not open t=60, train a GNN, or run X36/X48 under v30.

- Contract: `formation-retention-debt-v30-audit-v1`
- Source generation commit: `985120f7945205357a1643574d38e62feaf14063`
- Source cache SHA-256: `60dfbf2615181cde046af15f42bba37c415ea0034cb7ce53685b79042bfaf762`
- Preset / seed / time: `m24-formation-fov / 211 / 72`
- Return times: `[72 73 74]`
- New tracking state opened by audit: `0`

## Aggregate result

| Initial action | Mean tracking | Worst sensor | Formation gains | Window consensus | Terminal consensus | Byte saving | Mean card. error |
|:--|--:|--:|:--|--:|--:|--:|--:|
| `suspend-f2-f3-f4` | +4.3409% | +19.2311% | `[0 7.4418 5.0327 4.3878]` | +15.6057% | -1.5947% | +3.5878% | 2.708333 -> 2.430556 |

The two-step adaptive suspension produces material tracking, tail, cardinality, window-consensus, and communication gains. The strict decision remains a failure because the final consensus value is not allowed to regress.

## Temporal result

| Time | Reference tracking | Controller tracking | Tracking gain | Reference consensus | Controller consensus | Consensus gain | Byte saving |
|--:|--:|--:|--:|--:|--:|--:|--:|
| 72 | 58.481288 | 53.680709 | +8.209% | 43.391245 | 33.141132 | +23.623% | +8.081% |
| 73 | 62.128070 | 58.725677 | +5.476% | 40.150107 | 30.663681 | +23.627% | +8.204% |
| 74 | 61.426530 | 61.727488 | -0.490% | 38.948541 | 39.569662 | -1.595% | -5.295% |

## Controller trace

| Time | Debt fractions | Thresholds | Available singles | Requested | Selected | Reference fallback |
|--:|:--|:--|:--|:--|:--|:--:|
| 72 | `[0.0068703 0.057005 0.02344 0.046506]` | `[0.02 0.02 0.02 0.02]` | `[true true true true]` | `[2 3 4]` | `[2 3 4]` | 0 |
| 73 | `[0.010305 0.052298 0.01237 0.049228]` | `[0.02 0.01 0.01 0.01]` | `[true true true true]` | `[2 3 4]` | `[2 3 4]` | 0 |
| 74 | `[0.0074851 NaN NaN NaN]` | `[0.02 0.01 0.01 0.01]` | `[true false false false]` | `[]` | `[]` | 1 |

At t=73, the lower 1% off-threshold keeps formations 2, 3, and 4 suspended. At t=74, rolling-B3 makes those three single suspensions unavailable, so the controller returns to reference. Compared with v29, this reduces the terminal-consensus deficit from -7.995% to -1.595%, but does not eliminate it.

## Next admissible direction

Do not relax the terminal gate or open another state. A separate v31 no-outcome preflight may use the same opened trajectory to construct a bounded reconnect action at the forced t=74 return. That action must be chosen from the then-current posterior, pass the same label-retention and rolling-connectivity projection, and keep reference fallback. Its route family and thresholds must be frozen before the t=72 outcome is rerun.

## Evidence boundary

This audit reads only the frozen v30 primary M24 screen. It cannot relax the terminal-consensus gate, open t=60 or another M24 state, train a GNN, run X36/X48, or support validation and generalization claims. The same opened t=72 trajectory may motivate a separately frozen, source-only recovery controller.
