# V35 debt-coverage staggered-recovery audit

## Decision

**The frozen v35 primary gate passes with `1/1` strong candidate.** The causal runtime trace is `[2,3,4] -> [2,4] -> [3]`; all registered tracking, tail, consensus, communication, execution, and rolling-connectivity gates pass.

- Contract: `formation-staggered-recovery-v35-audit-v1`
- Source generation commit: `00a0634ca9fa84c5ab3d933501353dc1740d3c7c`
- Source preflight commit: `00a0634ca9fa84c5ab3d933501353dc1740d3c7c`
- Source cache SHA-256: `60dfbf2615181cde046af15f42bba37c415ea0034cb7ce53685b79042bfaf762`
- Preset / seed / times: `m24-formation-fov / 211 / [72 73 74]`

## Aggregate result

| Mean tracking | Worst sensor | Formation gains | Window consensus | Terminal consensus | Byte saving | Mean card. error |
|--:|--:|:--|--:|--:|--:|--:|
| +4.2952% | +19.2311% | `[0 7.4418 4.8241 4.3878]` | +15.4265% | +1.3945% | +3.5878% | 2.708333 -> 2.430556 |

## Temporal mechanism

| Time | Selected suspension | Reference tracking | Candidate tracking | Tracking gain | Reference consensus | Candidate consensus | Consensus gain | Byte saving |
|--:|:--|--:|--:|--:|--:|--:|--:|--:|
| 72 | `[2 3 4]` | 58.481288 | 53.680709 | +8.209% | 43.391245 | 33.141132 | +23.623% | +8.081% |
| 73 | `[2 4]` | 62.128070 | 59.219432 | +4.682% | 40.150107 | 32.047463 | +20.181% | +5.369% |
| 74 | `3` | 61.426530 | 61.316979 | +0.178% | 38.948541 | 38.405412 | +1.394% | -2.611% |

## Comparison with v30

V30 kept formations 2, 3, and 4 suspended for two steps and then returned abruptly, giving `-1.5947%` terminal-consensus gain. V35 restores formation 3 at t=73, restores the older f2/f4 inputs at t=74, and lets the live debt rule protect f3 again. Terminal consensus becomes `+1.3945%`, a `+2.9892` percentage-point repair, while mean tracking remains `+4.2952%` and attempted-byte saving remains `+3.5878%`.

## Next decision

A separate clean protocol may now preflight and screen additional already-opened M24 development states under the unchanged v35 controller. Do not train a GNN or open X36/X48 until the causal headroom recurs beyond this single state.

## Evidence boundary

This audit freezes one paired result on the already-opened M24 seed-211 t=72 state. It establishes local causal headroom for the fixed v35 controller and authorizes a separately frozen source-only preflight on additional already-opened M24 development states. It does not authorize GNN training, X36, X48, reserved seeds, validation, or paper-level generalization.
