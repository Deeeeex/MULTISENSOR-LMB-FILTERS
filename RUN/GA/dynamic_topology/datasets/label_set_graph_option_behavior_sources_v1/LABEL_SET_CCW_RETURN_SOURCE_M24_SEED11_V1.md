# M24 graph-option behavior H=3 return source

- Protocol: `m24-label-set-simulator-policy-redesign-v1`
- Contract: `m24-label-set-graph-option-behavior-source-shard-v1`
- Seed: `11`
- Times: `[78 79 80 81]`
- H=3 target-time union: `[78 79 80 81 82 83]`
- Behavior action: `fixed-counter-clockwise`
- Source cache SHA-256: `c0b67093bc055d23a920f2bf695108e74084acf7068e6ad19093645674c3fdef`
- Behavior seconds: `271.487`
- Truth / repair / emergency / infeasible: `0 / 0 / 0 / 0`

- Selected sensor / formation B3: `1 / 1`
- Restart reference hashes stored: `1`

| Time | Mean labels | Max labels | Mean GM | Max GM |
|--:|--:|--:|--:|--:|
| 78 | 15.83 | 16 | 298.25 | 316 |
| 79 | 15.62 | 16 | 291.58 | 308 |
| 80 | 15.88 | 16 | 295.75 | 308 |
| 81 | 15.83 | 16 | 297.71 | 312 |

## Boundary

This shard contains deployment-observable predecision states from the opened M24 training split under one registered deployable behavior policy. It stores no truth-derived target and opens no development, held-out M24 or X36 outcome.
