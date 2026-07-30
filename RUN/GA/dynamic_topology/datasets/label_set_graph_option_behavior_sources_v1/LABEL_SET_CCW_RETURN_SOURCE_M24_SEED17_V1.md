# M24 graph-option behavior H=3 return source

- Protocol: `m24-label-set-simulator-policy-redesign-v1`
- Contract: `m24-label-set-graph-option-behavior-source-shard-v1`
- Seed: `17`
- Times: `[78 79 80 81]`
- H=3 target-time union: `[78 79 80 81 82 83]`
- Behavior action: `fixed-counter-clockwise`
- Source cache SHA-256: `5e6649a27b42cefbac5e2f4cb88c0c0eed2fc81f84a21902dddf6d028be7a605`
- Behavior seconds: `271.447`
- Truth / repair / emergency / infeasible: `0 / 0 / 0 / 0`

- Selected sensor / formation B3: `1 / 1`
- Restart reference hashes stored: `1`

| Time | Mean labels | Max labels | Mean GM | Max GM |
|--:|--:|--:|--:|--:|
| 78 | 16.00 | 16 | 293.92 | 308 |
| 79 | 16.00 | 16 | 287.04 | 304 |
| 80 | 15.96 | 16 | 297.83 | 312 |
| 81 | 15.88 | 16 | 297.96 | 312 |

## Boundary

This shard contains deployment-observable predecision states from the opened M24 training split under one registered deployable behavior policy. It stores no truth-derived target and opens no development, held-out M24 or X36 outcome.
