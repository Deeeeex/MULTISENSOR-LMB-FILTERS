# M24 graph-option behavior H=3 return source

- Protocol: `m24-label-set-simulator-policy-redesign-v1`
- Contract: `m24-label-set-graph-option-behavior-source-shard-v1`
- Seed: `27`
- Times: `[78 79 80 81]`
- H=3 target-time union: `[78 79 80 81 82 83]`
- Behavior action: `fixed-counter-clockwise`
- Source cache SHA-256: `dcb48762473fcb39497aee3a038b176035005f1c25d2f853137e5443c65d6939`
- Behavior seconds: `273.876`
- Truth / repair / emergency / infeasible: `0 / 0 / 0 / 0`

- Selected sensor / formation B3: `1 / 1`
- Restart reference hashes stored: `1`

| Time | Mean labels | Max labels | Mean GM | Max GM |
|--:|--:|--:|--:|--:|
| 78 | 16.00 | 16 | 296.71 | 312 |
| 79 | 15.96 | 16 | 299.17 | 312 |
| 80 | 15.92 | 16 | 300.25 | 312 |
| 81 | 16.00 | 16 | 294.96 | 316 |

## Boundary

This shard contains deployment-observable predecision states from the opened M24 training split under one registered deployable behavior policy. It stores no truth-derived target and opens no development, held-out M24 or X36 outcome.
