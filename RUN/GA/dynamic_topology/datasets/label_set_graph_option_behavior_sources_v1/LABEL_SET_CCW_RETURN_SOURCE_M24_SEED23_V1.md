# M24 graph-option behavior H=3 return source

- Protocol: `m24-label-set-simulator-policy-redesign-v1`
- Contract: `m24-label-set-graph-option-behavior-source-shard-v1`
- Seed: `23`
- Times: `[78 79 80 81]`
- H=3 target-time union: `[78 79 80 81 82 83]`
- Behavior action: `fixed-counter-clockwise`
- Source cache SHA-256: `0508cbc0b4ec2647fa66e1ab227da5574a62f4463f064e098e0db8d6d2239f0b`
- Behavior seconds: `277.177`
- Truth / repair / emergency / infeasible: `0 / 0 / 0 / 0`

- Selected sensor / formation B3: `1 / 1`
- Restart reference hashes stored: `1`

| Time | Mean labels | Max labels | Mean GM | Max GM |
|--:|--:|--:|--:|--:|
| 78 | 16.00 | 16 | 296.33 | 304 |
| 79 | 16.00 | 16 | 296.17 | 304 |
| 80 | 16.00 | 16 | 300.33 | 312 |
| 81 | 16.00 | 16 | 304.17 | 312 |

## Boundary

This shard contains deployment-observable predecision states from the opened M24 training split under one registered deployable behavior policy. It stores no truth-derived target and opens no development, held-out M24 or X36 outcome.
