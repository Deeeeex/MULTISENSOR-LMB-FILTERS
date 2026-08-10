# V72 receiver-domain transport paired H=3 result

| Scale | Time | Formations | Mean tracking | Worst sensor | Minimum formation | Window consensus | Terminal consensus | Attempted bytes | B3 | Strong |
|:--|--:|:--|--:|--:|--:|--:|--:|--:|:--:|:--:|
| m24-formation-fov-merge-split | 80 | `3` | +0.473% | +0.000% | +0.000% | -0.528% | -1.061% | -0.158% | 1 | 0 |
| x36-formation-fov-merge-split | 52 | `[4 5]` | -0.290% | +0.000% | -2.399% | +0.142% | -0.031% | -0.013% | 1 | 0 |

- Both scales pass the frozen strong gate: `0`
- Tracking outcome read: `1`
- Validation claim allowed: `0`

## Evidence boundary

V72 opens exactly two already-frozen V71 source states: M24 t=80 and X36 t=52, seed 1401. Reference and candidate share the same predecision posterior, measurements, delivery draws, filter RNG, and current-physical-tree reference. The candidate executes one V71 fixed-message transport route at the anchor; both arms recompute the reference for the following two steps. Truth and future measurements score the paired H=3 outcome only after route construction. This is opened development evidence, not validation.
