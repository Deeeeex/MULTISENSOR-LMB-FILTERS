# V71 receiver-domain transport projection anchors

| Scale | Time | Nominated formations | Selected formations | Slots | Feasible subsets | Net mass / network | Messages | Rolling B3 | Route |
|:--|--:|:--|:--|--:|--:|--:|:--|:--|:--|
| m24-formation-fov-merge-split | 80 | `3` | `3` | 2 | 1 / 1 | 0.785% | `48 / 48` | `1` | `constructed` |
| x36-formation-fov-merge-split | 52 | `[4 5]` | `[4 5]` | 4 | 3 / 3 | 0.997% | `72 / 72` | `1` | `constructed` |

## Selected receiver slots

### m24-formation-fov-merge-split / t=80

| Receiver | Incumbent sender | Candidate sender | Weight | Gain | Harm | Net | Up / down crossings |
|--:|--:|--:|--:|--:|--:|--:|:--|
| 14 | 10 | 21 | 0.05 | 0.728% | 0.001% | 0.727% | `3 / 0` |
| 17 | 19 | 3 | 0.05 | 0.128% | 0.070% | 0.058% | `1 / 0` |

### x36-formation-fov-merge-split / t=52

| Receiver | Incumbent sender | Candidate sender | Weight | Gain | Harm | Net | Up / down crossings |
|--:|--:|--:|--:|--:|--:|--:|:--|
| 20 | 16 | 34 | 0.05 | 0.249% | 0.111% | 0.138% | `1 / 0` |
| 23 | 25 | 35 | 0.05 | 0.130% | 0.013% | 0.117% | `1 / 0` |
| 26 | 22 | 35 | 0.05 | 0.725% | 0.023% | 0.701% | `4 / 0` |
| 29 | 31 | 17 | 0.05 | 0.049% | 0.009% | 0.041% | `0 / 0` |

## Gate summary

- Both anchors constructed a non-reference route: `1`
- Exact message and row-weight parity at both anchors: `1`
- Rolling-B3 sensor and formation connectivity at both anchors: `1`
- Tracking outcome read: `0`
- Route executed: `0`

## Evidence boundary

V71 converts frozen V70 transport nominations into one current route. Each affected receiver replaces at most one registered cross-formation residual sender while retaining the same row weight and message count. Candidate formation bundles are selected by total receiver-domain net mass and must satisfy current physicality plus rolling-B3 sensor and formation connectivity. The projection reads no truth, future measurement, future link outcome, tracking outcome, or learned prediction.
