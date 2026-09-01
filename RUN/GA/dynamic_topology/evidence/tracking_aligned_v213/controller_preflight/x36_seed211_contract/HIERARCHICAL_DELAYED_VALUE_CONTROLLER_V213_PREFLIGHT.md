# V213 hierarchical delayed-value controller preflight

- Preflight passed: `1`
- Base policy / H: `online-positive-net-addressable-payload-v99-v1 / 3`
- Formation rule: `need >= 0.90 max`, at most `2`
- Mode cap / total cap: `3 per mode / 20`
- Static attempted-byte reserve: `1.0%`
- Registered scene presets: `17`
- New outcomes / fit / calibration: `0 / 0 / 0`

| Time | Need-selected formations | Label / release / total | Teacher selected | Feature bytes | Contract |
|--:|:--|:--|:--:|--:|:--:|
| 73 | `6 1` | 16 / 1 / 17 | 1 | 3328 B | 1 |
| 76 | `1` | 8 / 1 / 9 | 0 | 1664 B | 1 |
| 77 | `1` | 8 / 1 / 9 | 1 | 1664 B | 1 |

## Frozen risk budget on semantic anchors

| Action | Minimum target margin | Static saving | Admit | Expected |
|:--|--:|--:|:--:|:--:|
| `release-t72-f5` | -6.197% | +4.325% | 0 | 0 |
| `delete-t73-f6-refresh` | +0.818% | +4.544% | 1 | 1 |
| `delete-t76-f3-repair` | -4.374% | +1.419% | 0 | 0 |
| `delete-t77-f1-map-repair` | +0.909% | +2.708% | 1 | 1 |

## Exact short-horizon replay

V212 still reproduces the no-op and known t=73 F6 candidate branches with zero E-OSPA, RMSE and consistency difference and exact attempted bytes: `1`.

## Evidence boundary

V213 freezes a scale-aware, truth-free proposal cap, explicit release feasibility, one-page semantic cooldown, H=3 vector risk budget, propagation-feature communication charge and disjoint trajectory split before opening seeds 1301 and above.  The seed-211 anchors test only proposal membership, risk semantics and exact replay.  They cannot train, calibrate, validate or support a new tracking-performance claim.
