# Formation H=3 risk-tolerance sensitivity

- Contract: `formation-h3-risk-tolerance-sensitivity-v1`
- Generation commit: `6e70e27702994604b2bd374ed4a4684969fb344e`
- Source generation commit: `c32a0649834e0119b3993a95e9e441e6269c8c3b`
- Preset / seeds / times: `m24-formation-fov / [211 223 227] / [60 72]`
- Tolerance grid: `[0 0.1 0.25 0.5 1 2]` percentage points

## Aggregate oracle sensitivity

| Policy | Tau | Positive | Strong >=3% | Mean gain | Max selected debt | Comm. regressions | M24 gate |
|:--|--:|--:|--:|--:|--:|--:|:--|
| estimation-aux-tolerance-bytes-strict | 0.00 | 4/6 | 0/6 | +0.480% | -0.000% | 0/6 | FAIL |
| estimation-aux-tolerance-bytes-strict | 0.10 | 4/6 | 0/6 | +0.480% | -0.000% | 0/6 | FAIL |
| estimation-aux-tolerance-bytes-strict | 0.25 | 4/6 | 0/6 | +0.480% | -0.000% | 0/6 | FAIL |
| estimation-aux-tolerance-bytes-strict | 0.50 | 4/6 | 0/6 | +0.480% | -0.000% | 0/6 | FAIL |
| estimation-aux-tolerance-bytes-strict | 1.00 | 4/6 | 0/6 | +0.480% | -0.000% | 0/6 | FAIL |
| estimation-aux-tolerance-bytes-strict | 2.00 | 5/6 | 0/6 | +0.928% | 1.697% | 0/6 | FAIL |
| all-aux-tolerance | 0.00 | 4/6 | 0/6 | +0.480% | -0.000% | 0/6 | FAIL |
| all-aux-tolerance | 0.10 | 4/6 | 0/6 | +0.480% | -0.000% | 0/6 | FAIL |
| all-aux-tolerance | 0.25 | 4/6 | 0/6 | +0.480% | -0.000% | 0/6 | FAIL |
| all-aux-tolerance | 0.50 | 4/6 | 0/6 | +0.480% | -0.000% | 0/6 | FAIL |
| all-aux-tolerance | 1.00 | 6/6 | 0/6 | +1.778% | 0.887% | 4/6 | FAIL |
| all-aux-tolerance | 2.00 | 6/6 | 0/6 | +1.857% | 1.697% | 3/6 | FAIL |

## Selected actions

| Policy | Tau | Seed-time | Action | Gain | Six targets |
|:--|--:|:--|:--|--:|:--|
| estimation-aux-tolerance-bytes-strict | 0.00 | 211-60 | formations-1-2-dynamic-trust-0.30 | +1.591% | `+1.591 +0.000 +5.494 +2.807 +0.696 +0.742` |
| estimation-aux-tolerance-bytes-strict | 0.00 | 211-72 | formation-2-dynamic-trust-0.70 | +0.024% | `+0.024 +0.000 +0.100 +0.006 +0.998 +0.163` |
| estimation-aux-tolerance-bytes-strict | 0.00 | 223-60 | reference | +0.000% | `+0.000 +0.000 +0.000 +0.000 +0.000 +0.000` |
| estimation-aux-tolerance-bytes-strict | 0.00 | 223-72 | formations-1-2-dynamic-trust-0.30 | +0.516% | `+0.516 +0.000 +0.001 +1.882 +1.152 +1.183` |
| estimation-aux-tolerance-bytes-strict | 0.00 | 227-60 | reference | +0.000% | `+0.000 +0.000 +0.000 +0.000 +0.000 +0.000` |
| estimation-aux-tolerance-bytes-strict | 0.00 | 227-72 | formations-2-4-dynamic-trust-0.30 | +0.751% | `+0.751 +0.001 +0.003 +0.833 +2.479 +2.580` |
| estimation-aux-tolerance-bytes-strict | 0.25 | 211-60 | formations-1-2-dynamic-trust-0.30 | +1.591% | `+1.591 +0.000 +5.494 +2.807 +0.696 +0.742` |
| estimation-aux-tolerance-bytes-strict | 0.25 | 211-72 | formation-2-dynamic-trust-0.70 | +0.024% | `+0.024 +0.000 +0.100 +0.006 +0.998 +0.163` |
| estimation-aux-tolerance-bytes-strict | 0.25 | 223-60 | reference | +0.000% | `+0.000 +0.000 +0.000 +0.000 +0.000 +0.000` |
| estimation-aux-tolerance-bytes-strict | 0.25 | 223-72 | formations-1-2-dynamic-trust-0.30 | +0.516% | `+0.516 +0.000 +0.001 +1.882 +1.152 +1.183` |
| estimation-aux-tolerance-bytes-strict | 0.25 | 227-60 | reference | +0.000% | `+0.000 +0.000 +0.000 +0.000 +0.000 +0.000` |
| estimation-aux-tolerance-bytes-strict | 0.25 | 227-72 | formations-2-4-dynamic-trust-0.30 | +0.751% | `+0.751 +0.001 +0.003 +0.833 +2.479 +2.580` |
| estimation-aux-tolerance-bytes-strict | 0.50 | 211-60 | formations-1-2-dynamic-trust-0.30 | +1.591% | `+1.591 +0.000 +5.494 +2.807 +0.696 +0.742` |
| estimation-aux-tolerance-bytes-strict | 0.50 | 211-72 | formation-2-dynamic-trust-0.70 | +0.024% | `+0.024 +0.000 +0.100 +0.006 +0.998 +0.163` |
| estimation-aux-tolerance-bytes-strict | 0.50 | 223-60 | reference | +0.000% | `+0.000 +0.000 +0.000 +0.000 +0.000 +0.000` |
| estimation-aux-tolerance-bytes-strict | 0.50 | 223-72 | formations-1-2-dynamic-trust-0.30 | +0.516% | `+0.516 +0.000 +0.001 +1.882 +1.152 +1.183` |
| estimation-aux-tolerance-bytes-strict | 0.50 | 227-60 | reference | +0.000% | `+0.000 +0.000 +0.000 +0.000 +0.000 +0.000` |
| estimation-aux-tolerance-bytes-strict | 0.50 | 227-72 | formations-2-4-dynamic-trust-0.30 | +0.751% | `+0.751 +0.001 +0.003 +0.833 +2.479 +2.580` |
| estimation-aux-tolerance-bytes-strict | 1.00 | 211-60 | formations-1-2-dynamic-trust-0.30 | +1.591% | `+1.591 +0.000 +5.494 +2.807 +0.696 +0.742` |
| estimation-aux-tolerance-bytes-strict | 1.00 | 211-72 | formation-2-dynamic-trust-0.70 | +0.024% | `+0.024 +0.000 +0.100 +0.006 +0.998 +0.163` |
| estimation-aux-tolerance-bytes-strict | 1.00 | 223-60 | reference | +0.000% | `+0.000 +0.000 +0.000 +0.000 +0.000 +0.000` |
| estimation-aux-tolerance-bytes-strict | 1.00 | 223-72 | formations-1-2-dynamic-trust-0.30 | +0.516% | `+0.516 +0.000 +0.001 +1.882 +1.152 +1.183` |
| estimation-aux-tolerance-bytes-strict | 1.00 | 227-60 | reference | +0.000% | `+0.000 +0.000 +0.000 +0.000 +0.000 +0.000` |
| estimation-aux-tolerance-bytes-strict | 1.00 | 227-72 | formations-2-4-dynamic-trust-0.30 | +0.751% | `+0.751 +0.001 +0.003 +0.833 +2.479 +2.580` |
| all-aux-tolerance | 0.00 | 211-60 | formations-1-2-dynamic-trust-0.30 | +1.591% | `+1.591 +0.000 +5.494 +2.807 +0.696 +0.742` |
| all-aux-tolerance | 0.00 | 211-72 | formation-2-dynamic-trust-0.70 | +0.024% | `+0.024 +0.000 +0.100 +0.006 +0.998 +0.163` |
| all-aux-tolerance | 0.00 | 223-60 | reference | +0.000% | `+0.000 +0.000 +0.000 +0.000 +0.000 +0.000` |
| all-aux-tolerance | 0.00 | 223-72 | formations-1-2-dynamic-trust-0.30 | +0.516% | `+0.516 +0.000 +0.001 +1.882 +1.152 +1.183` |
| all-aux-tolerance | 0.00 | 227-60 | reference | +0.000% | `+0.000 +0.000 +0.000 +0.000 +0.000 +0.000` |
| all-aux-tolerance | 0.00 | 227-72 | formations-2-4-dynamic-trust-0.30 | +0.751% | `+0.751 +0.001 +0.003 +0.833 +2.479 +2.580` |
| all-aux-tolerance | 0.25 | 211-60 | formations-1-2-dynamic-trust-0.30 | +1.591% | `+1.591 +0.000 +5.494 +2.807 +0.696 +0.742` |
| all-aux-tolerance | 0.25 | 211-72 | formation-2-dynamic-trust-0.70 | +0.024% | `+0.024 +0.000 +0.100 +0.006 +0.998 +0.163` |
| all-aux-tolerance | 0.25 | 223-60 | reference | +0.000% | `+0.000 +0.000 +0.000 +0.000 +0.000 +0.000` |
| all-aux-tolerance | 0.25 | 223-72 | formations-1-2-dynamic-trust-0.30 | +0.516% | `+0.516 +0.000 +0.001 +1.882 +1.152 +1.183` |
| all-aux-tolerance | 0.25 | 227-60 | reference | +0.000% | `+0.000 +0.000 +0.000 +0.000 +0.000 +0.000` |
| all-aux-tolerance | 0.25 | 227-72 | formations-2-4-dynamic-trust-0.30 | +0.751% | `+0.751 +0.001 +0.003 +0.833 +2.479 +2.580` |
| all-aux-tolerance | 0.50 | 211-60 | formations-1-2-dynamic-trust-0.30 | +1.591% | `+1.591 +0.000 +5.494 +2.807 +0.696 +0.742` |
| all-aux-tolerance | 0.50 | 211-72 | formation-2-dynamic-trust-0.70 | +0.024% | `+0.024 +0.000 +0.100 +0.006 +0.998 +0.163` |
| all-aux-tolerance | 0.50 | 223-60 | reference | +0.000% | `+0.000 +0.000 +0.000 +0.000 +0.000 +0.000` |
| all-aux-tolerance | 0.50 | 223-72 | formations-1-2-dynamic-trust-0.30 | +0.516% | `+0.516 +0.000 +0.001 +1.882 +1.152 +1.183` |
| all-aux-tolerance | 0.50 | 227-60 | reference | +0.000% | `+0.000 +0.000 +0.000 +0.000 +0.000 +0.000` |
| all-aux-tolerance | 0.50 | 227-72 | formations-2-4-dynamic-trust-0.30 | +0.751% | `+0.751 +0.001 +0.003 +0.833 +2.479 +2.580` |
| all-aux-tolerance | 1.00 | 211-60 | formations-2-4-dynamic-trust-0.30 | +2.323% | `+2.323 -0.786 +5.494 +0.958 +0.225 -0.521` |
| all-aux-tolerance | 1.00 | 211-72 | formations-3-4-dynamic-trust-0.30 | +2.892% | `+2.892 -0.001 -0.003 +0.479 -0.642 -0.672` |
| all-aux-tolerance | 1.00 | 223-60 | formations-3-4-dynamic-trust-0.30 | +1.973% | `+1.973 -0.010 +1.993 -0.887 -0.017 -0.017` |
| all-aux-tolerance | 1.00 | 223-72 | formations-1-2-dynamic-trust-0.30 | +0.516% | `+0.516 +0.000 +0.001 +1.882 +1.152 +1.183` |
| all-aux-tolerance | 1.00 | 227-60 | formation-4-dynamic-trust-0.30 | +2.213% | `+2.213 +0.000 +2.314 -0.748 -0.041 -0.014` |
| all-aux-tolerance | 1.00 | 227-72 | formations-2-4-dynamic-trust-0.30 | +0.751% | `+0.751 +0.001 +0.003 +0.833 +2.479 +2.580` |

## Evidence boundary

This audit reselects actions from already opened, truth-scored v13 target matrices. It diagnoses constraint headroom only; it cannot validate a policy, authorize a relaxed deployment gate, or support M24/X36/final-seed claims.

Tau is a descriptive percentage-point tolerance applied after outcomes are known. The scan does not alter the registered strict gate, and no row is a deployable policy result.
