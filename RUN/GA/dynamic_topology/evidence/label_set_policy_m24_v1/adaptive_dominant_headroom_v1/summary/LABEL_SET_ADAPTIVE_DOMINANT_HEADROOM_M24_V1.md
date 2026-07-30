# M24 adaptive-dominant H=3 headroom audit

- Mean temporal / tail-safe adaptive oracle E-OSPA: `14.190508 / 12.322612`
- Tail-safe incremental gain: `13.163%` (gate `2.000%`)
- States with an admissible new action: `4 / 4`
- Persistent balanced gain vs temporal oracle: `5.535%`
- Persistent balanced positive states vs CCC: `4 / 4`
- Persistent balanced aggregate worst gain: `+0.698%`
- Persistent balanced aggregate consensus regression: `+0.303%`
- Persistent unbalanced gain / tail-admissible states: `14.842% / 3`
- Hard / action-headroom / persistent-balanced gates: `1 / 1 / 1`

| Seed | Source | Temporal | Safe new oracle | Best safe | Incremental | New admissible | Balanced | Bal. vs CCC | Unbalanced | U tail-safe |
|--:|:--|--:|--:|:--|--:|--:|--:|--:|--:|--:|
| 11 | learned | 14.897434 | 12.918320 | `D-U-D-U-D-U` | +13.285% | 4 | 14.913963 | +2.980% | 12.918320 | 1 |
| 19 | ccw | 19.440435 | 17.362118 | `D-B-D-B-D-B` | +10.691% | 1 | 17.362118 | +12.561% | 16.101727 | 0 |
| 23 | ccw | 5.896317 | 4.932660 | `D-T-D-T-D-T` | +16.343% | 5 | 5.404500 | +26.388% | 5.239797 | 1 |
| 27 | learned | 16.527847 | 14.077348 | `D-U-D-U-D-U` | +14.826% | 6 | 15.939628 | +8.833% | 14.077348 | 1 |

## Decision

The adaptive-dominant action space passes the registered tail-safe incremental-headroom gate. The persistent balanced controller also passes its four-state progression gate. Next: `full-six-seed-training-validation-of-persistent-balanced-default-plus-lcb-gated-concentrated-upgrade`.

## Boundary

This opened-training audit establishes action-space headroom and a promising truth-free persistent balanced controller on four registered behavior states. The safe oracle uses future return only to select among already truth-free actions. It does not establish a learned gate, six-seed training stability, development, held-out M24 or X36 performance.
